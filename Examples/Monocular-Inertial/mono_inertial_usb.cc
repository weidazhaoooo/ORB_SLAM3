/**
* This file is part of ORB-SLAM3
*
* Mono-Inertial example using USB webcam and WitMotion USB IMU
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <System.h>

using namespace std;

// Global flag for clean exit
bool b_continue_session = true;

// Signal handler for Ctrl+C
void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

// Structure to hold IMU data
struct ImuData {
    double timestamp;
    float ax, ay, az;  // accelerometer m/s²
    float gx, gy, gz;  // gyroscope rad/s
};

// Shared data structures
mutex imu_mutex;
mutex img_mutex;
condition_variable cond_image_rec;

queue<ImuData> imu_buffer;
cv::Mat current_frame;
double current_frame_timestamp = 0;
bool image_ready = false;

// WitMotion IMU packet types
const uint8_t WITMOTION_HEADER = 0x55;
const uint8_t WITMOTION_ACC = 0x51;
const uint8_t WITMOTION_GYRO = 0x52;

// Parse WitMotion IMU packet
bool parseWitMotionPacket(const uint8_t* packet, float& ax, float& ay, float& az, 
                          float& gx, float& gy, float& gz) {
    if (packet[0] != WITMOTION_HEADER) {
        return false;
    }
    
    uint8_t type = packet[1];
    
    // Extract raw data (little-endian)
    int16_t raw_x = (int16_t)(packet[2] | (packet[3] << 8));
    int16_t raw_y = (int16_t)(packet[4] | (packet[5] << 8));
    int16_t raw_z = (int16_t)(packet[6] | (packet[7] << 8));
    
    if (type == WITMOTION_ACC) {
        // Accelerometer: ±16g range
        ax = raw_x / 32768.0f * 16.0f * 9.8f;  // m/s²
        ay = raw_y / 32768.0f * 16.0f * 9.8f;
        az = raw_z / 32768.0f * 16.0f * 9.8f;
        return true;
    } else if (type == WITMOTION_GYRO) {
        // Gyroscope: ±2000°/s range
        gx = raw_x / 32768.0f * 2000.0f * M_PI / 180.0f;  // rad/s
        gy = raw_y / 32768.0f * 2000.0f * M_PI / 180.0f;
        gz = raw_z / 32768.0f * 2000.0f * M_PI / 180.0f;
        return true;
    }
    
    return false;
}

// Function to read WitMotion IMU data from serial port
void readImuData(const string& serial_port, int baudrate) {
    // Open serial port
    int fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        cerr << "Failed to open IMU serial port: " << serial_port << endl;
        return;
    }
    
    // Configure serial port
    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baudrate
    speed_t speed = B230400;  // WitMotion default
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Raw mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    tcsetattr(fd, TCSANOW, &options);
    
    cout << "WitMotion IMU opened: " << serial_port << " @ " << baudrate << endl;
    
    uint8_t buffer[11];
    int buffer_idx = 0;
    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    bool has_acc = false;
    bool has_gyro = false;
    
    while (b_continue_session) {
        uint8_t byte;
        int n = read(fd, &byte, 1);
        
        if (n > 0) {
            // Look for packet header
            if (buffer_idx == 0 && byte == WITMOTION_HEADER) {
                buffer[buffer_idx++] = byte;
            } else if (buffer_idx > 0) {
                buffer[buffer_idx++] = byte;
                
                // Complete packet
                if (buffer_idx >= 11) {
                    float ax, ay, az, gx, gy, gz;
                    
                    if (parseWitMotionPacket(buffer, ax, ay, az, gx, gy, gz)) {
                        if (buffer[1] == WITMOTION_ACC) {
                            acc_x = ax;
                            acc_y = ay;
                            acc_z = az;
                            has_acc = true;
                        } else if (buffer[1] == WITMOTION_GYRO) {
                            gyro_x = gx;
                            gyro_y = gy;
                            gyro_z = gz;
                            has_gyro = true;
                        }
                        
                        // When we have both acc and gyro, push to buffer
                        if (has_acc && has_gyro) {
                            ImuData imu;
                            imu.timestamp = chrono::duration_cast<chrono::duration<double>>(
                                chrono::system_clock::now().time_since_epoch()).count();
                            imu.ax = acc_x;
                            imu.ay = acc_y;
                            imu.az = acc_z;
                            imu.gx = gyro_x;
                            imu.gy = gyro_y;
                            imu.gz = gyro_z;
                            
                            unique_lock<mutex> lock(imu_mutex);
                            imu_buffer.push(imu);
                            
                            // Keep buffer size reasonable
                            while (imu_buffer.size() > 1000) {
                                imu_buffer.pop();
                            }
                            
                            has_acc = false;
                            has_gyro = false;
                        }
                    }
                    
                    buffer_idx = 0;
                }
            }
        }
    }
    
    close(fd);
    cout << "IMU thread stopped" << endl;
}

// Function to read frames from webcam
void readWebcamFrames(int camera_id, double fps) {
    cv::VideoCapture cap(camera_id);
    
    if (!cap.isOpened()) {
        cerr << "Failed to open camera " << camera_id << endl;
        return;
    }
    
    // Set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, fps);
    
    cout << "Camera opened successfully" << endl;
    
    while (b_continue_session) {
        cv::Mat frame;
        if (cap.read(frame)) {
            double timestamp = chrono::duration_cast<chrono::duration<double>>(
                chrono::system_clock::now().time_since_epoch()).count();
            
            unique_lock<mutex> lock(img_mutex);
            frame.copyTo(current_frame);
            current_frame_timestamp = timestamp;
            image_ready = true;
            lock.unlock();
            cond_image_rec.notify_all();
        }
        
        this_thread::sleep_for(chrono::milliseconds(int(1000.0/fps)));
    }
    
    cap.release();
}

int main(int argc, char **argv) {
    if (argc != 4 && argc != 5) {
        cerr << endl
             << "Usage: ./mono_inertial_usb path_to_vocabulary path_to_settings camera_device imu_device [trajectory_file]" << endl
             << "Example: ./mono_inertial_usb Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/USB_Webcam.yaml /dev/video4 /dev/ttyUSB0" << endl;
        return 1;
    }

    string vocab_path = argv[1];
    string settings_path = argv[2];
    string camera_device = argv[3];
    string imu_device = argv[4];
    
    string file_name;
    if (argc == 5) {
        file_name = string(argv[4]);
    }

    // Setup signal handler
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Extract camera ID from device path (e.g., /dev/video4 -> 4)
    int camera_id = 0;
    size_t pos = camera_device.find("video");
    if (pos != string::npos) {
        camera_id = stoi(camera_device.substr(pos + 5));
    }

    // Create SLAM system
    cout << "Creating SLAM system..." << endl;
    ORB_SLAM3::System SLAM(vocab_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true);
    
    float imageScale = SLAM.GetImageScale();
    double fps = 30.0;  // Default FPS
    
    // Start IMU thread
    cout << "Starting IMU thread..." << endl;
    thread imu_thread(readImuData, imu_device, 230400);
    
    // Give IMU time to start
    this_thread::sleep_for(chrono::milliseconds(500));
    
    // Start camera thread
    cout << "Starting camera thread..." << endl;
    thread camera_thread(readWebcamFrames, camera_id, fps);
    
    cout << "System ready. Press Ctrl+C to stop." << endl;
    
    // Main loop
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    double timestamp_last_imu = 0;
    
    while (b_continue_session) {
        cv::Mat im;
        double timestamp_image;
        
        // Wait for new image
        {
            unique_lock<mutex> lock(img_mutex);
            cond_image_rec.wait(lock, []{return image_ready;});
            
            if (!current_frame.empty()) {
                current_frame.copyTo(im);
                timestamp_image = current_frame_timestamp;
                image_ready = false;
            } else {
                continue;
            }
        }
        
        // Get IMU measurements between last and current image
        vImuMeas.clear();
        {
            unique_lock<mutex> lock(imu_mutex);
            while (!imu_buffer.empty()) {
                ImuData imu = imu_buffer.front();
                
                if (imu.timestamp <= timestamp_image) {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                        imu.ax, imu.ay, imu.az,
                        imu.gx, imu.gy, imu.gz,
                        imu.timestamp));
                    imu_buffer.pop();
                } else {
                    break;
                }
            }
        }
        
        // Convert to grayscale if needed
        if (im.channels() == 3) {
            cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
        }
        
        // Scale image if needed
        if (imageScale != 1.0f) {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }
        
        // Track mono-inertial
        if (vImuMeas.size() > 0) {
            SLAM.TrackMonocular(im, timestamp_image, vImuMeas);
        } else {
            // If no IMU data yet, just track with empty vector
            // (system will initialize once IMU data is available)
            SLAM.TrackMonocular(im, timestamp_image, vImuMeas);
        }
        
        // Display image
        cv::imshow("Current Frame", im);
        cv::waitKey(1);
    }
    
    cout << "\nShutting down..." << endl;
    
    // Stop threads
    camera_thread.join();
    // if (imu_thread.joinable()) imu_thread.join();
    
    // Save trajectory if requested
    if (!file_name.empty()) {
        cout << "Saving trajectory to " << file_name << endl;
        SLAM.SaveTrajectoryTUM(file_name);
    }
    
    // Shutdown SLAM
    SLAM.Shutdown();
    
    return 0;
}
