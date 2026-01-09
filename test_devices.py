#!/usr/bin/env python3
"""
Simple script to test USB webcam and IMU connectivity
This will capture frames from the webcam and attempt to read IMU data
"""

import cv2
import serial
import time
import numpy as np

def test_webcam(device_id=4, save_images=True, num_frames=30):
    """Test the USB webcam"""
    print(f"Testing webcam on /dev/video{device_id}...")
    
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera /dev/video{device_id}")
        return False
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print(f"Camera opened successfully!")
    print(f"Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print(f"FPS: {cap.get(cv2.CAP_PROP_FPS)}")
    
    frame_count = 0
    start_time = time.time()
    
    print(f"\nCapturing {num_frames} frames (no display mode)...")
    
    while frame_count < num_frames:
        ret, frame = cap.read()
        
        if not ret:
            print("ERROR: Failed to capture frame")
            break
        
        frame_count += 1
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Save first, middle, and last frames
        if save_images and (frame_count == 1 or frame_count == num_frames // 2 or frame_count == num_frames):
            filename = f"test_frame_{frame_count:03d}.jpg"
            cv2.imwrite(filename, gray)
            print(f"Saved {filename}")
        
        if frame_count % 10 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"Frame {frame_count}/{num_frames} - FPS: {fps:.1f}")
    
    elapsed = time.time() - start_time
    fps = frame_count / elapsed if elapsed > 0 else 0
    
    cap.release()
    
    print(f"\nCaptured {frame_count} frames in {elapsed:.2f} seconds")
    print(f"Average FPS: {fps:.2f}")
    
    return True

def test_imu(port='/dev/ttyUSB0', test_baudrates=[115200, 9600, 57600, 38400]):
    """Test the USB IMU with different baudrates"""
    print(f"\nTesting IMU on {port}...")
    
    best_baudrate = None
    best_readable_count = 0
    
    for baudrate in test_baudrates:
        print(f"\n--- Testing baudrate: {baudrate} ---")
        
        try:
            ser = serial.Serial(port, baudrate, timeout=0.5)
            time.sleep(0.5)  # Let the connection stabilize
            
            line_count = 0
            readable_count = 0
            binary_count = 0
            start_time = time.time()
            
            while time.time() - start_time < 3:  # Test for 3 seconds per baudrate
                if ser.in_waiting > 0:
                    try:
                        # Try reading as text
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            line_count += 1
                            # Check if line is mostly readable ASCII
                            printable = sum(c.isprintable() or c.isspace() for c in line)
                            if printable > len(line) * 0.5:  # More than 50% printable
                                readable_count += 1
                                if line_count <= 3:  # Show first few lines
                                    print(f"  [{line_count}] {line[:80]}")
                            else:
                                binary_count += 1
                    except Exception as e:
                        binary_count += 1
            
            ser.close()
            
            print(f"  Total lines: {line_count}, Readable: {readable_count}, Binary: {binary_count}")
            
            if readable_count > best_readable_count:
                best_readable_count = readable_count
                best_baudrate = baudrate
            
        except serial.SerialException as e:
            print(f"  ERROR: Cannot open IMU port at {baudrate}: {e}")
            continue
    
    if best_baudrate:
        print(f"\n✓ Best baudrate appears to be: {best_baudrate} ({best_readable_count} readable lines)")
        print(f"\nRecommendation: Use baudrate {best_baudrate} for IMU")
        
        # Do a final detailed capture with best baudrate
        print(f"\nCapturing sample data at {best_baudrate}...")
        try:
            ser = serial.Serial(port, best_baudrate, timeout=1)
            time.sleep(0.5)
            
            samples = []
            start_time = time.time()
            while time.time() - start_time < 5 and len(samples) < 20:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        samples.append(line)
            
            ser.close()
            
            print(f"\nSample IMU data (first 10 lines):")
            for i, sample in enumerate(samples[:10], 1):
                print(f"  {i}: {sample}")
                
        except Exception as e:
            print(f"Error in final capture: {e}")
        
        return True
    else:
        print("\n✗ Could not find a suitable baudrate")
        print("\nYour IMU might be:")
        print("  1. Using a binary protocol (not text-based)")
        print("  2. Require specific initialization commands")
        print("  3. Using a different baudrate not tested")
        print("\nPlease check your IMU's documentation.")
        return False
        
    except serial.SerialException as e:
        print(f"ERROR: Cannot open IMU port {port}")
        print(f"Error: {e}")
        print("\nMake sure:")
        print(f"  1. IMU is connected to {port}")
        print(f"  2. You have permission to access the port (add user to 'dialout' group)")
        print(f"     sudo usermod -a -G dialout $USER")
        return False

def main():
    print("=" * 60)
    print("USB Webcam and IMU Test")
    print("=" * 60)
    
    # Test IMU first
    imu_ok = test_imu('/dev/ttyUSB0')
    
    # Test webcam
    webcam_ok = test_webcam(4)  # Logitech BRIO is at /dev/video4
    
    print("\n" + "=" * 60)
    print("Test Summary:")
    print(f"  IMU:    {'✓ OK' if imu_ok else '✗ FAILED'}")
    print(f"  Webcam: {'✓ OK' if webcam_ok else '✗ FAILED'}")
    print("=" * 60)

if __name__ == "__main__":
    main()
