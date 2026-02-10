#!/usr/bin/env python3
"""
System Test Script
Tests all components individually before running the full system
"""
import sys
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_serial():
    """Test RP2040 connection"""
    logger.info("Testing RP2040 connection...")
    
    try:
        from serial_controller import RP2040Controller
        
        controller = RP2040Controller(port='/dev/ttyACM0')
        
        if controller.connect():
            logger.info("✓ RP2040 connected")
            
            # Test enable
            controller.enable()
            time.sleep(0.5)
            
            # Test velocity
            controller.send_velocity(0.1, 0)
            time.sleep(1)
            controller.send_velocity(0, 0)
            
            # Test disable
            controller.disable()
            
            controller.disconnect()
            logger.info("✓ RP2040 test passed")
            return True
        else:
            logger.error("✗ Failed to connect to RP2040")
            return False
            
    except Exception as e:
        logger.error(f"✗ Serial test failed: {e}")
        return False


def test_camera():
    """Test stereo camera"""
    logger.info("Testing cameras...")
    
    try:
        from stereo_camera import StereoCamera
        import cv2
        
        camera = StereoCamera(
            left_id=0,
            right_id=1,
            calibration_file='calibration.npz',
            width=1280,
            height=720
        )
        
        if not camera.calibration_loaded:
            logger.warning("⚠ Calibration not loaded (file not found)")
        else:
            logger.info("✓ Calibration loaded")
        
        if camera.open():
            logger.info("✓ Cameras opened")
            
            # Test frame capture
            camera.start_capture()
            time.sleep(1)
            
            frames = camera.get_frames(timeout=2.0)
            
            if frames:
                frame_left, frame_right, timestamp = frames
                logger.info(f"✓ Captured frames: {frame_left.shape}")
                
                # Save test image
                cv2.imwrite('test_left.jpg', frame_left)
                cv2.imwrite('test_right.jpg', frame_right)
                logger.info("✓ Saved test images")
                
                # Test rectification if calibration available
                if camera.calibration_loaded:
                    rect_left, rect_right = camera.rectify_frames(frame_left, frame_right)
                    cv2.imwrite('test_rect_left.jpg', rect_left)
                    cv2.imwrite('test_rect_right.jpg', rect_right)
                    logger.info("✓ Rectification test passed")
            else:
                logger.error("✗ Failed to capture frames")
                return False
            
            camera.close()
            logger.info("✓ Camera test passed")
            return True
        else:
            logger.error("✗ Failed to open cameras")
            return False
            
    except Exception as e:
        logger.error(f"✗ Camera test failed: {e}")
        return False


def test_orbslam():
    """Test ORB-SLAM3 installation"""
    logger.info("Testing ORB-SLAM3...")
    
    try:
        import os
        
        orbslam_path = '/home/jetson/ORB_SLAM3'
        
        # Check if directory exists
        if not os.path.exists(orbslam_path):
            logger.error(f"✗ ORB-SLAM3 not found at {orbslam_path}")
            return False
        
        # Check vocabulary
        vocab_path = os.path.join(orbslam_path, 'Vocabulary/ORBvoc.txt')
        if not os.path.exists(vocab_path):
            logger.error(f"✗ Vocabulary not found at {vocab_path}")
            return False
        
        logger.info("✓ ORB-SLAM3 directory found")
        logger.info("✓ Vocabulary found")
        
        # Check executable
        executable = os.path.join(orbslam_path, 'Examples/Stereo/stereo_euroc')
        if os.path.exists(executable):
            logger.info("✓ Stereo executable found")
        else:
            logger.warning(f"⚠ Executable not found: {executable}")
            logger.warning("  You may need to build ORB-SLAM3 first")
        
        logger.info("✓ ORB-SLAM3 test passed")
        return True
        
    except Exception as e:
        logger.error(f"✗ ORB-SLAM3 test failed: {e}")
        return False


def test_dependencies():
    """Test Python dependencies"""
    logger.info("Testing dependencies...")
    
    dependencies = [
        'numpy',
        'cv2',
        'serial',
        'flask'
    ]
    
    missing = []
    
    for dep in dependencies:
        try:
            __import__(dep)
            logger.info(f"✓ {dep} installed")
        except ImportError:
            logger.error(f"✗ {dep} not installed")
            missing.append(dep)
    
    if missing:
        logger.error(f"Missing dependencies: {', '.join(missing)}")
        logger.info("Install with: pip install -r requirements.txt")
        return False
    
    logger.info("✓ All dependencies installed")
    return True


def main():
    """Run all tests"""
    logger.info("=" * 60)
    logger.info("Robot System Test")
    logger.info("=" * 60)
    
    results = {
        'Dependencies': test_dependencies(),
        'Serial (RP2040)': test_serial(),
        'Camera': test_camera(),
        'ORB-SLAM3': test_orbslam()
    }
    
    logger.info("=" * 60)
    logger.info("Test Results:")
    logger.info("=" * 60)
    
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        logger.info(f"{test_name:20s}: {status}")
    
    logger.info("=" * 60)
    
    if all(results.values()):
        logger.info("All tests passed! System ready to run.")
        return 0
    else:
        logger.warning("Some tests failed. Check errors above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
