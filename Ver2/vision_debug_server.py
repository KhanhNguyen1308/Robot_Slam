#!/usr/bin/env python3
"""
Vision Processing Debug Web Interface
Visualizes stereo vision pipeline steps: cameras → grayscale → depth → features → matching
"""
import sys
import cv2
import numpy as np
from flask import Flask, render_template, Response, jsonify
import threading
import time
import logging
from io import BytesIO
import base64

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

class VisionDebugger:
    """Stereo vision pipeline debugger"""
    
    def __init__(self, left_cam_id=1, right_cam_id=0, width=640, height=480):
        self.left_cam_id = left_cam_id
        self.right_cam_id = right_cam_id
        self.width = width
        self.height = height
        
        # Camera captures
        self.left_cap = None
        self.right_cap = None
        
        # Processing results
        self.left_frame = None
        self.right_frame = None
        self.left_gray = None
        self.right_gray = None
        self.disparity = None
        self.depth = None
        self.fast_keypoints = []
        self.left_keypoints = []
        self.right_keypoints = []
        self.matches = []
        
        # Feature detectors
        self.fast = cv2.FastFeatureDetector_create(threshold=20)  # Lower threshold = more features
        self.orb = cv2.ORB_create(nfeatures=1000)  # Increased for better matching
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Stereo matcher
        self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
        # Thread control
        self.running = False
        self.lock = threading.Lock()
        
        # Stats
        self.fps = 0
        self.last_process_time = 0
        
    def init_cameras(self):
        """Initialize stereo cameras"""
        logger.info(f"Opening cameras: left={self.left_cam_id}, right={self.right_cam_id}")
        
        self.left_cap = cv2.VideoCapture(self.left_cam_id)
        self.right_cap = cv2.VideoCapture(self.right_cam_id)
        
        if not self.left_cap.isOpened() or not self.right_cap.isOpened():
            logger.error("Failed to open cameras!")
            return False
        
        # Set resolution
        self.left_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.left_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.right_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.right_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        logger.info("Cameras initialized successfully")
        return True
    
    def process_frame(self):
        """Process one frame through the entire pipeline"""
        start_time = time.time()
        
        # Step 1: Capture from cameras
        ret_left, left_frame = self.left_cap.read()
        ret_right, right_frame = self.right_cap.read()
        
        if not ret_left or not ret_right:
            return False
        
        with self.lock:
            self.left_frame = left_frame.copy()
            self.right_frame = right_frame.copy()
            
            # Step 2: Convert to grayscale
            self.left_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            self.right_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
            
            # Step 3: Compute disparity/depth
            disparity = self.stereo.compute(self.left_gray, self.right_gray)
            self.disparity = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            
            # Simple depth estimate (placeholder - needs calibration)
            self.depth = disparity.astype(np.float32)
            self.depth[self.depth <= 0] = 0.1  # Avoid division by zero
            
            # Step 4: Detect features with FAST (for visualization)
            fast_kp = self.fast.detect(self.left_gray, None)
            
            # Step 5 & 6: Use ORB for both detection and description
            # Using ORB detectAndCompute for both images ensures compatible descriptors
            self.left_keypoints, left_desc = self.orb.detectAndCompute(self.left_gray, None)
            self.right_keypoints, right_desc = self.orb.detectAndCompute(self.right_gray, None)
            
            # Also compute ORB on FAST keypoints for comparison (for FAST visualization)
            if len(fast_kp) > 0:
                fast_kp_with_orb, _ = self.orb.compute(self.left_gray, fast_kp)
                # Store FAST keypoints for visualization in step 4
                self.fast_keypoints = fast_kp_with_orb if fast_kp_with_orb else fast_kp
            else:
                self.fast_keypoints = []
            
            # Step 7: Match features (Hamming distance)
            if left_desc is not None and right_desc is not None and len(left_desc) > 0 and len(right_desc) > 0:
                try:
                    self.matches = self.bf_matcher.match(left_desc, right_desc)
                    self.matches = sorted(self.matches, key=lambda x: x.distance)[:50]
                except Exception as e:
                    logger.warning(f"Matching failed: {e}")
                    self.matches = []
            else:
                self.matches = []
                if left_desc is None or len(left_desc) == 0:
                    logger.warning("No left descriptors found")
                if right_desc is None or len(right_desc) == 0:
                    logger.warning("No right descriptors found")
        
        # Update stats
        self.last_process_time = time.time() - start_time
        self.fps = 1.0 / self.last_process_time if self.last_process_time > 0 else 0
        
        # Log stats periodically (every 30 frames)
        if hasattr(self, 'frame_count'):
            self.frame_count += 1
        else:
            self.frame_count = 0
            
        if self.frame_count % 30 == 0:
            logger.info(f"FPS: {self.fps:.1f}, Keypoints L/R: {len(self.left_keypoints)}/{len(self.right_keypoints)}, Matches: {len(self.matches)}")
        
        return True
    
    def run_processing_loop(self):
        """Main processing loop"""
        logger.info("Starting processing loop...")
        
        while self.running:
            self.process_frame()
            time.sleep(0.03)  # ~30 FPS max
    
    def start(self):
        """Start the vision debugger"""
        if not self.init_cameras():
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self.run_processing_loop, daemon=True)
        self.thread.start()
        
        logger.info("Vision debugger started")
        return True
    
    def stop(self):
        """Stop the vision debugger"""
        self.running = False
        
        if self.left_cap:
            self.left_cap.release()
        if self.right_cap:
            self.right_cap.release()
        
        logger.info("Vision debugger stopped")
    
    def frame_to_base64(self, frame):
        """Convert frame to base64 for web display"""
        if frame is None:
            return None
        
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        return base64.b64encode(buffer).decode('utf-8')
    
    def get_visualizations(self):
        """Get all visualization steps"""
        with self.lock:
            # 1. Original frames
            left_orig = self.frame_to_base64(self.left_frame)
            right_orig = self.frame_to_base64(self.right_frame)
            
            # 2. Grayscale
            left_gray_vis = None
            right_gray_vis = None
            if self.left_gray is not None:
                left_gray_vis = self.frame_to_base64(cv2.cvtColor(self.left_gray, cv2.COLOR_GRAY2BGR))
            if self.right_gray is not None:
                right_gray_vis = self.frame_to_base64(cv2.cvtColor(self.right_gray, cv2.COLOR_GRAY2BGR))
            
            # 3. Disparity/Depth
            disparity_vis = None
            depth_vis = None
            if self.disparity is not None:
                disparity_color = cv2.applyColorMap(self.disparity, cv2.COLORMAP_JET)
                disparity_vis = self.frame_to_base64(disparity_color)
                
                # Depth visualization
                depth_norm = cv2.normalize(self.depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)
                depth_vis = self.frame_to_base64(depth_color)
            
            # 4. FAST keypoints
            fast_vis = None
            if self.left_frame is not None and hasattr(self, 'fast_keypoints') and len(self.fast_keypoints) > 0:
                fast_img = self.left_frame.copy()
                for kp in self.fast_keypoints[:100]:  # Limit to 100 for visibility
                    x, y = int(kp.pt[0]), int(kp.pt[1])
                    cv2.circle(fast_img, (x, y), 3, (0, 255, 0), 1)
                fast_vis = self.frame_to_base64(fast_img)
            
            # 5. ORB descriptors visualization (simplified)
            orb_vis = None
            if self.left_frame is not None and len(self.left_keypoints) > 0:
                orb_img = self.left_frame.copy()
                cv2.drawKeypoints(orb_img, self.left_keypoints[:100], orb_img, 
                                 color=(255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                orb_vis = self.frame_to_base64(orb_img)
            
            # 6. Hamming matching - separate visualizations
            matches_left_vis = None
            matches_right_vis = None
            matches_combined_vis = None
            
            # Left camera with ORB keypoints
            if self.left_frame is not None and len(self.left_keypoints) > 0:
                left_match_img = self.left_frame.copy()
                cv2.drawKeypoints(left_match_img, self.left_keypoints, left_match_img,
                                 color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                matches_left_vis = self.frame_to_base64(left_match_img)
            
            # Right camera with ORB keypoints
            if self.right_frame is not None and len(self.right_keypoints) > 0:
                right_match_img = self.right_frame.copy()
                cv2.drawKeypoints(right_match_img, self.right_keypoints, right_match_img,
                                 color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                matches_right_vis = self.frame_to_base64(right_match_img)
            
            # Combined matching visualization
            if self.left_frame is not None and self.right_frame is not None and len(self.matches) > 0:
                match_img = cv2.drawMatches(
                    self.left_frame, self.left_keypoints,
                    self.right_frame, self.right_keypoints,
                    self.matches[:20], None,  # Show top 20 matches
                    matchColor=(0, 255, 0),
                    singlePointColor=(255, 0, 0),
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )
                matches_combined_vis = self.frame_to_base64(match_img)
            
            return {
                'left_original': left_orig,
                'right_original': right_orig,
                'left_gray': left_gray_vis,
                'right_gray': right_gray_vis,
                'disparity': disparity_vis,
                'depth': depth_vis,
                'fast_features': fast_vis,
                'orb_features': orb_vis,
                'matches_left': matches_left_vis,
                'matches_right': matches_right_vis,
                'matches_combined': matches_combined_vis,
                'stats': {
                    'fps': round(self.fps, 1),
                    'process_time_ms': round(self.last_process_time * 1000, 1),
                    'num_fast_keypoints': len(self.fast_keypoints) if hasattr(self, 'fast_keypoints') else 0,
                    'num_keypoints_left': len(self.left_keypoints),
                    'num_keypoints_right': len(self.right_keypoints),
                    'num_matches': len(self.matches)
                }
            }


# Global debugger instance
debugger = None


@app.route('/')
def index():
    """Main page"""
    return render_template('vision_debug.html')


@app.route('/api/images')
def get_images():
    """Get all processed images"""
    if debugger is None:
        return jsonify({'error': 'Debugger not initialized'}), 500
    
    return jsonify(debugger.get_visualizations())


def main():
    """Main entry point"""
    global debugger
    
    logger.info("=" * 70)
    logger.info("Vision Processing Debug Server")
    logger.info("=" * 70)
    
    # Create debugger
    debugger = VisionDebugger(left_cam_id=1, right_cam_id=0)
    
    if not debugger.start():
        logger.error("Failed to start vision debugger")
        return 1
    
    try:
        logger.info("Starting web server on http://0.0.0.0:5001")
        logger.info("Open browser and navigate to: http://<jetson-ip>:5001")
        app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
    finally:
        debugger.stop()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
