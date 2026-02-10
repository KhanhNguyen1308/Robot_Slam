"""
Flask Web Server for Robot Control
Provides web interface and REST API for robot control
"""
from flask import Flask, render_template, Response, jsonify, request
from flask_cors import CORS
import cv2
import numpy as np
import threading
import time
import logging
import json
from typing import Optional

logger = logging.getLogger(__name__)


class RobotWebServer:
    """
    Web server for robot control and monitoring
    """
    
    def __init__(self, 
                 robot_controller,
                 stereo_camera,
                 host: str = "0.0.0.0",
                 port: int = 5000):
        
        self.robot = robot_controller
        self.camera = stereo_camera
        self.host = host
        self.port = port
        
        # Flask app
        self.app = Flask(__name__)
        CORS(self.app)
        
        # Streaming
        self.stream_enabled = True
        self.stream_fps = 15
        self.last_frame = None
        self.frame_lock = threading.Lock()
        
        # Setup routes
        self._setup_routes()
        
        # Server thread
        self.server_thread = None
        self.running = False
    
    def _setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            """Main control page"""
            return render_template('index.html')
        
        @self.app.route('/api/status')
        def get_status():
            """Get robot status"""
            return jsonify({
                "motor_controller": self.robot.motor_controller.get_stats(),
                "slam": {
                    "pose": self.robot.slam_pose.tolist() if self.robot.slam_pose is not None else None,
                    "tracking": self.robot.slam_tracking_state
                },
                "camera": self.camera.get_camera_info(),
                "timestamp": time.time()
            })
        
        @self.app.route('/api/velocity', methods=['POST'])
        def set_velocity():
            """Set robot velocity"""
            data = request.json
            linear = data.get('linear', 0)
            angular = data.get('angular', 0)
            
            success = self.robot.set_velocity(linear, angular)
            
            return jsonify({
                "success": success,
                "linear": linear,
                "angular": angular
            })
        
        @self.app.route('/api/enable', methods=['POST'])
        def enable_motors():
            """Enable motors"""
            success = self.robot.enable_motors()
            return jsonify({"success": success})
        
        @self.app.route('/api/disable', methods=['POST'])
        def disable_motors():
            """Disable motors"""
            success = self.robot.disable_motors()
            return jsonify({"success": success})
        
        @self.app.route('/api/stop', methods=['POST'])
        def emergency_stop():
            """Emergency stop"""
            success = self.robot.emergency_stop()
            return jsonify({"success": success})
        
        @self.app.route('/video/left')
        def video_left():
            """Stream left camera"""
            return Response(
                self._generate_stream('left'),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/video/right')
        def video_right():
            """Stream right camera"""
            return Response(
                self._generate_stream('right'),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/video/stereo')
        def video_stereo():
            """Stream stereo (side by side)"""
            return Response(
                self._generate_stream('stereo'),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.app.route('/api/trajectory')
        def get_trajectory():
            """Get SLAM trajectory"""
            return jsonify({
                "trajectory": self.robot.get_trajectory(),
                "length": len(self.robot.trajectory)
            })
    
    def _generate_stream(self, mode: str):
        """Generate video stream"""
        while self.stream_enabled:
            if self.robot.current_frame_left is None:
                time.sleep(0.1)
                continue
            
            try:
                if mode == 'left':
                    frame = self.robot.current_frame_left.copy()
                elif mode == 'right':
                    frame = self.robot.current_frame_right.copy()
                elif mode == 'stereo':
                    left = self.robot.current_frame_left
                    right = self.robot.current_frame_right
                    frame = np.hstack([left, right])
                else:
                    continue
                
                # Add overlay info
                frame = self._add_overlay(frame, mode)
                
                # Encode as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
            except Exception as e:
                logger.error(f"Stream error: {e}")
            
            time.sleep(1.0 / self.stream_fps)
    
    def _add_overlay(self, frame: np.ndarray, mode: str) -> np.ndarray:
        """Add status overlay to frame"""
        overlay = frame.copy()
        
        # Status text
        status_text = [
            f"Tracking: {self.robot.slam_tracking_state}",
            f"Motors: {'ON' if self.robot.motor_controller.enabled else 'OFF'}",
            f"FPS: {self.robot.fps:.1f}"
        ]
        
        # Draw background
        cv2.rectangle(overlay, (5, 5), (250, 80), (0, 0, 0), -1)
        
        # Draw text
        y = 25
        for text in status_text:
            cv2.putText(overlay, text, (10, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y += 20
        
        # Blend
        frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
        
        return frame
    
    def start(self):
        """Start web server in background thread"""
        if self.running:
            return
        
        self.running = True
        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=True
        )
        self.server_thread.start()
        
        logger.info(f"Web server starting on http://{self.host}:{self.port}")
    
    def _run_server(self):
        """Run Flask server"""
        self.app.run(
            host=self.host,
            port=self.port,
            debug=False,
            threaded=True,
            use_reloader=False
        )
    
    def stop(self):
        """Stop web server"""
        self.running = False
        self.stream_enabled = False
        logger.info("Web server stopped")


# HTML Template
INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            color: #4CAF50;
        }
        .video-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-bottom: 20px;
        }
        .video-container {
            background: #000;
            border-radius: 8px;
            overflow: hidden;
        }
        .video-container img {
            width: 100%;
            height: auto;
            display: block;
        }
        .controls {
            background: #2a2a2a;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        .button-group {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        button {
            padding: 12px 24px;
            font-size: 16px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            transition: background 0.3s;
        }
        .btn-primary {
            background: #4CAF50;
            color: white;
        }
        .btn-primary:hover {
            background: #45a049;
        }
        .btn-danger {
            background: #f44336;
            color: white;
        }
        .btn-danger:hover {
            background: #da190b;
        }
        .joystick {
            width: 200px;
            height: 200px;
            background: #333;
            border-radius: 50%;
            margin: 20px auto;
            position: relative;
            touch-action: none;
        }
        .joystick-handle {
            width: 60px;
            height: 60px;
            background: #4CAF50;
            border-radius: 50%;
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: pointer;
        }
        .status {
            background: #2a2a2a;
            padding: 15px;
            border-radius: 8px;
            font-family: monospace;
        }
        .status-item {
            margin: 5px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Control Panel</h1>
        
        <div class="video-grid">
            <div class="video-container">
                <h3 style="text-align: center; margin: 10px;">Left Camera</h3>
                <img src="/video/left" alt="Left Camera">
            </div>
            <div class="video-container">
                <h3 style="text-align: center; margin: 10px;">Right Camera</h3>
                <img src="/video/right" alt="Right Camera">
            </div>
        </div>
        
        <div class="controls">
            <h2>Motor Control</h2>
            <div class="button-group">
                <button class="btn-primary" onclick="enableMotors()">Enable Motors</button>
                <button class="btn-danger" onclick="disableMotors()">Disable Motors</button>
                <button class="btn-danger" onclick="emergencyStop()">EMERGENCY STOP</button>
            </div>
            
            <div class="joystick" id="joystick">
                <div class="joystick-handle" id="handle"></div>
            </div>
        </div>
        
        <div class="status" id="status">
            <h3>Status</h3>
            <div id="status-content">Loading...</div>
        </div>
    </div>
    
    <script>
        // Joystick control
        const joystick = document.getElementById('joystick');
        const handle = document.getElementById('handle');
        let isDragging = false;
        
        function handleMove(e) {
            if (!isDragging) return;
            
            const rect = joystick.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            let x = (e.clientX || e.touches[0].clientX) - rect.left - centerX;
            let y = (e.clientY || e.touches[0].clientY) - rect.top - centerY;
            
            const distance = Math.sqrt(x*x + y*y);
            const maxDistance = rect.width / 2 - 30;
            
            if (distance > maxDistance) {
                x = x / distance * maxDistance;
                y = y / distance * maxDistance;
            }
            
            handle.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
            
            // Send velocity command
            const linear = -y / maxDistance * 0.5;  // max 0.5 m/s
            const angular = -x / maxDistance * 2.0;  // max 2.0 rad/s
            
            setVelocity(linear, angular);
        }
        
        handle.addEventListener('mousedown', () => isDragging = true);
        handle.addEventListener('touchstart', () => isDragging = true);
        
        document.addEventListener('mouseup', () => {
            isDragging = false;
            handle.style.transform = 'translate(-50%, -50%)';
            setVelocity(0, 0);
        });
        
        document.addEventListener('touchend', () => {
            isDragging = false;
            handle.style.transform = 'translate(-50%, -50%)';
            setVelocity(0, 0);
        });
        
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('touchmove', handleMove);
        
        // API calls
        function setVelocity(linear, angular) {
            fetch('/api/velocity', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear, angular})
            });
        }
        
        function enableMotors() {
            fetch('/api/enable', {method: 'POST'})
                .then(r => r.json())
                .then(data => alert(data.success ? 'Motors enabled' : 'Failed'));
        }
        
        function disableMotors() {
            fetch('/api/disable', {method: 'POST'})
                .then(r => r.json())
                .then(data => alert(data.success ? 'Motors disabled' : 'Failed'));
        }
        
        function emergencyStop() {
            fetch('/api/stop', {method: 'POST'})
                .then(r => r.json())
                .then(data => alert('Emergency stop activated'));
        }
        
        // Status update
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('status-content').innerHTML = `
                        <div class="status-item">Motors: ${data.motor_controller.enabled ? 'ON' : 'OFF'}</div>
                        <div class="status-item">SLAM Tracking: ${data.slam.tracking}</div>
                        <div class="status-item">Commands Sent: ${data.motor_controller.commands_sent}</div>
                        <div class="status-item">Errors: ${data.motor_controller.errors}</div>
                    `;
                });
        }
        
        setInterval(updateStatus, 1000);
        updateStatus();
    </script>
</body>
</html>
"""
