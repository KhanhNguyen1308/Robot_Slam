"""
Flask Web Server for Robot Monitoring and Control
Provides web interface for SLAM visualization and teleoperation
"""
from flask import Flask, render_template, jsonify, request, Response
import cv2
import numpy as np
import logging
import json
import base64
from threading import Lock
import time

logger = logging.getLogger(__name__)

app = Flask(__name__)

# Global state (will be set by main application)
robot_state = {
    'motor_controller': None,
    'slam_system': None,
    'stereo_camera': None,
    'autonomous_mapper': None,
    'imu_sensor': None,
    'latest_frame': None,
    'latest_map': None,
    'lock': Lock()
}


def init_web_server(motor_controller, slam_system, stereo_camera, autonomous_mapper, imu_sensor=None):
    """Initialize web server with robot components"""
    robot_state['motor_controller'] = motor_controller
    robot_state['slam_system'] = slam_system
    robot_state['stereo_camera'] = stereo_camera
    robot_state['autonomous_mapper'] = autonomous_mapper
    robot_state['imu_sensor'] = imu_sensor
    logger.info(f"Web server initialized with robot components (IMU: {imu_sensor is not None})")


@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html')


@app.route('/api/status')
def get_status():
    """Get robot status"""
    with robot_state['lock']:
        motor = robot_state['motor_controller']
        slam = robot_state['slam_system']
        mapper = robot_state['autonomous_mapper']
        imu = robot_state['imu_sensor']
        
        status = {
            'timestamp': time.time(),
            'motor': motor.get_stats() if motor else {},
            'slam': slam.get_stats() if slam else {},
            'mapper': mapper.get_stats() if mapper else {},
            'imu_available': imu is not None,
        }
        
        # Add IMU fusion stats if available
        if slam and hasattr(slam, 'get_statistics'):
            status['fusion'] = slam.get_statistics()
        
        return jsonify(status)


@app.route('/api/pose')
def get_pose():
    """Get current robot pose"""
    with robot_state['lock']:
        slam = robot_state['slam_system']
        
        if slam:
            pose = slam.get_2d_pose()
            if pose:
                x, y, theta = pose
                return jsonify({
                    'x': float(x),
                    'y': float(y),
                    'theta': float(theta)
                })
        
        return jsonify({'x': 0.0, 'y': 0.0, 'theta': 0.0})


@app.route('/api/imu')
def get_imu():
    """Get current IMU data"""
    with robot_state['lock']:
        imu = robot_state['imu_sensor']
        
        if imu:
            accel = imu.get_accel()
            gyro = imu.get_gyro()
            yaw, pitch, roll = imu.get_orientation()
            
            return jsonify({
                'accel': {
                    'x': float(accel[0]),
                    'y': float(accel[1]),
                    'z': float(accel[2])
                },
                'gyro': {
                    'x': float(gyro[0]),
                    'y': float(gyro[1]),
                    'z': float(gyro[2])
                },
                'orientation': {
                    'yaw': float(yaw),
                    'pitch': float(pitch),
                    'roll': float(roll)
                },
                'temperature': float(imu.latest_temp)
            })
        
        return jsonify({'error': 'IMU not available'})


@app.route('/api/motor/enable', methods=['POST'])
def motor_enable():
    """Enable motors"""
    with robot_state['lock']:
        motor = robot_state['motor_controller']
        if motor:
            success = motor.enable()
            return jsonify({'success': success})
        return jsonify({'success': False, 'error': 'Motor controller not initialized'})


@app.route('/api/motor/disable', methods=['POST'])
def motor_disable():
    """Disable motors"""
    with robot_state['lock']:
        motor = robot_state['motor_controller']
        if motor:
            success = motor.disable()
            return jsonify({'success': success})
        return jsonify({'success': False, 'error': 'Motor controller not initialized'})


@app.route('/api/motor/stop', methods=['POST'])
def motor_stop():
    """Emergency stop"""
    with robot_state['lock']:
        motor = robot_state['motor_controller']
        if motor:
            success = motor.stop()
            return jsonify({'success': success})
        return jsonify({'success': False, 'error': 'Motor controller not initialized'})


@app.route('/api/motor/velocity', methods=['POST'])
def set_velocity():
    """Set motor velocities"""
    data = request.json
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    
    with robot_state['lock']:
        motor = robot_state['motor_controller']
        if motor:
            success = motor.send_velocity(linear, angular)
            return jsonify({'success': success})
        return jsonify({'success': False, 'error': 'Motor controller not initialized'})


@app.route('/api/exploration/start', methods=['POST'])
def start_exploration():
    """Start autonomous exploration"""
    with robot_state['lock']:
        mapper = robot_state['autonomous_mapper']
        if mapper:
            mapper.start_exploration()
            return jsonify({'success': True})
        return jsonify({'success': False, 'error': 'Mapper not initialized'})


@app.route('/api/exploration/stop', methods=['POST'])
def stop_exploration():
    """Stop autonomous exploration"""
    with robot_state['lock']:
        mapper = robot_state['autonomous_mapper']
        if mapper:
            mapper.stop_exploration()
            return jsonify({'success': True})
        return jsonify({'success': False, 'error': 'Mapper not initialized'})


@app.route('/api/map/save', methods=['POST'])
def save_map():
    """Save current map"""
    data = request.json
    filepath = data.get('filepath', 'map.npz')
    
    with robot_state['lock']:
        mapper = robot_state['autonomous_mapper']
        if mapper:
            mapper.save_map(filepath)
            return jsonify({'success': True, 'filepath': filepath})
        return jsonify({'success': False, 'error': 'Mapper not initialized'})


@app.route('/api/map/load', methods=['POST'])
def load_map():
    """Load saved map"""
    data = request.json
    filepath = data.get('filepath', 'map.npz')
    
    with robot_state['lock']:
        mapper = robot_state['autonomous_mapper']
        if mapper:
            mapper.load_map(filepath)
            return jsonify({'success': True, 'filepath': filepath})
        return jsonify({'success': False, 'error': 'Mapper not initialized'})


@app.route('/video_feed')
def video_feed():
    """Video streaming route for stereo camera"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_frames():
    """Generate video frames for streaming"""
    while True:
        with robot_state['lock']:
            camera = robot_state['stereo_camera']
            
            if camera:
                left, right, disparity = camera.get_latest_frames()
                
                if left is not None:
                    # Create side-by-side view
                    if right is not None:
                        combined = np.hstack([left, right])
                    else:
                        combined = left
                    
                    # Encode frame
                    ret, buffer = cv2.imencode('.jpg', combined)
                    if ret:
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        time.sleep(0.033)  # ~30 FPS


@app.route('/map_feed')
def map_feed():
    """Map streaming route"""
    return Response(generate_map(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_map():
    """Generate map frames for streaming"""
    while True:
        with robot_state['lock']:
            mapper = robot_state['autonomous_mapper']
            
            if mapper:
                # Get occupancy grid image
                map_img = mapper.grid.get_image()
                
                # Overlay robot position
                if mapper.x is not None:
                    robot_gx, robot_gy = mapper.grid.world_to_grid(mapper.x, mapper.y)
                    if mapper.grid.is_valid(robot_gx, robot_gy):
                        # Draw robot as red circle
                        cv2.circle(map_img, (robot_gx, robot_gy), 5, (0, 0, 255), -1)
                        
                        # Draw heading indicator
                        # Note: Image Y-axis points down, World Y-axis points up
                        # So we need to negate the Y component
                        arrow_len = 10
                        end_x = int(robot_gx + arrow_len * np.cos(mapper.theta))
                        end_y = int(robot_gy - arrow_len * np.sin(mapper.theta))  # Negate Y for image coords
                        cv2.arrowedLine(map_img, (robot_gx, robot_gy), 
                                      (end_x, end_y), (255, 0, 0), 2)
                
                # Overlay goal if exists
                if mapper.planner.current_goal:
                    goal_x, goal_y = mapper.planner.current_goal
                    goal_gx, goal_gy = mapper.grid.world_to_grid(goal_x, goal_y)
                    if mapper.grid.is_valid(goal_gx, goal_gy):
                        cv2.circle(map_img, (goal_gx, goal_gy), 5, (0, 255, 0), -1)
                
                # Resize for display
                scale = 4
                display_img = cv2.resize(map_img, None, fx=scale, fy=scale, 
                                       interpolation=cv2.INTER_NEAREST)
                
                # Encode
                ret, buffer = cv2.imencode('.jpg', display_img)
                if ret:
                    frame = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        
        time.sleep(0.1)  # 10 FPS


def create_html_template():
    """Create HTML template for web interface"""
    html_content = """<!DOCTYPE html>
<html>
<head>
    <title>Robot Control Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            text-align: center;
        }
        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-top: 20px;
        }
        .panel {
            border: 1px solid #ddd;
            border-radius: 5px;
            padding: 15px;
            background-color: #fafafa;
        }
        .panel h2 {
            margin-top: 0;
            color: #555;
        }
        .video-container {
            position: relative;
            width: 100%;
            background-color: #000;
            border-radius: 5px;
            overflow: hidden;
        }
        .video-container img {
            width: 100%;
            height: auto;
            display: block;
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 10px;
        }
        button {
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: background-color 0.3s;
        }
        .btn-primary {
            background-color: #4CAF50;
            color: white;
        }
        .btn-primary:hover {
            background-color: #45a049;
        }
        .btn-danger {
            background-color: #f44336;
            color: white;
        }
        .btn-danger:hover {
            background-color: #da190b;
        }
        .btn-warning {
            background-color: #ff9800;
            color: white;
        }
        .btn-warning:hover {
            background-color: #e68900;
        }
        .status-item {
            margin: 10px 0;
            padding: 10px;
            background-color: white;
            border-radius: 5px;
            border-left: 4px solid #4CAF50;
        }
        .joystick {
            width: 200px;
            height: 200px;
            border: 2px solid #333;
            border-radius: 50%;
            margin: 20px auto;
            position: relative;
            background-color: #e0e0e0;
        }
        .joystick-handle {
            width: 50px;
            height: 50px;
            border-radius: 50%;
            background-color: #4CAF50;
            position: absolute;
            top: 75px;
            left: 75px;
            cursor: move;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Autonomous Mapping Robot Dashboard</h1>
        
        <div class="grid">
            <!-- Camera Feed -->
            <div class="panel">
                <h2>📷 Stereo Camera Feed</h2>
                <div class="video-container">
                    <img src="{{ url_for('video_feed') }}" alt="Stereo Camera">
                </div>
            </div>
            
            <!-- Map View -->
            <div class="panel">
                <h2>🗺️ Occupancy Grid Map</h2>
                <div class="video-container">
                    <img src="{{ url_for('map_feed') }}" alt="Map">
                </div>
            </div>
            
            <!-- Motor Controls -->
            <div class="panel">
                <h2>⚙️ Motor Control</h2>
                <div class="controls">
                    <button class="btn-primary" onclick="enableMotors()">Enable Motors</button>
                    <button class="btn-warning" onclick="disableMotors()">Disable Motors</button>
                    <button class="btn-danger" onclick="emergencyStop()">STOP</button>
                </div>
                
                <h3>Manual Control</h3>
                <div class="controls">
                    <button class="btn-primary" onclick="moveForward()">↑ Forward</button>
                    <button class="btn-primary" onclick="moveBackward()">↓ Backward</button>
                    <button class="btn-primary" onclick="turnLeft()">← Left</button>
                    <button class="btn-primary" onclick="turnRight()">→ Right</button>
                    <button class="btn-warning" onclick="stopMoving()">Stop</button>
                </div>
                
                <div id="motor-status" class="status-item">
                    <strong>Status:</strong> <span id="motor-state">Unknown</span>
                </div>
            </div>
            
            <!-- Exploration Control -->
            <div class="panel">
                <h2>🎯 Autonomous Exploration</h2>
                <div class="controls">
                    <button class="btn-primary" onclick="startExploration()">Start Exploration</button>
                    <button class="btn-danger" onclick="stopExploration()">Stop Exploration</button>
                </div>
                
                <div class="controls">
                    <button class="btn-warning" onclick="saveMap()">Save Map</button>
                    <button class="btn-warning" onclick="loadMap()">Load Map</button>
                </div>
                
                <div id="exploration-status" class="status-item">
                    <div><strong>Mode:</strong> <span id="exp-mode">Idle</span></div>
                    <div><strong>Coverage:</strong> <span id="coverage">0%</span></div>
                    <div><strong>Frontiers:</strong> <span id="frontiers">0</span></div>
                </div>
            </div>
            
            <!-- Robot Status -->
            <div class="panel">
                <h2>📊 Robot Status</h2>
                <div id="pose-status" class="status-item">
                    <div><strong>X:</strong> <span id="pose-x">0.00</span> m</div>
                    <div><strong>Y:</strong> <span id="pose-y">0.00</span> m</div>
                    <div><strong>θ:</strong> <span id="pose-theta">0.00</span> rad</div>
                </div>
                
                <div id="slam-status" class="status-item">
                    <div><strong>SLAM State:</strong> <span id="slam-state">Unknown</span></div>
                    <div><strong>Frames:</strong> <span id="slam-frames">0</span></div>
                    <div><strong>Map Points:</strong> <span id="map-points">0</span></div>
                </div>
            </div>
            
            <!-- System Info -->
            <div class="panel">
                <h2>💻 System Information</h2>
                <div id="system-status" class="status-item">
                    <div><strong>Commands Sent:</strong> <span id="cmd-count">0</span></div>
                    <div><strong>Errors:</strong> <span id="error-count">0</span></div>
                    <div><strong>Uptime:</strong> <span id="uptime">0s</span></div>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        // Update status every second
        setInterval(updateStatus, 1000);
        
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Motor status
                    document.getElementById('motor-state').textContent = 
                        data.motor.enabled ? 'Enabled' : 'Disabled';
                    document.getElementById('cmd-count').textContent = data.motor.commands_sent || 0;
                    document.getElementById('error-count').textContent = data.motor.errors || 0;
                    
                    // SLAM status
                    document.getElementById('slam-state').textContent = 
                        data.slam.tracking_state || 'Unknown';
                    document.getElementById('slam-frames').textContent = 
                        data.slam.frames_processed || 0;
                    document.getElementById('map-points').textContent = 
                        data.slam.map_points || 0;
                    
                    // Mapper status
                    document.getElementById('exp-mode').textContent = 
                        data.mapper.mode || 'idle';
                    document.getElementById('coverage').textContent = 
                        (data.mapper.coverage_percent || 0).toFixed(1) + '%';
                    document.getElementById('frontiers').textContent = 
                        data.mapper.frontiers_visited || 0;
                });
            
            // Update pose
            fetch('/api/pose')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('pose-x').textContent = data.x.toFixed(2);
                    document.getElementById('pose-y').textContent = data.y.toFixed(2);
                    document.getElementById('pose-theta').textContent = data.theta.toFixed(2);
                });
        }
        
        function enableMotors() {
            fetch('/api/motor/enable', {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log('Motors enabled:', data));
        }
        
        function disableMotors() {
            fetch('/api/motor/disable', {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log('Motors disabled:', data));
        }
        
        function emergencyStop() {
            fetch('/api/motor/stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log('Emergency stop:', data));
        }
        
        function sendVelocity(linear, angular) {
            fetch('/api/motor/velocity', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({linear: linear, angular: angular})
            });
        }
        
        function moveForward() { sendVelocity(0.2, 0); }
        function moveBackward() { sendVelocity(-0.2, 0); }
        function turnLeft() { sendVelocity(0, 1.0); }
        function turnRight() { sendVelocity(0, -1.0); }
        function stopMoving() { sendVelocity(0, 0); }
        
        function startExploration() {
            fetch('/api/exploration/start', {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log('Exploration started:', data));
        }
        
        function stopExploration() {
            fetch('/api/exploration/stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => console.log('Exploration stopped:', data));
        }
        
        function saveMap() {
            const filename = prompt('Enter filename:', 'room_map.npz');
            if (filename) {
                fetch('/api/map/save', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({filepath: filename})
                })
                .then(response => response.json())
                .then(data => alert('Map saved: ' + data.filepath));
            }
        }
        
        function loadMap() {
            const filename = prompt('Enter filename:', 'room_map.npz');
            if (filename) {
                fetch('/api/map/load', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({filepath: filename})
                })
                .then(response => response.json())
                .then(data => alert('Map loaded: ' + data.filepath));
            }
        }
        
        // Keyboard controls
        document.addEventListener('keydown', (e) => {
            switch(e.key) {
                case 'ArrowUp': moveForward(); break;
                case 'ArrowDown': moveBackward(); break;
                case 'ArrowLeft': turnLeft(); break;
                case 'ArrowRight': turnRight(); break;
                case ' ': stopMoving(); break;
            }
        });
    </script>
</body>
</html>"""
    
    return html_content


def save_html_template():
    """Save HTML template to templates directory"""
    import os
    os.makedirs('templates', exist_ok=True)
    with open('templates/index.html', 'w') as f:
        f.write(create_html_template())
    logger.info("HTML template saved to templates/index.html")


def run_web_server(host='0.0.0.0', port=5000, debug=False):
    """Run Flask web server"""
    save_html_template()
    app.run(host=host, port=port, debug=debug, threaded=True)