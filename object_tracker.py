import cv2
import numpy as np
import time
import json
import os
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import threading
from collections import deque
from datetime import datetime

class ArducamTracker:
    def __init__(self, camera_index=0, width=1980, height=1080):
        print("Initializing Arducam Tracker...")
        
        # Camera setup
        self.cap = None
        self.test_mode = False
        self.camera_index = camera_index
        self.width = width
        self.height = height
        
        # Initialize camera with Arducam-specific settings
        self.initialize_arducam()
        
        # Tracking variables
        self.trajectory = deque(maxlen=50)
        self.detection_log = []
        self.current_position = None
        self.tracking_enabled = True
        self.detection_count = 0
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        self.test_counter = 0
        
        # Flask setup
        current_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = os.path.join(current_dir, 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.setup_flask_routes()
        
    def initialize_arducam(self):
        """Initialize Arducam with optimized settings for macOS"""
        print(f"Initializing Arducam at index {self.camera_index}...")

        # Default to test mode unless we succeed
        self.test_mode = False
        self.cap = None

        try:
            # Use AVFoundation like in your working test script
            self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_AVFOUNDATION)

            if not self.cap.isOpened():
                print(f"Could not open camera {self.camera_index}")
                self.cap = None
                self.test_mode = True
                return

            print(f"✓ Camera {self.camera_index} opened successfully")

            # Warm up the camera BEFORE changing properties
            ret = False
            frame = None
            for i in range(30):
                ret, frame = self.cap.read()
                print(f"  Warmup attempt {i+1}: {ret}")
                if ret and frame is not None:
                    break
                time.sleep(0.05)

            if not ret or frame is None:
                print("✗ Camera opened but cannot read frames")
                self.cap.release()
                self.cap = None
                self.test_mode = True
                return

            # Now try to set resolution (keep it minimal)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"✓ Frame capture successful! Resolution: {actual_width}x{actual_height}")

            # If you really want, you can try FPS after things work:
            # self.cap.set(cv2.CAP_PROP_FPS, 30)

            # I would drop these on macOS, they are often unsupported:
            # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

        except Exception as e:
            print(f"Camera initialization error: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None
            self.test_mode = True

        
        # If we get here, camera failed - enable test mode
        if self.cap is None:
            print("⚠ Enabling TEST MODE")
            self.test_mode = True
        
    def setup_flask_routes(self):
        @self.app.route('/')
        def index():
            return """<!DOCTYPE html>
<html>
<head>
    <title>Brown Box Tracker</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            margin: 0; 
            padding: 20px; 
            background: #f5f5f5; 
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
            background: white; 
            padding: 20px; 
            border-radius: 10px; 
            box-shadow: 0 2px 10px rgba(0,0,0,0.1); 
        }
        .header { 
            text-align: center; 
            margin-bottom: 30px; 
            border-bottom: 2px solid #eee; 
            padding-bottom: 20px; 
        }
        .dashboard { 
            display: grid; 
            grid-template-columns: 1fr 1fr; 
            gap: 20px; 
            margin-bottom: 20px; 
        }
        .video-container { 
            grid-column: 1 / -1; 
            text-align: center; 
        }
        .stats-panel, .controls-panel { 
            background: #f8f9fa; 
            padding: 15px; 
            border-radius: 8px; 
            border-left: 4px solid #007bff; 
        }
        .controls { 
            display: flex; 
            gap: 10px; 
            margin: 15px 0; 
        }
        button { 
            padding: 10px 20px; 
            border: none; 
            border-radius: 5px; 
            cursor: pointer; 
            font-size: 14px; 
            transition: background 0.3s; 
        }
        .btn-start { background: #28a745; color: white; }
        .btn-stop { background: #dc3545; color: white; }
        .btn-clear { background: #6c757d; color: white; }
        button:hover { opacity: 0.9; }
        .status { 
            padding: 10px; 
            border-radius: 5px; 
            margin: 10px 0; 
            font-weight: bold; 
        }
        .detected { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .not-detected { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        .logs { 
            height: 200px; 
            overflow-y: auto; 
            border: 1px solid #ddd; 
            padding: 10px; 
            background: #fafafa; 
            border-radius: 5px; 
            font-family: monospace; 
            font-size: 12px; 
        }
        .stat-item { 
            margin: 8px 0; 
            display: flex; 
            justify-content: space-between; 
        }
        .stat-value { 
            font-weight: bold; 
            color: #007bff; 
        }
        #trajectoryPlot { 
            background: white; 
            border-radius: 8px; 
            padding: 10px; 
            margin-top: 20px; 
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>📦 Brown Box Tracker</h1>
            <p>Real-time object tracking and trajectory analysis</p>
        </div>

        <div class="dashboard">
            <div class="video-container">
                <img id="videoFeed" src="/video_feed" width="640" height="480" style="border: 2px solid #ddd; border-radius: 8px;">
            </div>

            <div class="stats-panel">
                <h3>📊 System Statistics</h3>
                <div class="stat-item">Detection Count: <span id="statDetections" class="stat-value">0</span></div>
                <div class="stat-item">FPS: <span id="statFPS" class="stat-value">0</span></div>
                <div class="stat-item">Trajectory Points: <span id="statTrajectory" class="stat-value">0</span></div>
                <div class="stat-item">Log Entries: <span id="statLogs" class="stat-value">0</span></div>
                <div class="stat-item">Tracking Status: <span id="statTracking" class="stat-value">Active</span></div>
                <div class="stat-item">Camera: 
                    <span id="statCamera" class="stat-value">Arducam</span>
                </div>
                <div class="stat-item">Resolution: 
                    <span id="statResolution" class="stat-value">1280x720</span>
                </div>
            </div>

            <div class="controls-panel">
                <h3>⚙️ Controls</h3>
                <div class="controls">
                    <button class="btn-start" onclick="toggleTracking(true)">▶ Start Tracking</button>
                    <button class="btn-stop" onclick="toggleTracking(false)">⏹ Stop Tracking</button>
                    <button class="btn-clear" onclick="clearLogs()">🗑 Clear Logs</button>
                </div>
                <div id="status" class="status not-detected">
                    🔍 Status: Waiting for box detection...
                </div>
            </div>
        </div>

        <div id="trajectoryPlot" style="width:100%; height:400px;"></div>

        <div>
            <h3>📋 Detection Log</h3>
            <div class="logs" id="logContainer">
                <div id="logs"></div>
            </div>
        </div>
    </div>

    <script>
        const socket = io();
        let trajectoryData = { x: [], y: [], time: [] };
        let lastUpdate = Date.now();
        
        socket.on('detection_update', function(data) {
            updateStatus(data);
            updateTrajectory(data);
            addLog(data);
        });
        
        socket.on('system_stats', function(data) {
            updateStats(data);
        });
        
        socket.on('logs_cleared', function() {
            document.getElementById('logs').innerHTML = '';
            trajectoryData = { x: [], y: [], time: [] };
            updatePlot();
        });
        
        function toggleTracking(enabled) {
            socket.emit('toggle_tracking', { enabled: enabled });
        }
        
        function clearLogs() {
            socket.emit('clear_logs');
        }
        
        function updateStatus(data) {
            const statusDiv = document.getElementById('status');
            const now = Date.now();
            const elapsed = now - lastUpdate;
            lastUpdate = now;
            
            statusDiv.innerHTML = `✅ Box detected at (${Math.round(data.x)}, ${Math.round(data.y)})<br>
                                  📏 Size: ${data.width}×${data.height} | 🚀 Speed: ${data.speed ? data.speed.toFixed(1) : '0'} px/frame<br>
                                  🧭 Direction: ${data.direction ? data.direction.toFixed(1) + '°' : 'N/A'}`;
            statusDiv.className = 'status detected';
        }
        
        function updateTrajectory(data) {
            const now = Date.now();
            trajectoryData.x.push(data.x);
            trajectoryData.y.push(data.y);
            trajectoryData.time.push(now);
            
            // Keep only last 50 points
            if (trajectoryData.x.length > 50) {
                trajectoryData.x.shift();
                trajectoryData.y.shift();
                trajectoryData.time.shift();
            }
            
            updatePlot();
        }


        // Update stats with resolution
        socket.on('system_stats', function(data) {
            updateStats(data);
            document.getElementById('statCamera').textContent = data.test_mode ? 'Test Mode' : 'Arducam Live';
            document.getElementById('statCamera').style.color = data.test_mode ? '#ffc107' : '#28a745';
            document.getElementById('statResolution').textContent = data.resolution || '1280x720';
        });
        
        function updatePlot() {
            const plotData = [{
                x: trajectoryData.x,
                y: trajectoryData.y,
                mode: 'lines+markers',
                type: 'scatter',
                name: 'Trajectory',
                line: { color: 'blue', width: 2 },
                marker: { color: 'red', size: 6 }
            }];
            
            const layout = {
                title: 'Box Trajectory Over Time',
                xaxis: { title: 'X Position (pixels)', range: [0, 640] },
                yaxis: { title: 'Y Position (pixels)', range: [480, 0], scaleanchor: "x" },
                showlegend: false
            };
            
            Plotly.react('trajectoryPlot', plotData, layout);
        }
        
        function updateStats(data) {
            document.getElementById('statDetections').textContent = data.detection_count;
            document.getElementById('statFPS').textContent = data.fps.toFixed(1);
            document.getElementById('statTrajectory').textContent = data.trajectory_length;
            document.getElementById('statLogs').textContent = data.log_entries;
            document.getElementById('statTracking').textContent = data.tracking_enabled ? 'Active' : 'Paused';
            document.getElementById('statTracking').style.color = data.tracking_enabled ? '#28a745' : '#dc3545';
        }
        
        function addLog(data) {
            const logsDiv = document.getElementById('logs');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.innerHTML = `<span style="color: #666;">[${timestamp}]</span> 
                                📍 X: ${Math.round(data.x)}, Y: ${Math.round(data.y)} 
                                📏 ${data.width}×${data.height} 
                                🚀 ${data.speed ? data.speed.toFixed(1) : '0'} px/frame`;
            logsDiv.appendChild(logEntry);
            logsDiv.scrollTop = logsDiv.scrollHeight;
        }
        
        // Initialize plot
        updatePlot();
    </script>
</body>
</html>"""
            
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/test')
        def test():
            return "Flask is working! If you see this, the server is running."
        
        @self.app.route('/camera_info')
        def camera_info():
            if self.cap and not self.test_mode:
                width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = self.cap.get(cv2.CAP_PROP_FPS)
                return {
                    'status': 'connected',
                    'resolution': f"{width}x{height}",
                    'fps': fps,
                    'test_mode': False
                }
            else:
                return {
                    'status': 'test_mode',
                    'resolution': '640x480',
                    'fps': 30,
                    'test_mode': True
                }
        
        @self.socketio.on('connect')
        def handle_connect():
            print('Client connected to WebSocket')
            self.socketio.emit('trajectory_update', list(self.trajectory))
            self.socketio.emit('system_stats', self.get_system_stats())
            
        @self.socketio.on('toggle_tracking')
        def toggle_tracking(data):
            self.tracking_enabled = data['enabled']
            self.socketio.emit('system_stats', self.get_system_stats())
            

    def detect_brown_box(self, frame):
        """Enhanced brown box detection for Arducam"""
        try:
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # BROWN color detection in HSV
            # Brown typically has low saturation and medium value
            lower_brown = np.array([0, 50, 20])
            upper_brown = np.array([20, 200, 200])
            
            # Alternative brown ranges
            lower_brown2 = np.array([160, 50, 20])
            upper_brown2 = np.array([180, 200, 200])
            
            # Create masks
            mask1 = cv2.inRange(hsv, lower_brown, upper_brown)
            mask2 = cv2.inRange(hsv, lower_brown2, upper_brown2)
            brown_mask = mask1 | mask2
            
            # Noise reduction
            kernel = np.ones((5,5), np.uint8)
            brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_CLOSE, kernel)
            brown_mask = cv2.morphologyEx(brown_mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Filter contours by area and shape
                valid_contours = []
                for contour in contours:
                    area = cv2.contourArea(contour)
                    # Adjust these values based on your box size
                    if 20000 < area < 1000000:  # Larger range for high-res camera
                        x, y, w, h = cv2.boundingRect(contour)
                        aspect_ratio = w / h
                        # Look for box-like shapes (aspect ratio near 1)
                        if 0.5 < aspect_ratio < 2.0:
                            valid_contours.append(contour)
                
                if valid_contours:
                    # Use largest valid contour
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    return (center_x, center_y, w, h), frame, brown_mask
                    
        except Exception as e:
            print(f"Detection error: {e}")
            
        return None, frame, None

    def create_test_frame(self):
        """Create test frame that simulates Arducam high resolution"""
        # Create background
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 100
        
        # Create moving brown box
        self.test_counter += 1
        center_x = self.width // 2 + int(300 * np.sin(self.test_counter * 0.02))
        center_y = self.height // 2 + int(200 * np.cos(self.test_counter * 0.03))
        w, h = 80, 60
        
        # Draw brown box
        brown_color = (42, 42, 165)  # Brown in BGR
        cv2.rectangle(frame, 
                     (center_x - w//2, center_y - h//2),
                     (center_x + w//2, center_y + h//2),
                     brown_color, -1)
        
        # Add info text
        cv2.putText(frame, "ARDUCAM TEST MODE - No Camera Detected", 
                   (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame, f"Resolution: {self.width}x{self.height}", 
                   (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "Check USB connection and run arducam_setup.py", 
                   (50, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame, (center_x, center_y, w, h)

    def generate_frames(self):
        """Generate video frames with Arducam optimization"""
        while True:
            try:
                if self.test_mode:
                    frame, box_position = self.create_test_frame()
                    mask = None
                else:
                    ret, frame = self.cap.read()
                    if not ret:
                        print("Failed to read from Arducam - switching to test mode")
                        self.test_mode = True
                        continue
                    box_position, frame, mask = self.detect_brown_box(frame)
                
                # Calculate FPS
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed = time.time() - self.start_time
                    self.fps = 30 / elapsed
                    self.start_time = time.time()
                
                processed_frame = frame.copy()
                
                if self.tracking_enabled and box_position:
                    center_x, center_y, w, h = box_position
                    self.current_position = (center_x, center_y)
                    self.trajectory.append((center_x, center_y))
                    self.detection_count += 1
                    
                    # Draw detection
                    cv2.rectangle(processed_frame, 
                                (center_x - w//2, center_y - h//2),
                                (center_x + w//2, center_y + h//2),
                                (0, 255, 0), 3)
                    cv2.circle(processed_frame, (center_x, center_y), 8, (0, 0, 255), -1)
                    
                    # Draw trajectory
                    for i in range(1, len(self.trajectory)):
                        cv2.line(processed_frame, 
                               self.trajectory[i-1], 
                               self.trajectory[i], 
                               (255, 0, 0), 3)
                    
                    # Add detection info
                    mode_text = "TEST MODE" if self.test_mode else "ARDUCAM LIVE"
                    cv2.putText(processed_frame, f'Mode: {mode_text}', 
                              (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f'Position: ({center_x}, {center_y})', 
                              (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(processed_frame, f'Size: {w}x{h}', 
                              (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # Send to dashboard
                    self.socketio.emit('detection_update', {
                        'x': center_x,
                        'y': center_y,
                        'width': w,
                        'height': h,
                        'speed': 0,
                        'direction': 0,
                        'timestamp': time.time()
                    })
                
                # Add FPS and status
                status_color = (0, 255, 0) if not self.test_mode else (0, 255, 255)
                cv2.putText(processed_frame, f'FPS: {self.fps:.1f}', 
                          (20, processed_frame.shape[0] - 40), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, f'Status: {"LIVE" if not self.test_mode else "TEST MODE"}', 
                          (20, processed_frame.shape[0] - 10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                
                # Send stats
                if self.frame_count % 10 == 0:
                    self.socketio.emit('system_stats', self.get_system_stats())
                
                # Resize for streaming if too large
                if processed_frame.shape[1] > 1280:
                    processed_frame = cv2.resize(processed_frame, (1280, 720))
                
                # Encode frame
                ret, buffer = cv2.imencode('.jpg', processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                       
            except Exception as e:
                print(f"Frame generation error: {e}")
                time.sleep(0.1)

    def get_system_stats(self):
        return {
            'detection_count': self.detection_count,
            'fps': self.fps,
            'tracking_enabled': self.tracking_enabled,
            'trajectory_length': len(self.trajectory),
            'current_position': self.current_position,
            'log_entries': len(self.detection_log),
            'test_mode': self.test_mode,
            'resolution': f"{self.width}x{self.height}"
        }

    def run(self):
        """Start the Arducam tracking system"""
        try:
            print("🚀 Starting Arducam Tracking System...")
            print(f"📷 Camera status: {'TEST MODE' if self.test_mode else 'LIVE'}")
            print(f"📊 Resolution: {self.width}x{self.height}")
            print("🌐 Web dashboard: http://localhost:5000")
            print("   - /camera_info for camera status")
            print("   - /video_feed for live stream")
            
            self.socketio.run(
                self.app, 
                host='0.0.0.0', 
                port=5000, 
                debug=False, 
                allow_unsafe_werkzeug=True
            )
                
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            if self.cap and not self.test_mode:
                self.cap.release()

if __name__ == "__main__":
    # Try different resolutions for Arducam
    tracker = ArducamTracker(camera_index=0, width=1920, height=1080)
    tracker.run()