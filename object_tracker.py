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
import serial
import config
from motion_planner import MotionPlanner
from mecanum_controller import MecanumController, MotorCommand
from odometry import MecanumOdometry
import sys
import glob

class ArducamTracker:
    def __init__(self, camera_index=0, width=1980, height=1080):
        print("Initializing Arducam Tracker...")
        
        # Camera setup
        self.cap = None
        self.test_mode = False
        self.camera_index = camera_index
        self.width = width
        self.height = height

        self.motion_planner = MotionPlanner(width=self.width, height=self.height)
        self.mecanum = MecanumController()
        self.last_control_time = None
        self.motor_serial = self._init_motor_serial()
        self.last_motion = None
        self.last_regression = None
        self.odom = MecanumOdometry()
        self.last_control_time = None

        
        # Initialize camera with Arducam-specific settings
        self.initialize_arducam()
        
        # Tracking variables
        self.trajectory = deque(maxlen=500)
        self.detection_log = []
        self.current_position = None
        self.tracking_enabled = True
        self.detection_count = 0
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        self.test_counter = 0

        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=200,
            varThreshold=16,
            detectShadows=False
        )
        
        # Flask setup
        current_dir = os.path.dirname(os.path.abspath(__file__))
        template_dir = os.path.join(current_dir, 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.setup_flask_routes()

        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.last_manual_time = 0
        self.manual_timeout = 0.5 # seconds before stopping if no key sent


        
    def initialize_arducam(self):
        """Initialize Arducam on Raspberry Pi"""
        print(f"Initializing Arducam at index {self.camera_index}...")

        self.test_mode = False
        self.cap = None

        try:
            # On Pi, use the default backend (V4L2)
            self.cap = cv2.VideoCapture(self.camera_index)

            if not self.cap.isOpened():
                print(f"Could not open camera {self.camera_index}")
                self.cap = None
                self.test_mode = True
                return

            # Try to grab one test frame
            print("Testing first frame from Arducam...")
            for _ in range(10):
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    break
                time.sleep(0.05)

            if not ret or frame is None:
                print("Camera opened but cannot read frames")
                self.cap.release()
                self.cap = None
                self.test_mode = True
                return

            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"Frame capture successful! Resolution: {actual_width}x{actual_height}")

        except Exception as e:
            print(f"Camera initialization error: {e}")
            if self.cap:
                self.cap.release()
                self.cap = None
            self.test_mode = True

        
    def setup_flask_routes(self):
        @self.app.route('/')
        def index():
            return render_template("index.html")
            
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
        
        @self.socketio.on('manual_drive')
        def handle_manual_drive(data):
            self.manual_vx = data.get('vx', 0.0)
            self.manual_vy = data.get('vy', 0.0)
            self.last_manual_time = time.time()

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
        """Box detection with motion gating and clearly tunable parameters."""
        try:
            # ==========================
            # TUNABLE PARAMETERS - COLOR
            # ==========================
            # OpenCV HSV ranges:
            #   H: 0..179, S: 0..255, V: 0..255
            COLOR_H_MIN = 0      # min hue (0 = red, 30 = yellow, 60 = green)
            COLOR_H_MAX = 255     # max hue (up to olive / muted green)
            COLOR_S_MIN = 0     # min saturation (increase to reject gray/skin)
            COLOR_S_MAX = 255    # max saturation (decrease to reject vivid colors)
            COLOR_V_MIN = 0     # min value (increase to ignore very dark areas)
            COLOR_V_MAX = 255    # max value (decrease to ignore very bright glare)

            # ============================
            # TUNABLE PARAMETERS - MOTION
            # ============================
            # Background subtractor learning rate:
            #   higher -> adapts faster, background updates quickly
            #   lower  -> adapts slower, motion persists longer
            MOTION_LEARNING_RATE = 0.9

            # Threshold on bg subtractor output:
            #   lower -> more pixels considered moving (noisier)
            #   higher -> fewer pixels considered moving
            MOTION_BINARY_THRESH = 15

            # ==========================
            # TUNABLE PARAMETERS - SHAPE
            # ==========================
            # Area in pixels. Set based on how big the box appears.
            AREA_MIN = 15000      # min area for a contour to be considered
            AREA_MAX = 500_000 # max area (probably never hit but kept as guard)

            # Minimum width and height of bounding rectangle
            WIDTH_MIN = 150
            HEIGHT_MIN = 150

            # Acceptable width / height ratio
            ASPECT_MIN = 0.3
            ASPECT_MAX = 3.0

            # Solidity filter (area / convex_hull_area)
            #   closer to 1 -> filled, compact shapes
            SOLIDITY_MIN = 0.7

            # Kernel size for morphology (noise cleaning)
            MORPH_KERNEL_SIZE = 3

            # =====================
            #     PIPELINE
            # =====================

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Color mask for muted cardboard like stuff
            lower_cardboard = np.array(
                [COLOR_H_MIN, COLOR_S_MIN, COLOR_V_MIN],
                dtype=np.uint8
            )
            upper_cardboard = np.array(
                [COLOR_H_MAX, COLOR_S_MAX, COLOR_V_MAX],
                dtype=np.uint8
            )
            color_mask = cv2.inRange(hsv, lower_cardboard, upper_cardboard)

            # Motion mask from background subtractor
            motion_raw = self.bg_subtractor.apply(
                frame, learningRate=MOTION_LEARNING_RATE
            )
            _, motion_mask = cv2.threshold(
                motion_raw, MOTION_BINARY_THRESH, 255, cv2.THRESH_BINARY
            )

            # Only moving cardboard colored pixels
            box_mask = cv2.bitwise_and(color_mask, motion_mask)

            # Morphology to clean noise
            kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
            box_mask = cv2.morphologyEx(box_mask, cv2.MORPH_CLOSE, kernel)
            box_mask = cv2.morphologyEx(box_mask, cv2.MORPH_OPEN, kernel)

            # Find contours
            contours, _ = cv2.findContours(
                box_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours:
                return None, frame, box_mask

            valid_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if not (AREA_MIN <= area <= AREA_MAX):
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                if w < WIDTH_MIN or h < HEIGHT_MIN:
                    continue

                aspect_ratio = w / float(h)
                if not (ASPECT_MIN <= aspect_ratio <= ASPECT_MAX):
                    continue

                # Solidity filter
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area == 0:
                    continue
                solidity = float(area) / hull_area
                if solidity < SOLIDITY_MIN:
                    continue

                valid_contours.append(contour)

            if not valid_contours:
                return None, frame, box_mask

            largest_contour = max(valid_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2

            return (center_x, center_y, w, h), frame, box_mask

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
    
    def _init_motor_serial(self):
        """
        Initialize serial connection, handling macOS and Linux naming differences.
        """
        # 1. Define patterns based on OS
        if sys.platform.startswith('darwin'):
            # macOS patterns
            patterns = ['/dev/tty.usbmodem*', '/dev/tty.usbserial*']
        else:
            # Linux/Pi patterns
            patterns = ['/dev/ttyACM*', '/dev/ttyUSB*']

        # 2. Find matching ports
        possible_ports = []
        for pattern in patterns:
            possible_ports.extend(glob.glob(pattern))

        if not possible_ports:
            print(f"⚠ No serial devices found matching patterns: {patterns}")
            print("  - Check USB cable (ensure it has data wires)")
            print("  - On Mac, run 'ls /dev/tty.*' in terminal to verify")
            return None

        # 3. Connect to the first found port
        port = possible_ports[0]
        baud = getattr(config, 'SERIAL_BAUDRATE', 115200)

        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            print(f"✓ Motor link established: {port} @ {baud}")
            return ser
        except Exception as e:
            print(f"⚠ Failed to open {port}: {e}")
            return None

    def _send_motor_command(self, cmd: MotorCommand):
        """
        Send motor powers as comma-separated percentages over serial.
        Format: "FL,FR,RL,RR\n" (e.g., "100,100,50,0")
        """
        if self.motor_serial is None:
            return

        try:
            # 1. Scale normalized float (-1.0 to 1.0) to percentage (-100 to 100)
            # 2. Clamp between -100 and 100 to ensure safety
            # 3. Convert to integer
            def to_pct(val):
                return int(max(-100, min(100, val * 100)))

            fl_pct = to_pct(cmd.fl)
            fr_pct = to_pct(cmd.fr)
            rl_pct = to_pct(cmd.rl)
            rr_pct = to_pct(cmd.rr)

            # Format: "100,100,50,0"
            line = f"{fl_pct},{fr_pct},{rl_pct},{rr_pct}\n"
            
            # Write bytes to serial
            self.motor_serial.write(line.encode("ascii"))
            
        except Exception as e:
            print(f"Motor serial error: {e}")
            # Optional: Attempt to reconnect if error persists


    def generate_frames(self):
        """Generate video frames with Arducam optimization and Manual/Auto control mixing."""
        TAIL_SECONDS = 3.0  # how long the trail should live

        while True:
            try:
                # --- 1. Acquire frame and detection ---
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

                # --- 2. FPS calculation ---
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed = time.time() - self.start_time
                    self.fps = 30 / elapsed if elapsed > 0 else 0.0
                    self.start_time = time.time()

                processed_frame = frame.copy()
                now = time.time()

                # --- 3. Update trajectory (only if tracking enabled + box detected) ---
                if self.tracking_enabled and box_position:
                    center_x, center_y, w, h = box_position
                    self.current_position = (center_x, center_y)
                    self.detection_count += 1
                    # store (x, y, t)
                    self.trajectory.append((center_x, center_y, now))

                # Drop old points so trail is at most TAIL_SECONDS long
                cutoff = now - TAIL_SECONDS
                while self.trajectory and self.trajectory[0][2] < cutoff:
                    self.trajectory.popleft()

                # --- 4. CONTROL LOGIC (Manual vs Autonomous) ---
                motor_cmd = None
                
                # Check if manual input was received recently (within 0.5s)
                # Defaults to 0 if variables aren't set yet to prevent crashes
                last_manual = getattr(self, 'last_manual_time', 0)
                manual_active = (now - last_manual) < 0.5

                if manual_active:
                    # >>> MANUAL MODE <<<
                    vx = getattr(self, 'manual_vx', 0.0)
                    vy = getattr(self, 'manual_vy', 0.0)
                    # Use the new compute_manual method you added to MecanumController
                    motor_cmd = self.mecanum.compute_manual(vx, vy)
                
                elif self.tracking_enabled and len(self.trajectory) >= 2:
                    # >>> AUTONOMOUS MODE <<<
                    # 1. Plan Motion
                    motion, reg = self.motion_planner.compute(list(self.trajectory))
                    self.last_motion = motion
                    self.last_regression = reg

                    # 2. Update Web Interface with Plan
                    self.socketio.emit("motion_update", motion.to_dict())
                    self.socketio.emit("trajectory_fit", reg.to_dict())

                    # 3. Calculate DT for PID
                    if self.last_control_time is None:
                        dt = 0.0
                    else:
                        dt = max(1e-3, now - self.last_control_time)
                    
                    # 4. Compute Motor Command via PID
                    motor_cmd = self.mecanum.compute(motion, dt)

                # --- 5. Execute Motor Command ---
                if motor_cmd is not None:
                    self.last_control_time = now
                    
                    # Send to Web UI
                    self.socketio.emit("motor_update", motor_cmd.to_dict())
                    
                    # Send to STM32 via Serial
                    self._send_motor_command(motor_cmd)

                    # Update Odometry
                    dt_odom = 0.1 # approximate if dt not available
                    pose = self.odom.step(motor_cmd, dt_odom)
                    self.socketio.emit("pose_update", pose.to_dict())

                # --- 6. Visualization: Draw Trajectory ---
                points = list(self.trajectory)
                if len(points) >= 2:
                    for i in range(1, len(points)):
                        x1, y1, t1 = points[i - 1]
                        x2, y2, t2 = points[i]

                        # Newer segments brighter
                        age = now - t2
                        frac = max(0.0, min(1.0, 1.0 - age / TAIL_SECONDS))
                        intensity = int(60 + 195 * frac)
                        color = (intensity, 0, 0) # BGR: Blue

                        cv2.line(
                            processed_frame,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            color,
                            6
                        )

                # --- 7. Visualization: Draw Box ---
                if self.tracking_enabled and box_position:
                    center_x, center_y, w, h = box_position
                    cv2.rectangle(
                        processed_frame,
                        (center_x - w // 2, center_y - h // 2),
                        (center_x + w // 2, center_y + h // 2),
                        (0, 255, 0),
                        3,
                    )
                    cv2.circle(processed_frame, (center_x, center_y), 8, (0, 0, 255), -1)

                    self.socketio.emit(
                        "detection_update",
                        {
                            "x": center_x,
                            "y": center_y,
                            "width": w,
                            "height": h,
                            "speed": 0,
                            "direction": 0,
                            "timestamp": now,
                        },
                    )

                # --- 8. Housekeeping ---
                # Emit stats every 10 frames
                if self.frame_count % 10 == 0:
                    self.socketio.emit("system_stats", self.get_system_stats())

                # Resize for streaming if needed
                if processed_frame.shape[1] > 1280:
                    processed_frame = cv2.resize(processed_frame, (1280, 720))

                # Encode and yield
                ret, buffer = cv2.imencode(".jpg", processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                )

            except Exception as e:
                print(f"Frame generation error: {e}")
                time.sleep(0.1)

    def _apply_motion_to_motors(self, motion):
        """
        Map image-plane desired velocity to robot commands.

        For now this is a stub. Here you convert motion.vx, motion.vy
        into wheel/steering commands and send them over serial / CAN
        to the STM32.
        """
        if not motion.has_data:
            return

        # example placeholder:
        # forward proportional to -motion.vy, turn proportional to -motion.vx
        forward = -motion.vy
        turn = -motion.vx

        # TODO: scale and saturate, then send to STM32
        # e.g. self.motor_serial.write(f"F{forward:.2f} T{turn:.2f}\n".encode())
        pass



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