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
from collections import deque
import time


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
    <title>WALL-E</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-main: #050608;
            --bg-panel: #0b0d10;
            --bg-panel-alt: #111318;
            --border-soft: #1f2933;
            --accent: #ffffff;
            --accent-soft: rgba(249, 115, 22, 0.2);
            --text-main: #f9fafb;
            --text-muted: #9ca3af;
            --danger: #ef4444;
            --success: #22c55e;
        }

        * {
            box-sizing: border-box;
        }

        body {
            font-family: 'Space Grotesk', system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            margin: 0;
            padding: 24px;
            background: radial-gradient(circle at top, #111827 0, #02040a 45%, #000000 100%);
            color: var(--text-main);
        }

        .container {
            max-width: 1320px;
            margin: 0 auto;
            background: linear-gradient(145deg, rgba(6, 9, 18, 0.9), rgba(3, 7, 18, 0.98));
            padding: 20px 24px 28px 24px;
            border-radius: 18px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            box-shadow:
                0 24px 80px rgba(0, 0, 0, 0.9),
                0 0 0 1px rgba(12, 17, 28, 0.9);
            backdrop-filter: blur(12px);
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: flex-end;
            margin-bottom: 24px;
            padding-bottom: 16px;
            border-bottom: 1px solid rgba(55, 65, 81, 0.7);
        }

        .header-left h1 {
            font-size: 24px;
            letter-spacing: 0.18em;
            text-transform: uppercase;
            margin: 0 0 4px 0;
            color: var(--text-main);
        }

        .header-left .subline {
            font-size: 12px;
            letter-spacing: 0.24em;
            text-transform: uppercase;
            color: var(--text-muted);
        }

        .header-right {
            text-align: right;
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 0.18em;
            color: var(--text-muted);
        }

        .status-pill {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 4px 10px;
            border-radius: 999px;
            background: rgba(15, 23, 42, 0.9);
            border: 1px solid rgba(75, 85, 99, 0.8);
            font-size: 11px;
        }

        .status-indicator {
            width: 7px;
            height: 7px;
            border-radius: 999px;
            background: var(--success);
            box-shadow: 0 0 10px rgba(34, 197, 94, 0.9);
        }

        .dashboard {
            display: grid;
            grid-template-columns: 2.2fr 1.2fr;
            gap: 18px;
        }

        .left-column,
        .right-column {
            display: flex;
            flex-direction: column;
            gap: 16px;
        }

        .panel {
            background: radial-gradient(circle at top left, rgba(15, 23, 42, 0.9), rgba(15, 15, 20, 0.98));
            border-radius: 14px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            padding: 14px 14px 16px 14px;
            position: relative;
            overflow: hidden;
        }

        .panel::before {
            content: "";
            position: absolute;
            inset: 0;
            background: radial-gradient(circle at top left, rgba(249, 115, 22, 0.08), transparent 55%);
            opacity: 0.9;
            pointer-events: none;
        }

        .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }

        .panel-title {
            font-size: 12px;
            letter-spacing: 0.16em;
            text-transform: uppercase;
            color: var(--text-muted);
        }

        .panel-title span {
            color: var(--accent);
        }

        .panel-tag {
            font-size: 10px;
            text-transform: uppercase;
            letter-spacing: 0.16em;
            color: var(--text-muted);
            background: rgba(17, 24, 39, 0.9);
            padding: 3px 8px;
            border-radius: 999px;
            border: 1px solid rgba(55, 65, 81, 0.9);
        }

        .video-layout {
            display: grid;
            grid-template-columns: auto minmax(260px, 1fr);
            gap: 12px;
            align-items: flex-start;
        }

        #videoFeed {
            border-radius: 10px;
            border: 1px solid rgba(75, 85, 99, 0.9);
            background: #000;
            display: block;
        }

        .log-panel {
            display: flex;
            flex-direction: column;
            gap: 6px;
            height: 100%;
        }

        .logs {
            flex: 1;
            min-height: 160px;
            max-height: 360px;
            overflow-y: auto;
            border-radius: 10px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            background: rgba(3, 7, 18, 0.96);
            padding: 8px 10px;
            font-family: ui-monospace, SFMonoRegular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            font-size: 11px;
            color: var(--text-main);
        }

        .logs::-webkit-scrollbar {
            width: 6px;
        }

        .logs::-webkit-scrollbar-track {
            background: rgba(15, 23, 42, 0.9);
        }

        .logs::-webkit-scrollbar-thumb {
            background: rgba(75, 85, 99, 0.9);
            border-radius: 999px;
        }

        .log-entry {
            display: grid;
            grid-template-columns: 90px 1fr;
            column-gap: 8px;
            row-gap: 2px;
            padding: 4px 0;
            border-bottom: 1px solid rgba(30, 41, 59, 0.6);
        }

        .log-entry:last-child {
            border-bottom: none;
        }

        .log-timestamp {
            color: var(--accent);
            text-align: right;
            white-space: nowrap;
        }

        .log-content {
            color: var(--text-muted);
        }

        .stat-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 8px;
        }

        .stat-card {
            background: rgba(3, 7, 18, 0.96);
            border-radius: 10px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            padding: 8px 10px;
            font-size: 11px;
        }

        .stat-label {
            color: var(--text-muted);
            text-transform: uppercase;
            letter-spacing: 0.14em;
            margin-bottom: 4px;
        }

        .stat-value {
            font-size: 14px;
            font-weight: 500;
            color: var(--text-main);
        }

        .stat-value.accent {
            color: var(--accent);
        }

        .stat-value.success {
            color: var(--success);
        }

        .stat-value.danger {
            color: var(--danger);
        }

        .controls-row {
            display: flex;
            justify-content: space-between;
            align-items: center;
            gap: 10px;
            margin-bottom: 10px;
        }

        .controls {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }

        button {
            padding: 7px 14px;
            border-radius: 999px;
            border: 1px solid rgba(75, 85, 99, 0.9);
            background: linear-gradient(135deg, #020617, #020617);
            color: var(--text-main);
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 0.14em;
            cursor: pointer;
            outline: none;
            position: relative;
            overflow: hidden;
        }

        button::before {
            content: "";
            position: absolute;
            inset: 0;
            background: linear-gradient(135deg, rgba(249, 115, 22, 0.25), transparent);
            opacity: 0;
            transition: opacity 120ms ease;
        }

        button:hover::before {
            opacity: 1;
        }

        .btn-start {
            border-color: rgba(34, 197, 94, 0.8);
        }

        .btn-stop {
            border-color: rgba(248, 113, 113, 0.9);
        }

        .btn-clear {
            border-color: rgba(107, 114, 128, 0.9);
        }

        .status {
            padding: 8px 10px;
            border-radius: 10px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            font-size: 11px;
            background: rgba(3, 7, 18, 0.96);
            display: flex;
            flex-direction: column;
            gap: 4px;
        }

        .status-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 11px;
            text-transform: uppercase;
            letter-spacing: 0.16em;
            color: var(--text-muted);
        }

        .status-line {
            font-family: ui-monospace, SFMonoRegular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            font-size: 11px;
            color: var(--text-main);
        }

        .status-detected {
            border-color: rgba(34, 197, 94, 0.8);
            box-shadow: 0 0 24px rgba(34, 197, 94, 0.28);
        }

        .status-idle {
            border-color: rgba(75, 85, 99, 0.9);
        }

        #trajectoryPlot {
            width: 100%;
            height: 220px;
        }

        .trajectory-wrapper {
            border-radius: 10px;
            border: 1px solid rgba(31, 41, 55, 0.9);
            background: rgba(3, 7, 18, 0.96);
            padding: 6px;
        }

        .trajectory-meta {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 4px;
            font-size: 10px;
            text-transform: uppercase;
            letter-spacing: 0.14em;
            color: var(--text-muted);
        }

        .trajectory-meta span {
            color: var(--accent);
        }

        @media (max-width: 1024px) {
            .dashboard {
                grid-template-columns: 1fr;
            }
            .video-layout {
                grid-template-columns: 1fr;
            }
            .logs {
                max-height: 200px;
            }
        }

        @media (max-width: 640px) {
            body {
                padding: 12px;
            }
            .container {
                padding: 14px;
            }
            #videoFeed {
                width: 100%;
                height: auto;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <header class="header">
            <div class="header-left">
                <h1>WALL-E</h1>
                <div class="subline">Democratizing trash collection</div>
            </div>
            <div class="header-right">
                <div class="status-pill">
                    <div class="status-indicator" id="globalStatusIndicator"></div>
                    <span id="globalStatusText">Tracking Online</span>
                </div>
                <div style="margin-top: 6px;">Team 01</div>
            </div>
        </header>

        <main class="dashboard">
            <section class="left-column">
                <div class="panel">
                    <div class="panel-header">
                        <div class="panel-title"><span>Feed</span> / Detection Channel</div>
                        <div class="panel-tag">Camera Link</div>
                    </div>
                    <div class="video-layout">
                        <img id="videoFeed" src="/video_feed" width="640" height="480" alt="Video feed">
                        <div class="log-panel">
                            <div class="panel-header" style="margin-bottom: 4px;">
                                <div class="panel-title"><span>Log</span> / Detection Events</div>
                                <div class="panel-tag" id="logCountTag">0 entries</div>
                            </div>
                            <div class="logs" id="logContainer">
                                <div id="logs"></div>
                            </div>
                        </div>
                    </div>
                </div>

                <div class="panel">
                    <div class="panel-header">
                        <div class="panel-title"><span>Trajectory</span> / Position History</div>
                        <div class="panel-tag">Pixels Space</div>
                    </div>
                    <div class="trajectory-wrapper">
                        <div class="trajectory-meta">
                            <div>Plot: X vs Y</div>
                            <div>Anchor: <span>Top Left</span></div>
                        </div>
                        <div id="trajectoryPlot"></div>
                    </div>
                </div>
            </section>

            <section class="right-column">
                <div class="panel">
                    <div class="panel-header">
                        <div class="panel-title"><span>System</span> / Telemetry</div>
                        <div class="panel-tag" id="cameraModeTag">Arducam Live</div>
                    </div>
                    <div class="stat-grid">
                        <div class="stat-card">
                            <div class="stat-label">Detections</div>
                            <div class="stat-value accent" id="statDetections">0</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">Frame Rate</div>
                            <div class="stat-value" id="statFPS">0.0 fps</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">Trajectory Points</div>
                            <div class="stat-value" id="statTrajectory">0</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">Log Entries</div>
                            <div class="stat-value" id="statLogs">0</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">Tracking Status</div>
                            <div class="stat-value success" id="statTracking">Active</div>
                        </div>
                        <div class="stat-card">
                            <div class="stat-label">Resolution</div>
                            <div class="stat-value" id="statResolution">1280 x 720</div>
                        </div>
                    </div>
                </div>

                <div class="panel">
                    <div class="panel-header">
                        <div class="panel-title"><span>Control</span> / Session</div>
                        <div class="panel-tag">Command</div>
                    </div>
                    <div class="controls-row">
                        <div class="controls">
                            <button class="btn-start" onclick="toggleTracking(true)">Start</button>
                            <button class="btn-stop" onclick="toggleTracking(false)">Stop</button>
                            <button class="btn-clear" onclick="clearLogs()">Clear Log</button>
                        </div>
                    </div>
                    <div id="status" class="status status-idle">
                        <div class="status-header">
                            <span>Status</span>
                            <span id="statusModeLabel">Idle</span>
                        </div>
                        <div class="status-line" id="statusLine1">Waiting for box detection.</div>
                        <div class="status-line" id="statusLine2">No active target.</div>
                    </div>
                </div>
            </section>
        </main>
    </div>

    <script>
        const socket = io();
        let trajectoryData = { x: [], y: [] };
        let lastUpdate = Date.now();
        let logEntries = 0;
        let trajectoryPoints = [];
        const TRAJECTORY_WINDOW_MS = 3000;
        let lastDetectionTime = 0;
        const DETECTION_TIMEOUT_MS = 20;
        let trajMaxX = 1920;  
        let trajMaxY = 1080;

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
            trajectoryPoints = [];
            logEntries = 0;
            updateLogCount();
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
            const statusModeLabel = document.getElementById('statusModeLabel');
            const statusLine1 = document.getElementById('statusLine1');
            const statusLine2 = document.getElementById('statusLine2');
            const globalIndicator = document.getElementById('globalStatusIndicator');
            const globalText = document.getElementById('globalStatusText');

            const now = Date.now();
            lastDetectionTime = now;  // remember last time we saw a target

            const x = Math.round(data.x);
            const y = Math.round(data.y);
            // const speed = data.speed ? data.speed.toFixed(1) : '0.0';
            // const direction = data.direction ? data.direction.toFixed(1) + ' deg' : 'N/A';

            statusDiv.classList.remove('status-idle');
            statusDiv.classList.add('status-detected');
            statusModeLabel.textContent = 'Target Locked';

            statusLine1.textContent = `[COORD] x=${x}, y=${y}  size=${data.width}x${data.height}`;
            // statusLine2.textContent = `[MOTION] speed=${speed} px/frame  heading=${direction}`;

            globalIndicator.style.background = '#22c55e';
            globalIndicator.style.boxShadow = '0 0 10px rgba(34, 197, 94, 0.9)';
            globalText.textContent = 'Tracking Online';
        }

        function updateIdleStatusIfStale() {
            const now = Date.now();
            if (now - lastDetectionTime > DETECTION_TIMEOUT_MS) {
                const statusDiv = document.getElementById('status');
                const statusModeLabel = document.getElementById('statusModeLabel');
                const statusLine1 = document.getElementById('statusLine1');
                const statusLine2 = document.getElementById('statusLine2');
                const globalIndicator = document.getElementById('globalStatusIndicator');
                const globalText = document.getElementById('globalStatusText');

                statusDiv.classList.remove('status-detected');
                statusDiv.classList.add('status-idle');
                statusModeLabel.textContent = 'Idle';

                statusLine1.textContent = 'Waiting for box detection.';
                statusLine2.textContent = 'No active target.';
            }
        }



        function updateTrajectory(data) {
            const now = Date.now();

            // Add new point with timestamp
            trajectoryPoints.push({
                x: data.x,
                y: data.y,
                t: now
            });

            // Keep only points from the last TRAJECTORY_WINDOW_MS
            trajectoryPoints = trajectoryPoints.filter(p => now - p.t <= TRAJECTORY_WINDOW_MS);

            updatePlot();
        }


    function updatePlot() {
        const plotEl = document.getElementById('trajectoryPlot');
        if (!plotEl) return;

        const now = Date.now();

        // drop old points
        trajectoryPoints = trajectoryPoints.filter(
            p => now - p.t <= TRAJECTORY_WINDOW_MS
        );

        const layoutBase = {
            margin: { l: 40, r: 10, t: 4, b: 30 },
            xaxis: {
                title: { text: 'X (px)', font: { size: 10 } },
                range: [0, trajMaxX],
                gridcolor: '#1f2933',
                zeroline: false
            },
            yaxis: {
                title: { text: 'Y (px)', font: { size: 10 } },
                range: [trajMaxY, 0],
                gridcolor: '#1f2933',
                zeroline: false,
                scaleanchor: 'x'
            },
            showlegend: false,
            paper_bgcolor: 'rgba(0,0,0,0)',
            plot_bgcolor: 'rgba(0,0,0,0)',
            font: { color: '#f9fafb', size: 10 }
        };

        if (trajectoryPoints.length === 0) {
            Plotly.react(
                'trajectoryPlot',
                [],
                layoutBase,
                { displayModeBar: false, staticPlot: true }  // no zoom or drag
            );
            return;
        }

        const xs = trajectoryPoints.map(p => p.x);
        const ys = trajectoryPoints.map(p => p.y);
        const opacities = trajectoryPoints.map(p => {
            const age = now - p.t;
            const alpha = Math.max(0, Math.min(1, 1 - age / TRAJECTORY_WINDOW_MS));
            return alpha;
        });

        const trace = {
            x: xs,
            y: ys,
            mode: 'lines+markers',
            type: 'scatter',
            line: {
                width: 5,
                shape: 'spline'
            },
            marker: {
                size: 9,
                opacity: opacities
            }
        };

        Plotly.react(
            'trajectoryPlot',
            [trace],
            layoutBase,
            { displayModeBar: false, staticPlot: true }
        );
    }




        function updateStats(data) {
            const detEl = document.getElementById('statDetections');
            const fpsEl = document.getElementById('statFPS');
            const trajEl = document.getElementById('statTrajectory');
            const logsEl = document.getElementById('statLogs');
            const trackEl = document.getElementById('statTracking');
            const resEl = document.getElementById('statResolution');
            const modeTag = document.getElementById('cameraModeTag');
            const globalIndicator = document.getElementById('globalStatusIndicator');
            const globalText = document.getElementById('globalStatusText');

            detEl.textContent = data.detection_count;
            fpsEl.textContent = `${data.fps.toFixed(1)} fps`;
            trajEl.textContent = data.trajectory_length;

            // use frontend counter for logs
            logsEl.textContent = logEntries;

            // parse resolution string like "1920x1080"
            if (data.resolution) {
                resEl.textContent = data.resolution.replace('x', ' x ');
                const parts = data.resolution.split('x');
                if (parts.length === 2) {
                    const w = parseInt(parts[0].trim(), 10);
                    const h = parseInt(parts[1].trim(), 10);
                    if (!Number.isNaN(w) && !Number.isNaN(h)) {
                        trajMaxX = w;
                        trajMaxY = h;
                    }
                }
            } else {
                resEl.textContent = '1280 x 720';
                trajMaxX = 1280;
                trajMaxY = 720;
            }

            if (data.tracking_enabled) {
                trackEl.textContent = 'Active';
                trackEl.classList.remove('danger');
                trackEl.classList.add('success');
                globalIndicator.style.background = '#22c55e';
                globalIndicator.style.boxShadow = '0 0 10px rgba(34, 197, 94, 0.9)';
                globalText.textContent = 'Tracking Online';
            } else {
                trackEl.textContent = 'Paused';
                trackEl.classList.remove('success');
                trackEl.classList.add('danger');
                globalIndicator.style.background = '#ef4444';
                globalIndicator.style.boxShadow = '0 0 10px rgba(239, 68, 68, 0.9)';
                globalText.textContent = 'Tracking Paused';
            }

            modeTag.textContent = data.test_mode ? 'Test Mode' : 'Arducam Live';
}

        function addLog(data) {
            const logsDiv = document.getElementById('logs');
            const logContainer = document.getElementById('logContainer');
            const timestamp = new Date().toLocaleTimeString();

            const x = Math.round(data.x);
            const y = Math.round(data.y);
            // const speed = data.speed ? data.speed.toFixed(1) : '0.0';

            const entry = document.createElement('div');
            entry.className = 'log-entry';

            const ts = document.createElement('div');
            ts.className = 'log-timestamp';
            ts.textContent = `[${timestamp}]`;

            const content = document.createElement('div');
            content.className = 'log-content';
            content.textContent = `coord: x=${x}, y=${y}  size=${data.width}x${data.height}`;

            entry.appendChild(ts);
            entry.appendChild(content);
            logsDiv.appendChild(entry);

            logEntries += 1;
            updateLogCount();

            // auto scroll to bottom
            logContainer.scrollTop = logContainer.scrollHeight;
        }

        function updateLogCount() {
            const tag = document.getElementById('logCountTag');
            tag.textContent = `${logEntries} entries`;
            const statLogs = document.getElementById('statLogs');
            if (statLogs && !Number.isNaN(logEntries)) {
                statLogs.textContent = logEntries;
            }
        }

        updateIdleStatusIfStale();
        updatePlot();
        setInterval(updatePlot, 100);
        setInterval(updateIdleStatusIfStale, 100);
    </script>
</body>
</html>

"""
            
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
            AREA_MIN = 20000      # min area for a contour to be considered
            AREA_MAX = 500_000 # max area (probably never hit but kept as guard)

            # Minimum width and height of bounding rectangle
            WIDTH_MIN = 210
            HEIGHT_MIN = 210

            # Acceptable width / height ratio
            ASPECT_MIN = 0.1
            ASPECT_MAX = 6.0

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

    def generate_frames(self):
        """Generate video frames with Arducam optimization"""
        TAIL_SECONDS = 3.0  # how long the trail should live

        while True:
            try:
                # --- acquire frame and detection ---
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

                # FPS calculation
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    elapsed = time.time() - self.start_time
                    self.fps = 30 / elapsed if elapsed > 0 else 0.0
                    self.start_time = time.time()

                processed_frame = frame.copy()
                now = time.time()

                # --- update trajectory only when we have a detection ---
                if self.tracking_enabled and box_position:
                    center_x, center_y, w, h = box_position
                    self.current_position = (center_x, center_y)
                    self.detection_count += 1

                    # store (x, y, t)
                    self.trajectory.append((center_x, center_y, now))

                # drop old points so trail is at most TAIL_SECONDS long
                cutoff = now - TAIL_SECONDS
                while self.trajectory and self.trajectory[0][2] < cutoff:
                    self.trajectory.popleft()

                # --- draw trajectory regardless of whether this frame had a detection ---
                points = list(self.trajectory)
                if len(points) >= 2:
                    for i in range(1, len(points)):
                        x1, y1, t1 = points[i - 1]
                        x2, y2, t2 = points[i]

                        # newer segments brighter, older segments darker
                        age = now - t2
                        frac = max(0.0, min(1.0, 1.0 - age / TAIL_SECONDS))
                        intensity = int(60 + 195 * frac)  # from 60 to 255
                        color = (intensity, 0, 0)        # bright blue in BGR

                        cv2.line(
                            processed_frame,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            color,
                            6,  # thickness
                        )

                # --- draw current detection box only on frames with detection ---
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

                    # emit detection only when there is one
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


                # stats emitted every 10 frames as before
                if self.frame_count % 10 == 0:
                    self.socketio.emit("system_stats", self.get_system_stats())

                # resize for streaming if needed
                if processed_frame.shape[1] > 1280:
                    processed_frame = cv2.resize(processed_frame, (1280, 720))

                # encode and yield frame
                ret, buffer = cv2.imencode(".jpg", processed_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
                )

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