const socket = io();
        let trajectoryData = { x: [], y: [] };
        let lastUpdate = Date.now();
        let logEntries = 0;
        let trajectoryPoints = [];
        const TRAJECTORY_WINDOW_MS = 500;
        let lastDetectionTime = 0;
        const DETECTION_TIMEOUT_MS = 50;
        let trajMaxX = 1920;  
        let trajMaxY = 1080;
        let desiredMotion = {
            vx: 0,
            vy: 0,
            ax: 0,
            ay: 0,
            hasData: false
        };

        const MIN_TRAJ_POINTS = 4;


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

        socket.on('motion_update', function (data) {
            updateMotionWidgetBackend(data);
        });

        socket.on('trajectory_fit', function (data) {
            updateRegressionPlotBackend(data);
        });

        socket.on('motor_update', function (data) {
            updateMotorWidget(data);
        });

        socket.on('pose_update', function (data) {
            updatePoseWidget(data);
        });

        const keys = {
                ArrowUp: false,
                ArrowDown: false,
                ArrowLeft: false,
                ArrowRight: false
            };

            function sendManualControl() {
                let vx = 0;
                let vy = 0;

                // Calculate vector based on held keys
                if (keys.ArrowUp) vy -= 1.0;   // Up in image is negative Y
                if (keys.ArrowDown) vy += 1.0;
                if (keys.ArrowLeft) vx -= 1.0;
                if (keys.ArrowRight) vx += 1.0;

                // Emit to server
                socket.emit('manual_drive', { vx: vx, vy: vy });
            }

            window.addEventListener('keydown', (e) => {
        if (keys.hasOwnProperty(e.code)) {
            e.preventDefault();
            keys[e.code] = true;
            sendManualControl();
        }
    });

    window.addEventListener('keyup', (e) => {
        if (keys.hasOwnProperty(e.code)) {
            e.preventDefault(); // <--- ADD THIS LINE
            keys[e.code] = false;
            sendManualControl();
        }
    });


        function toggleTracking(enabled) {
            socket.emit('toggle_tracking', { enabled: enabled });
        }

                function clearLogs() {
            const logsDiv = document.getElementById('logs');
            const logContainer = document.getElementById('logContainer');

            if (logsDiv) {
                logsDiv.innerHTML = '';
            }

            logEntries = 0;
            updateLogCount();

            if (logContainer) {
                logContainer.scrollTop = 0;
            }
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
            statusLine2.textContent = `Tracking target!`;

            globalIndicator.style.background = '#22c55e';
            globalIndicator.style.boxShadow = '0 0 10px rgba(34, 197, 94, 0.9)';
            globalText.textContent = 'Vision Online';
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

        function updateMotionWidgetBackend(motion) {
            const textV = document.getElementById('motionVelocityText');
            const textA = document.getElementById('motionAccelText');

            if (!motion || !motion.has_data) {
                if (textV) textV.textContent = 'v_des: vx=0.0, vy=0.0 px/s';
                if (textA) textA.textContent = 'a_des: ax=0.0, ay=0.0 px/s²';
                // optional: clear arrows from Plotly plot
                return;
            }

            const vx = motion.vx || 0;
            const vy = motion.vy || 0;
            const ax = motion.ax || 0;
            const ay = motion.ay || 0;

            if (textV) {
                textV.textContent = `v_des: vx=${vx.toFixed(1)}, vy=${vy.toFixed(1)} px/s`;
            }
            if (textA) {
                textA.textContent = `a_des: ax=${ax.toFixed(1)}, ay=${ay.toFixed(1)} px/s²`;
            }

            // if you built a Plotly vector plot, update it here using vx, vy, ax, ay
            // for example: draw quiver arrows centered at frame center
        }

        function updateRegressionPlotBackend(data) {
            const plotId = 'regressionPlot';
            const el = document.getElementById(plotId);
            if (!el) return;

            const layout = {
                margin: { l: 40, r: 10, t: 10, b: 30 },
                xaxis: {
                    title: { text: 'X (px)', font: { size: 9 } },
                    range: [0, trajMaxX],
                    gridcolor: '#1f2933',
                    zeroline: false
                },
                yaxis: {
                    title: { text: 'Y (px)', font: { size: 9 } },
                    range: [trajMaxY, 0],
                    gridcolor: '#1f2933',
                    zeroline: false,
                    scaleanchor: 'x'
                },
                showlegend: true,
                legend: { orientation: 'h', y: -0.2, x: 0.0 },
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: '#f9fafb', size: 9 }
            };

            if (!data || !data.has_data || trajectoryPoints.length < MIN_TRAJ_POINTS) {
                Plotly.react(plotId, [], layout, { displayModeBar: false, staticPlot: true });
                return;
            }

            const xs = trajectoryPoints.map(p => p.x);
            const ys = trajectoryPoints.map(p => p.y);

            const scatterTrace = {
                x: xs,
                y: ys,
                mode: 'markers',
                type: 'scatter',
                name: 'trajectory',
                marker: { size: 6 }
            };

            const lineTrace = {
                x: data.line_x || [],
                y: data.line_y || [],
                mode: 'lines',
                type: 'scatter',
                name: 'line of best fit',
                line: { width: 2, dash: 'dot' }
            };

            Plotly.react(
                plotId,
                [scatterTrace, lineTrace],
                layout,
                { displayModeBar: false, staticPlot: true }
            );
        }

        function updatePoseWidget(data) {
            const plotId = 'posePlot';
            const el = document.getElementById(plotId);
            if (!el || !data) {
                return;
            }

            const x = data.x || 0.0;
            const y = data.y || 0.0;
            const theta = data.theta || 0.0;

            // robot body as a short arrow
            const bodyLen = 0.3; // meters, just for drawing
            const x2 = x + bodyLen * Math.cos(theta);
            const y2 = y + bodyLen * Math.sin(theta);

            const bodyTrace = {
                x: [x, x2],
                y: [y, y2],
                mode: 'lines+markers',
                type: 'scatter',
                name: 'robot',
                line: { width: 3 },
                marker: { size: 6 }
            };

            const layout = {
                margin: { l: 40, r: 10, t: 10, b: 30 },
                xaxis: {
                    title: { text: 'x (m)', font: { size: 9 } },
                    range: [-2, 2],
                    gridcolor: '#1f2933',
                    zeroline: true
                },
                yaxis: {
                    title: { text: 'y (m)', font: { size: 9 } },
                    range: [-2, 2],
                    gridcolor: '#1f2933',
                    zeroline: true,
                    scaleanchor: 'x'
                },
                showlegend: false,
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: '#f9fafb', size: 9 }
            };

            Plotly.react(plotId, [bodyTrace], layout, { displayModeBar: false, staticPlot: true });

            const poseText = document.getElementById('poseText');
            if (poseText) {
                poseText.innerHTML =
                    `x=<span>${x.toFixed(2)}</span> m, ` +
                    `y=<span>${y.toFixed(2)}</span> m, ` +
                    `θ=<span>${theta.toFixed(2)}</span> rad`;
            }
        }



        function updateMotorWidget(data) {
            if (!data) return;

            const wheels = [
                { key: 'fl', bar: 'wheel-fl-bar', val: 'wheel-fl-val', dir: 'wheel-fl-dir' },
                { key: 'fr', bar: 'wheel-fr-bar', val: 'wheel-fr-val', dir: 'wheel-fr-dir' },
                { key: 'rl', bar: 'wheel-rl-bar', val: 'wheel-rl-val', dir: 'wheel-rl-dir' },
                { key: 'rr', bar: 'wheel-rr-bar', val: 'wheel-rr-val', dir: 'wheel-rr-dir' },
            ];

            wheels.forEach(w => {
                const raw = typeof data[w.key] === 'number' ? data[w.key] : 0.0;
                const clipped = Math.max(-1.0, Math.min(1.0, raw));
                const magnitude = Math.abs(clipped);

                const barEl = document.getElementById(w.bar);
                const valEl = document.getElementById(w.val);
                const dirEl = document.getElementById(w.dir);

                if (!barEl || !valEl || !dirEl) return;

                // height between 5 percent and 100 percent
                const h = magnitude < 0.02 ? 5 : 5 + 95 * magnitude;
                barEl.style.height = h.toFixed(1) + '%';

                // direction label and bar color
                if (magnitude < 0.02) {
                    dirEl.textContent = 'IDLE';
                    barEl.style.background =
                        'linear-gradient(180deg, rgba(148, 163, 184, 0.4), rgba(55, 65, 81, 0.2))';
                } else if (clipped > 0) {
                    dirEl.textContent = 'FWD';
                    barEl.style.background =
                        'linear-gradient(180deg, rgba(249, 250, 251, 0.95), rgba(156, 163, 175, 0.3))';
                } else {
                    dirEl.textContent = 'REV';
                    barEl.style.background =
                        'linear-gradient(180deg, rgba(248, 113, 113, 0.95), rgba(127, 29, 29, 0.5))';
                }

                valEl.textContent = clipped.toFixed(2);
            });
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

        function computeDesiredMotion() {
            // need at least two points and valid resolution
            if (trajectoryPoints.length < MIN_TRAJ_POINTS || trajMaxX <= 0 || trajMaxY <= 0) {
                desiredMotion = {
                    vx: 0,
                    vy: 0,
                    ax: 0,
                    ay: 0,
                    hasData: false
                };
                return desiredMotion;
            }

            const n = trajectoryPoints.length;
            const pLast = trajectoryPoints[n - 1];
            const pPrev = trajectoryPoints[n - 2];
            const pFirst = trajectoryPoints[0];

            const dt = (pLast.t - pPrev.t) / 1000.0; // seconds
            const cx = trajMaxX / 2;
            const cy = trajMaxY / 2;

            // image-plane error: from frame center to current detection
            const ex = pLast.x - cx;
            const ey = pLast.y - cy;

            // approximate measured velocity in image plane (object)
            let vx_meas = 0;
            let vy_meas = 0;
            if (dt > 0) {
                vx_meas = (pLast.x - pPrev.x) / dt;
                vy_meas = (pLast.y - pPrev.y) / dt;
            }

            // linear regression of trajectory in image plane
            const fit = computeLinearFit(trajectoryPoints);
            if (!fit) {
                desiredMotion = {
                    vx: 0,
                    vy: 0,
                    ax: 0,
                    ay: 0,
                    hasData: false
                };
                return desiredMotion;
            }

            // unit direction along line of best fit
            let dirX;
            let dirY;
            if (fit.slope === null) {
                // vertical line
                dirX = 0;
                dirY = 1;
            } else {
                dirX = 1;
                dirY = fit.slope;
                const mag = Math.hypot(dirX, dirY);
                if (mag > 0) {
                    dirX /= mag;
                    dirY /= mag;
                }
            }

            // choose sign so that direction matches overall motion (first to last)
            const dxFL = pLast.x - pFirst.x;
            const dyFL = pLast.y - pFirst.y;
            const sign = (dxFL * dirX + dyFL * dirY) >= 0 ? 1 : -1;
            dirX *= sign;
            dirY *= sign;

            // signed distance from frame center to last point along the fitted line
            const proj = ex * dirX + ey * dirY;

            // desired velocity - move opposite along the trajectory so object tends toward center
            const k_v = 1.0;
            const vx_des = -k_v * proj * dirX;
            const vy_des = -k_v * proj * dirY;

            // desired acceleration chases the desired velocity
            const k_a = 0.5;
            const ax_des = k_a * (vx_des - vx_meas);
            const ay_des = k_a * (vy_des - vy_meas);

            desiredMotion = {
                vx: vx_des,
                vy: vy_des,
                ax: ax_des,
                ay: ay_des,
                hasData: true
            };

            return desiredMotion;
        }



        function updateMotionWidget() {
            const plotId = 'motionVectorPlot';
            const plotEl = document.getElementById(plotId);
            if (!plotEl) {
                return;
            }

            const layout = {
                margin: { l: 30, r: 10, t: 10, b: 30 },
                xaxis: {
                    title: { text: 'vx / ax (norm)', font: { size: 9 } },
                    range: [-1, 1],
                    gridcolor: '#1f2933',
                    zeroline: true
                },
                yaxis: {
                    title: { text: 'vy / ay (norm)', font: { size: 9 } },
                    range: [-1, 1],
                    gridcolor: '#1f2933',
                    zeroline: true,
                    scaleanchor: 'x'
                },
                showlegend: true,
                legend: {
                    orientation: 'h',
                    y: -0.2,
                    x: 0.0
                },
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: '#f9fafb', size: 9 }
            };

            const motion = computeDesiredMotion();

            const velTextEl = document.getElementById('motionVelocityText');
            const accTextEl = document.getElementById('motionAccelText');

            if (!motion.hasData) {
                Plotly.react(plotId, [], layout, { displayModeBar: false, staticPlot: true });
                if (velTextEl) {
                    velTextEl.textContent = 'v_des: vx=0.0, vy=0.0 px/s';
                }
                if (accTextEl) {
                    accTextEl.textContent = 'a_des: ax=0.0, ay=0.0 px/s²';
                }
                return;
            }

            // normalize vectors so they fit nicely in [-1, 1] and flip y for display
            const vMag = Math.hypot(motion.vx, motion.vy);
            const aMag = Math.hypot(motion.ax, motion.ay);
            const maxMag = Math.max(vMag, aMag, 1e-6);
            const scale = 0.8 / maxMag;

            function normForPlot(x, y) {
                return {
                    x: x * scale,
                    y: -y * scale   // invert y so "up" is up in the widget
                };
            }

            const vNorm = normForPlot(motion.vx, motion.vy);
            const aNorm = normForPlot(motion.ax, motion.ay);

            const traces = [
                {
                    x: [0, vNorm.x],
                    y: [0, vNorm.y],
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'v_des',
                    line: { width: 3 },
                    marker: { size: 6 }
                },
                {
                    x: [0, aNorm.x],
                    y: [0, aNorm.y],
                    mode: 'lines+markers',
                    type: 'scatter',
                    name: 'a_des',
                    line: { width: 3, dash: 'dot' },
                    marker: { size: 6 }
                }
            ];

            Plotly.react(plotId, traces, layout, { displayModeBar: false, staticPlot: true });

            if (velTextEl) {
                velTextEl.textContent =
                    `v_des: vx=${motion.vx.toFixed(1)}, vy=${motion.vy.toFixed(1)} px/s`;
            }
            if (accTextEl) {
                accTextEl.textContent =
                    `a_des: ax=${motion.ax.toFixed(1)}, ay=${motion.ay.toFixed(1)} px/s²`;
            }
        }

                function updateRegressionPlot() {
            const plotId = 'regressionPlot';
            const plotEl = document.getElementById(plotId);
            if (!plotEl) {
                return;
            }

            const now = Date.now();
            // keep the same time window as trajectory
            trajectoryPoints = trajectoryPoints.filter(
                p => now - p.t <= TRAJECTORY_WINDOW_MS
            );

            const layout = {
                margin: { l: 40, r: 10, t: 10, b: 30 },
                xaxis: {
                    title: { text: 'X (px)', font: { size: 9 } },
                    range: [0, trajMaxX],
                    gridcolor: '#1f2933',
                    zeroline: false
                },
                yaxis: {
                    title: { text: 'Y (px)', font: { size: 9 } },
                    range: [trajMaxY, 0],
                    gridcolor: '#1f2933',
                    zeroline: false,
                    scaleanchor: 'x'
                },
                showlegend: true,
                legend: {
                    orientation: 'h',
                    y: -0.2,
                    x: 0.0
                },
                paper_bgcolor: 'rgba(0,0,0,0)',
                plot_bgcolor: 'rgba(0,0,0,0)',
                font: { color: '#f9fafb', size: 9 }
            };

            if (trajectoryPoints.length < MIN_TRAJ_POINTS) {
                Plotly.react(plotId, [], layout, { displayModeBar: false, staticPlot: true });
                return;
            }

            const xs = trajectoryPoints.map(p => p.x);
            const ys = trajectoryPoints.map(p => p.y);

            const scatterTrace = {
                x: xs,
                y: ys,
                mode: 'markers',
                type: 'scatter',
                name: 'trajectory',
                marker: { size: 6 }
            };

            const fit = computeLinearFit(trajectoryPoints);
            let lineTrace = null;

            if (fit) {
                if (fit.slope === null) {
                    // vertical line at meanX
                    const x0 = fit.meanX;
                    const x1 = fit.meanX;
                    lineTrace = {
                        x: [x0, x1],
                        y: [0, trajMaxY],
                        mode: 'lines',
                        type: 'scatter',
                        name: 'line of best fit',
                        line: { width: 2, dash: 'dot' }
                    };
                } else {
                    const x0 = 0;
                    const x1 = trajMaxX;
                    const y0 = fit.slope * x0 + fit.intercept;
                    const y1 = fit.slope * x1 + fit.intercept;
                    lineTrace = {
                        x: [x0, x1],
                        y: [y0, y1],
                        mode: 'lines',
                        type: 'scatter',
                        name: 'line of best fit',
                        line: { width: 2, dash: 'dot' }
                    };
                }
            }

            const traces = lineTrace ? [scatterTrace, lineTrace] : [scatterTrace];

            Plotly.react(plotId, traces, layout, { displayModeBar: false, staticPlot: true });
        }


        function computeLinearFit(points) {
            const n = points.length;
            if (n < MIN_TRAJ_POINTS) {
                return null;
            }

            let sumX = 0;
            let sumY = 0;
            for (const p of points) {
                sumX += p.x;
                sumY += p.y;
            }
            const meanX = sumX / n;
            const meanY = sumY / n;

            let num = 0;
            let den = 0;
            for (const p of points) {
                const dx = p.x - meanX;
                const dy = p.y - meanY;
                num += dx * dy;
                den += dx * dx;
            }

            // null slope means vertical line x = meanX
            const slope = den === 0 ? null : num / den;
            const intercept = slope === null ? null : (meanY - slope * meanX);

            return { slope, intercept, meanX, meanY };
        }




        function updateStats(data) {
            // const detEl = document.getElementById('statDetections');
            const fpsEl = document.getElementById('statFPS');
            const trajEl = document.getElementById('statTrajectory');
            const logsEl = document.getElementById('statLogs');
            // const trackEl = document.getElementById('statTracking');
            const resEl = document.getElementById('statResolution');
            const modeTag = document.getElementById('cameraModeTag');
            const globalIndicator = document.getElementById('globalStatusIndicator');
            const globalText = document.getElementById('globalStatusText');

            // detEl.textContent = data.detection_count;
            fpsEl.textContent = `${data.fps.toFixed(1)} fps`;
            trajEl.textContent = trajectoryPoints.length;

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

            // if (data.tracking_enabled) {
            //     trackEl.textContent = 'Active';
            //     trackEl.classList.remove('danger');
            //     trackEl.classList.add('success');
            //     globalIndicator.style.background = '#22c55e';
            //     globalIndicator.style.boxShadow = '0 0 10px rgba(34, 197, 94, 0.9)';
            //     globalText.textContent = 'Tracking Online';
            // } else {
            //     trackEl.textContent = 'Paused';
            //     trackEl.classList.remove('success');
            //     trackEl.classList.add('danger');
            //     globalIndicator.style.background = '#ef4444';
            //     globalIndicator.style.boxShadow = '0 0 10px rgba(239, 68, 68, 0.9)';
            //     globalText.textContent = 'Tracking Paused';
            // }

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
            tag.textContent = `${logEntries} logs`;
            const statLogs = document.getElementById('statLogs');
            if (statLogs && !Number.isNaN(logEntries)) {
                statLogs.textContent = logEntries;
            }
        }

        updateIdleStatusIfStale();
        updatePlot();
        updateMotionWidget();
        updateRegressionPlot();

        setInterval(updatePlot, 100);
        setInterval(updateMotionWidget, 100);
        setInterval(updateRegressionPlot, 100);
        setInterval(updateIdleStatusIfStale, 100);
