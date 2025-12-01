## 🚗 Driving Trash Can

Welcome! This project is a fun experiment in robotics, computer vision, and web control, built around a Raspberry Pi-powered trash can that can drive itself using mecanum wheels and a camera.

### What is this?
This is a Python-based system that lets a trash can (or any robot with mecanum wheels) track and follow a brown box using a camera, plan its motion, and drive itself around. It features:

- **Real-time object tracking** (finds a brown box in the camera feed)
- **Motion planning** (figures out how to move toward the target)
- **Mecanum wheel control** (lets the robot move in any direction)
- **Odometry** (estimates the robot's position)
- **Web dashboard** (see video, stats, and control the robot from your browser)

### How does it work?
1. **Camera**: The robot uses an Arducam (or similar) to capture video.
2. **Object Tracking**: The system detects a brown box in the video stream using color filtering and motion gating.
3. **Motion Planning**: It computes the best way to move toward the target using a simple regression and control logic.
4. **Mecanum Control**: The robot's wheels are controlled to move in any direction, using PID controllers for smooth motion.
5. **Web Interface**: A Flask web server streams video and stats, and lets you control tracking from your browser.

### Project Structure
- `object_tracker.py` — Main app: camera, tracking, Flask server, and robot control
- `motion_planner.py` — Computes desired motion from detected positions
- `mecanum_controller.py` — Maps motion to wheel commands
- `odometry.py` — Estimates robot position from wheel commands
- `static/` and `templates/` — Web dashboard (HTML/CSS/JS)
- `config.py` — Tuning parameters

### How do I run it?
1. **Install dependencies:**
	```sh
	pip install -r requirements.txt
	```
2. **Connect your camera and STM32 (for motor control).**
3. **Start the app:**
	```sh
	python object_tracker.py
	```
4. **Open your browser:**
	Go to `http://localhost:5000` to see the dashboard.

### What can I see on the dashboard?
- Live video feed from the robot
- Trajectory and motion plots
- System stats (FPS, tracking status, etc.)
- Logs of detections
- Controls to start/stop tracking and clear logs

### Can I use this for my own robot?
Absolutely! The code is modular and can be adapted for other robots with mecanum wheels and a camera. Tweak the color detection, PID gains, and motion logic as needed.

---
Made with 🗑️ and ❤️ for robotics experiments.



create wifi hotspot
sudo nmcli dev wifi hotspot ifname wlan0 con-name trashcan-hotspot ssid TrashCanNet password 'trash1234'

ip addr show wlan0

http://10.42.0.1:5000

turn it off
sudo nmcli connection down trashcan-hotspot
sudo nmcli radio wifi off


turn it on
sudo nmcli radio wifi on
sudo nmcli connection up trashcan-hotspot