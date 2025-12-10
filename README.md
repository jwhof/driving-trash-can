## Driving Trash Can

A trash can that follows trash thrown at it using motion planning, real-time object tracking, and a cool web dashboard :)


### Installation
**install dependencies:**
```sh
pip install -r requirements.txt
```
- **connect your camera and STM32 (for motor control).**
**Start the app:**
```sh
python object_tracker.py
```
**open your browser:**
Go to `http://localhost:5000` to see the dash

### Dashboard features
- Live video feed from the robot
- Trajectory and motion plots
- System stats (FPS, tracking status, etc.)
- Logs of detections
- Controls to start/stop tracking and clear logs


### wifi hotspot info (if running on rpi)

**create wifi hotspot:**
```sh
sudo nmcli dev wifi hotspot ifname wlan0 con-name trashcan-hotspot ssid TrashCanNet password 'trash1234'
```

```sh
ip addr show wlan0
```

site: `http://10.42.0.1:5000`

**turn it off**
```sh
sudo nmcli connection down trashcan-hotspot
sudo nmcli radio wifi off
```

**turn it on**
```sh
sudo nmcli radio wifi on
sudo nmcli connection up trashcan-hotspot
```