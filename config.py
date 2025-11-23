# config.py
"""Global configuration for BALL-E backend.

Keep this file small and focused: only core tuning parameters and
hardware endpoints live here.
"""

# camera / image geometry
CAMERA_INDEX = 0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080

# minimum number of trajectory points required before computing a
# regression line / desired motion
MIN_TRAJ_POINTS_FOR_REGRESSION = 4

# motion planner gains in image space
MOTION_K_V = 1.0   # velocity gain along fitted line
MOTION_K_A = 0.5   # accel gain toward desired velocity

# mecanum PID gains (body frame commands)
PID_KP_X = 0.002
PID_KI_X = 0.0
PID_KD_X = 0.0

PID_KP_Y = 0.002
PID_KI_Y = 0.0
PID_KD_Y = 0.0

# clamp for PID outputs before mecanum mixing
PID_OUTPUT_CLAMP = (-1.5, 1.5)

# STM32 serial link
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.02
