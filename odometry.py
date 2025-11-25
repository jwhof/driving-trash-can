# odometry.py
from dataclasses import dataclass, asdict
from mecanum_controller import MotorCommand
import math


# rough guesses for now. tune once you know real robot numbers
V_MAX = 1.5      # max wheel linear speed in m/s
LX = 0.20        # half of wheelbase length (front back) in meters
LY = 0.20        # half of track width (left right) in meters


@dataclass
class Pose:
    x: float = 0.0     # meters, robot frame integrated in world
    y: float = 0.0
    theta: float = 0.0  # radians, 0 = facing +x

    def to_dict(self):
        return asdict(self)


class MecanumOdometry:
    """Very simple open loop odometry from commanded mecanum wheel powers."""

    def __init__(self):
        self.pose = Pose()

    def reset(self):
        self.pose = Pose()

    def step(self, cmd: MotorCommand, dt: float) -> Pose:
        if dt <= 0.0:
            dt = 1e-3

        # convert normalized wheel commands to linear wheel speeds
        v_fl = cmd.fl * V_MAX
        v_fr = cmd.fr * V_MAX
        v_rl = cmd.rl * V_MAX
        v_rr = cmd.rr * V_MAX

        # mecanum forward kinematics in body frame
        # convention:
        #   v_x forward
        #   v_y left
        #   omega positive CCW
        v_x = (v_fl + v_fr + v_rl + v_rr) / 4.0
        v_y = (-v_fl + v_fr + v_rl - v_rr) / 4.0
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (LX + LY))

        # integrate in world frame
        theta_mid = self.pose.theta + 0.5 * omega * dt
        cos_th = math.cos(theta_mid)
        sin_th = math.sin(theta_mid)

        # rotate body velocity into world
        dx = v_x * cos_th - v_y * sin_th
        dy = v_x * sin_th + v_y * cos_th

        self.pose.x += dx * dt
        self.pose.y += dy * dt
        self.pose.theta += omega * dt

        return self.pose
