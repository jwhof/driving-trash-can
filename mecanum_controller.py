from dataclasses import dataclass, asdict
from typing import Optional

import config
from motion_planner import MotionResult


@dataclass
class MotorCommand:
    fl: float  # front left
    fr: float  # front right
    rl: float  # rear left
    rr: float  # rear right

    def to_dict(self):
        return asdict(self)


class PID:
    def __init__(self, kp: float, ki: float, kd: float, clamp: Optional[tuple[float, float]] = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.clamp = clamp

        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.clamp is not None:
            lo, hi = self.clamp
            if out < lo:
                out = lo
            elif out > hi:
                out = hi

        return out


class MecanumController:
    """Map image space desired accel (from MotionPlanner) to mecanum wheel powers."""

    def __init__(self):
        clamp = config.PID_OUTPUT_CLAMP

        self.pid_x = PID(
            kp=config.PID_KP_X,
            ki=config.PID_KI_X,
            kd=config.PID_KD_X,
            clamp=clamp,
        )
        self.pid_y = PID(
            kp=config.PID_KP_Y,
            ki=config.PID_KI_Y,
            kd=config.PID_KD_Y,
            clamp=clamp,
        )

    def compute_manual(self, vx: float, vy: float, omega: float = 0.0) -> MotorCommand:
        """
        Directly map inputs to wheel powers without PID.
        vx, vy: -1.0 to 1.0 (from arrow keys)
        """
        # Map inputs to robot frame 
        # Image Up (negative Y) -> Robot Forward (+x)
        # Image Left (negative X) -> Robot Strafe Left (+y)
        v_forward = -vy
        v_strafe = -vx
        
        # Mecanum mixing
        # fl = forward - strafe - rotate
        fl = v_forward - v_strafe - omega
        fr = v_forward + v_strafe + omega
        rl = v_forward + v_strafe - omega
        rr = v_forward - v_strafe + omega

        return MotorCommand(fl=fl, fr=fr, rl=rl, rr=rr)

    def compute(self, motion: MotionResult, dt: float) -> Optional[MotorCommand]:
        if not motion.has_data:
            return None

        # treat desired acceleration as setpoint; for now assume measured accel = 0
        err_x = motion.ax
        err_y = motion.ay

        u_x = self.pid_x.step(err_x, dt)
        u_y = self.pid_y.step(err_y, dt)



        # map from image frame to robot frame
        # image x right  -> robot +y (strafe right)
        # image y down   -> robot +x (forward)
        # move opposite the error
        v_forward = -u_y
        v_strafe = -u_x
        omega = 0.0  # no rotation term yet

        # mecanum mixing (front left, front right, rear left, rear right)
        fl = v_forward - v_strafe - omega
        fr = v_forward + v_strafe + omega
        rl = v_forward + v_strafe - omega
        rr = v_forward - v_strafe + omega

        # normalize to [-1, 1]
        max_abs = max(abs(fl), abs(fr), abs(rl), abs(rr), 1e-6)
        fl /= max_abs
        fr /= max_abs
        rl /= max_abs
        rr /= max_abs

        return MotorCommand(fl=fl, fr=fr, rl=rl, rr=rr)
