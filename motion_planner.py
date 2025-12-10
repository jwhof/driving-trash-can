from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional
import time
import config

@dataclass
class MotionResult:
    has_data: bool
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0

    def to_dict(self):
        return asdict(self)


@dataclass
class RegressionResult:
    has_data: bool
    line_x: Optional[list] = None  # [x0, x1] for line of best fit
    line_y: Optional[list] = None  # [y0, y1]

    def to_dict(self):
        return asdict(self)


class MotionPlanner:
    """Pure backend motion planning in image space.

    Inputs: list of (x, y, t) points in pixel coordinates, where
      - x grows to the right
      - y grows downward
      - t is a timestamp in seconds (time.time())

    Outputs:
      - MotionResult with desired velocity / acceleration in px/s, px/s^2
      - RegressionResult with a simple 2 point representation of the line of
        best fit across the image width.
    """

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

        # gains from config
        self.k_v = config.MOTION_K_V
        self.k_a = config.MOTION_K_A

        # minimum points for any useful regression
        self.min_points = config.MIN_TRAJ_POINTS_FOR_REGRESSION
        
        # Timeout duration in seconds
        self.timeout = 0.5

    def _compute_linear_fit(self, pts: List[Tuple[float, float, float]]):
        n = len(pts)
        if n < self.min_points:
            return None

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]

        mean_x = sum(xs) / n
        mean_y = sum(ys) / n

        num = 0.0
        den = 0.0
        for x, y in zip(xs, ys):
            dx = x - mean_x
            dy = y - mean_y
            num += dx * dy
            den += dx * dx

        if den == 0.0:
            # vertical line x = mean_x
            slope = None
            intercept = None
        else:
            slope = num / den
            intercept = mean_y - slope * mean_x

        return {
            "slope": slope,
            "intercept": intercept,
            "mean_x": mean_x,
            "mean_y": mean_y,
        }

    def compute(self, pts: List[Tuple[float, float, float]]) -> tuple[MotionResult, RegressionResult]:
        """Compute desired motion and regression from a trajectory segment."""

        # Basic validation
        if len(pts) < self.min_points or self.width <= 0 or self.height <= 0:
            return MotionResult(has_data=False), RegressionResult(has_data=False)

        # 1. SAFETY TIMEOUT CHECK
        # Check if the last data point is older than 0.5 seconds
        last_timestamp = pts[-1][2]
        if (time.time() - last_timestamp) > self.timeout:
            # Return explicit zero motion to stop motors
            return (
                MotionResult(has_data=True, vx=0.0, vy=0.0, ax=0.0, ay=0.0),
                RegressionResult(has_data=False)
            )

        # last two points
        x_last, y_last, t_last = pts[-1]
        x_prev, y_prev, t_prev = pts[-2]

        dt = t_last - t_prev if t_last > t_prev else 0.0
        cx = self.width / 2.0
        cy = self.height / 2.0

        # image plane error from center to current detection
        ex = x_last - cx
        ey = y_last - cy

        # measured velocity in image plane
        vx_meas = 0.0
        vy_meas = 0.0
        if dt > 0.0:
            vx_meas = (x_last - x_prev) / dt
            vy_meas = (y_last - y_prev) / dt

        # regression fit for direction
        fit = self._compute_linear_fit(pts)
        if fit is None:
            return MotionResult(has_data=False), RegressionResult(has_data=False)

        slope = fit["slope"]
        mean_x = fit["mean_x"]

        # unit direction along fitted line
        if slope is None:
            dir_x = 0.0
            dir_y = 1.0
        else:
            dir_x = 1.0
            dir_y = slope
            mag = (dir_x ** 2 + dir_y ** 2) ** 0.5
            if mag > 0.0:
                dir_x /= mag
                dir_y /= mag

        # choose sign so direction matches overall motion from first to last
        x_first, y_first, _ = pts[0]
        dx_fl = x_last - x_first
        dy_fl = y_last - y_first
        sign = 1.0 if (dx_fl * dir_x + dy_fl * dir_y) >= 0.0 else -1.0
        dir_x *= sign
        dir_y *= sign

        # signed distance along the fitted direction from center to last point
        proj = ex * dir_x + ey * dir_y

        # desired velocity is opposite this projection along the line
        vx_des = -self.k_v * proj * dir_x
        vy_des = -self.k_v * proj * dir_y

        # desired acceleration chases desired velocity
        ax_des = self.k_a * (vx_des - vx_meas)
        ay_des = self.k_a * (vy_des - vy_meas)

        motion = MotionResult(
            has_data=True,
            vx=vx_des,
            vy=vy_des,
            ax=ax_des,
            ay=ay_des,
        )

        # regression line for visualization: two points across the width
        if slope is None:
            # vertical line at mean_x
            x0 = mean_x
            x1 = mean_x
            y0 = 0.0
            y1 = float(self.height)
        else:
            x0 = 0.0
            x1 = float(self.width)
            intercept = fit["intercept"]
            y0 = slope * x0 + intercept
            y1 = slope * x1 + intercept

        reg = RegressionResult(
            has_data=True,
            line_x=[x0, x1],
            line_y=[y0, y1],
        )

        return motion, reg