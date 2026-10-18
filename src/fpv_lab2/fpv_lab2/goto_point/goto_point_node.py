import math

from geometry_msgs.msg import TwistStamped

from ..flight_test.flight_test_node import FlightTestNode
from .handlers import GOTO_HANDLERS


_PARAM_SPEC = (
    ("delta_x", 0.0),
    ("delta_y", 0.0),
    ("takeoff_altitude", 2.0),
    ("tolerance", 0.25),
    ("max_speed", 1.5),
    ("min_speed", 0.15),
    ("slowdown_radius", 2.5),
    ("stabilize_duration", 5.0),
    ("axis_direction", 1.0),
)


class GotoPointNode(FlightTestNode):

    def __init__(self):
        super().__init__()
        self._handlers = GOTO_HANDLERS

        raw = {}
        for key, fallback in _PARAM_SPEC:
            self.declare_parameter(key, fallback)
            raw[key] = self.get_parameter(key).value

        sign = raw["axis_direction"]
        self._shift = (raw["delta_x"] * sign, raw["delta_y"] * sign)
        self.TAKEOFF_ALTITUDE = raw["takeoff_altitude"]
        self.HOVER_DURATION = raw["stabilize_duration"]
        self.hold_s = raw["stabilize_duration"]
        self.tol = raw["tolerance"]
        self.v_max = raw["max_speed"]
        self.v_min = raw["min_speed"]
        self.brake_r = raw["slowdown_radius"]

        self._ever_armed = False
        self._home = None
        self._goal = None
        self.up_at = None
        self.brake_at = None
        self.result_xyz = None
        self.result_err = None

        self._cmd = self.create_publisher(TwistStamped, "/ap/v1/cmd_vel", 10)

        self.get_logger().info(
            f"Target offset applied: dx={self._shift[0]:+.2f} m, "
            f"dy={self._shift[1]:+.2f} m (axis_direction={sign:+.0f})"
        )

    def status_callback(self, message):
        super().status_callback(message)
        self._ever_armed = self._ever_armed or bool(message.armed)

    def was_disarmed(self):
        if self.status is None:
            return False
        return self._ever_armed and not self.status.armed

    def odometry_callback(self, message):
        super().odometry_callback(message)
        if self._home is not None:
            return

        p = message.pose.pose.position
        self._home = (p.x, p.y, p.z)
        self._goal = (p.x + self._shift[0], p.y + self._shift[1])

        self.get_logger().info(f"Start position:  x0={p.x:.3f} m, y0={p.y:.3f} m")
        self.get_logger().info(
            f"Target position: x={self._goal[0]:.3f} m, y={self._goal[1]:.3f} m"
        )

    @property
    def home_xy(self):
        if self._home is None:
            return None
        return (self._home[0], self._home[1])

    @property
    def goal_xy(self):
        return self._goal

    def xyz(self):
        if self.odometry is None:
            return None
        p = self.odometry.pose.pose.position
        return (p.x, p.y, p.z)

    def height(self):
        now = self.xyz()
        if now is None or self._home is None:
            return None
        return now[2] - self._home[2]

    def range_xy(self):
        now = self.xyz()
        if now is None or self._goal is None:
            return None
        return math.hypot(self._goal[0] - now[0], self._goal[1] - now[1])

    def horiz_speed(self):
        if self.odometry is None:
            return 0.0
        t = self.odometry.twist.twist
        return math.hypot(t.linear.x, t.linear.y)

    def drive(self, east, north):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.twist.linear.x = float(east)
        msg.twist.linear.y = float(north)
        msg.twist.linear.z = 0.0
        msg.twist.angular.z = 0.0
        self._cmd.publish(msg)

    def halt(self):
        self.drive(0.0, 0.0)
