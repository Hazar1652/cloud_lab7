import math
import time
from typing import TYPE_CHECKING

from ...flight_test.flight_state import FlightState
from ...flight_test.handlers.handler import FlightStateHandler

if TYPE_CHECKING:
    from ..goto_point_node import GotoPointNode


class StoppingHandler(FlightStateHandler):

    SETTLE_TIMEOUT = 5.0
    SPEED_EPSILON = 0.15

    def handle(self, node: "GotoPointNode") -> None:
        node.halt()

        if node.brake_at is None:
            node.brake_at = time.monotonic()

        elapsed = time.monotonic() - node.brake_at
        if node.horiz_speed() > self.SPEED_EPSILON and elapsed < self.SETTLE_TIMEOUT:
            return

        pos = node.xyz()
        if pos is not None and node.goal_xy is not None:
            err = math.hypot(pos[0] - node.goal_xy[0], pos[1] - node.goal_xy[1])
            node.result_xyz = pos
            node.result_err = err
            home = node.home_xy
            node.get_logger().info("----- RESULT -----")
            node.get_logger().info(
                f"Start position:  x={home[0]:.3f} y={home[1]:.3f}"
            )
            node.get_logger().info(
                f"Target position: x={node.goal_xy[0]:.3f} "
                f"y={node.goal_xy[1]:.3f}"
            )
            node.get_logger().info(
                f"Actual position: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}"
            )
            node.get_logger().info(f"Position error:  e={err:.3f} m")
            node.get_logger().info("------------------")

        node.get_logger().info("Horizontal motion stopped, starting landing")
        node.state = FlightState.LANDING
