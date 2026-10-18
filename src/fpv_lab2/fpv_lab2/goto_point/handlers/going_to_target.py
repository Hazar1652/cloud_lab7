from typing import TYPE_CHECKING

from ...flight_test.handlers.handler import FlightStateHandler
from ..goto_state import GotoState

if TYPE_CHECKING:
    from ..goto_point_node import GotoPointNode


class GoingToTargetHandler(FlightStateHandler):

    LOG_PERIOD = 5

    def __init__(self):
        self._tick = 0

    def handle(self, node: "GotoPointNode") -> None:
        if node.was_disarmed():
            node.fail("Drone disarmed unexpectedly during flight")
            return

        pos = node.xyz()
        dist = node.range_xy()

        if pos is None or dist is None:
            return

        if dist <= node.tol:
            node.halt()
            node.get_logger().info(
                f"Target reached, distance {dist:.3f} m "
                f"(tolerance {node.tol:.2f} m)"
            )
            node.state = GotoState.STOPPING
            return

        dx = node.goal_xy[0] - pos[0]
        dy = node.goal_xy[1] - pos[1]
        speed = max(node.v_min, node.v_max * min(1.0, dist / node.brake_r))
        node.drive(speed * dx / dist, speed * dy / dist)

        self._tick += 1
        if self._tick % self.LOG_PERIOD == 0:
            node.get_logger().info(
                f"x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f} "
                f"| distance {dist:.2f} m | speed {speed:.2f} m/s"
            )
