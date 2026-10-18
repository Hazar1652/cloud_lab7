import time
from typing import TYPE_CHECKING

from ...flight_test.handlers.handler import FlightStateHandler
from ..goto_state import GotoState

if TYPE_CHECKING:
    from ..goto_point_node import GotoPointNode


class StabilizingHandler(FlightStateHandler):

    ALTITUDE_FRACTION = 0.9
    TAKEOFF_TIMEOUT = 60.0

    def handle(self, node: "GotoPointNode") -> None:
        if node.was_disarmed():
            node.fail("Drone disarmed unexpectedly during flight")
            return

        if node.xyz() is None:
            return

        alt = node.height()
        if alt is None:
            return

        elapsed = time.monotonic() - node.hover_started_at
        needed = node.TAKEOFF_ALTITUDE * self.ALTITUDE_FRACTION

        if alt < needed:
            if elapsed > self.TAKEOFF_TIMEOUT:
                node.fail(
                    f"Take-off timed out, altitude {alt:.2f} m "
                    f"instead of {node.TAKEOFF_ALTITUDE:.2f} m"
                )
            return

        if node.up_at is None:
            node.up_at = time.monotonic()
            node.get_logger().info(
                f"Take-off altitude reached: {alt:.2f} m "
                f"above the start point (target {node.TAKEOFF_ALTITUDE:.1f} m)"
            )
            return

        if time.monotonic() - node.up_at < node.hold_s:
            return

        if node.goal_xy is None:
            node.fail("No odometry received, target point is unknown")
            return

        node.get_logger().info(
            f"Hover stabilised at {alt:.2f} m for "
            f"{node.hold_s:.0f} s, starting flight to the target"
        )
        node.state = GotoState.GOING_TO_TARGET
