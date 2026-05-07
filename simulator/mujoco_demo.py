"""Minimal MuJoCo adapter placeholder.

This file is intentionally lightweight at first. Replace the mock methods
with real MuJoCo control once the API contract is stable.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SimPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class MuJoCoDemo:
    def __init__(self) -> None:
        self.pose = SimPose()

    def go_to(self, x: float, y: float) -> SimPose:
        self.pose.x = x
        self.pose.y = y
        return self.pose

    def stop(self) -> None:
        return None

    def capture_image(self) -> str:
        return "sim://camera/front"


if __name__ == "__main__":
    sim = MuJoCoDemo()
    print(sim.go_to(1.0, 2.0))

