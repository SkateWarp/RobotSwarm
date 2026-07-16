"""Arc-length trajectory storage for natural arbitrary-size leader following."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable, List


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TracePoint:
    distance: float
    x: float
    y: float
    yaw: float


class ArcLengthTrace:
    """Store and sample a path by travelled distance rather than elapsed time."""

    def __init__(self, minimum_step: float = 0.01):
        self.minimum_step = max(float(minimum_step), 1e-6)
        self._points: Deque[TracePoint] = deque()

    @property
    def points(self) -> List[TracePoint]:
        return list(self._points)

    @property
    def total_distance(self) -> float:
        return self._points[-1].distance if self._points else 0.0

    @property
    def span(self) -> float:
        if len(self._points) < 2:
            return 0.0
        return self._points[-1].distance - self._points[0].distance

    def clear(self) -> None:
        self._points.clear()

    def seed_line(
        self,
        x: float,
        y: float,
        yaw: float,
        distance_behind: float,
        sample_step: float = 0.1,
    ) -> None:
        """Seed a straight path ending at the leader's current pose."""
        self.clear()
        length = max(0.0, float(distance_behind))
        steps = max(1, int(math.ceil(length / max(sample_step, self.minimum_step))))
        for index in range(steps + 1):
            distance = length * index / steps
            behind = length - distance
            self._points.append(TracePoint(
                distance=distance,
                x=float(x) - behind * math.cos(yaw),
                y=float(y) - behind * math.sin(yaw),
                yaw=_normalize_angle(float(yaw)),
            ))

    def append(self, x: float, y: float, yaw: float) -> None:
        """Append a pose, coalescing sub-centimetre odometry noise."""
        normalized_yaw = _normalize_angle(float(yaw))
        if not self._points:
            self._points.append(TracePoint(0.0, float(x), float(y), normalized_yaw))
            return

        latest = self._points[-1]
        travelled = math.hypot(float(x) - latest.x, float(y) - latest.y)
        if travelled < self.minimum_step:
            # Keep the last committed sample in place.  The next odometry
            # update is then measured from it, so several small movements can
            # accumulate into a real path step instead of being discarded.
            # Heading can still change while the robot turns in place.
            self._points[-1] = TracePoint(
                latest.distance, latest.x, latest.y, normalized_yaw
            )
            return

        self._points.append(TracePoint(
            latest.distance + travelled,
            float(x),
            float(y),
            normalized_yaw,
        ))

    def ensure_distance(self, distance_behind: float, sample_step: float = 0.1) -> None:
        if not self._points:
            return

        required = max(0.0, float(distance_behind))
        step_size = max(self.minimum_step, float(sample_step))
        while self.span + 1e-9 < required:
            first = self._points[0]
            step = min(step_size, required - self.span)
            self._points.appendleft(TracePoint(
                distance=first.distance - step,
                x=first.x - step * math.cos(first.yaw),
                y=first.y - step * math.sin(first.yaw),
                yaw=first.yaw,
            ))

    def trim(self, retained_distance: float) -> None:
        """Discard path data farther behind than the longest follower needs."""
        retain = max(0.0, float(retained_distance))
        while (
            len(self._points) > 2
            and self.total_distance - self._points[1].distance > retain
        ):
            self._points.popleft()

    def points_behind(self, distances: Iterable[float]) -> List[TracePoint]:
        """Interpolate path points at the requested distances behind the head."""
        requested = [max(0.0, float(distance)) for distance in distances]
        if not requested:
            return []
        if not self._points:
            raise ValueError("cannot sample an empty path trace")

        points = list(self._points)
        result: List[TracePoint] = []
        latest_distance = points[-1].distance

        for distance_behind in requested:
            target_distance = latest_distance - distance_behind
            if target_distance <= points[0].distance:
                result.append(points[0])
                continue
            if target_distance >= latest_distance:
                result.append(points[-1])
                continue

            lower = points[0]
            upper = points[-1]
            for index in range(len(points) - 1, 0, -1):
                candidate_lower = points[index - 1]
                candidate_upper = points[index]
                if candidate_lower.distance <= target_distance <= candidate_upper.distance:
                    lower = candidate_lower
                    upper = candidate_upper
                    break

            span = upper.distance - lower.distance
            fraction = 0.0 if span <= 1e-12 else (target_distance - lower.distance) / span
            yaw_delta = _normalize_angle(upper.yaw - lower.yaw)
            result.append(TracePoint(
                distance=target_distance,
                x=lower.x + fraction * (upper.x - lower.x),
                y=lower.y + fraction * (upper.y - lower.y),
                yaw=_normalize_angle(lower.yaw + fraction * yaw_delta),
            ))

        return result
