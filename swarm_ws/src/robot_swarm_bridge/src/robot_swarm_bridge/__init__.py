"""Public package entry points.

The ROS bridge is imported lazily so ROS-independent modules, including the
swarm algorithm kernels, can be imported and tested on machines without ROS.
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .bridge import RobotSwarmBridge

__all__ = ['RobotSwarmBridge']


def __getattr__(name):
    if name == 'RobotSwarmBridge':
        from .bridge import RobotSwarmBridge
        return RobotSwarmBridge
    raise AttributeError("module {!r} has no attribute {!r}".format(__name__, name))
