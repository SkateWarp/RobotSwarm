"""Public utility helpers.

Keep these imports lazy. Some small helpers, such as the shutdown publication
lane, are also used by the dependency-light unit tests and should not require a
complete ROS installation just to import their package.
"""


def load_config(*args, **kwargs):
    from .config import load_config as _load_config

    return _load_config(*args, **kwargs)


def setup_logger(*args, **kwargs):
    from .logger import setup_logger as _setup_logger

    return _setup_logger(*args, **kwargs)

__all__ = ['load_config', 'setup_logger']
