"""Isaac Sim backend: a stub. Exits at once with a message, so it cannot be mistaken for a backend.

See docs/simulation.md for the contract a backend has to meet and the plan for this one.
"""
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo
from launch.events import Shutdown

NOT_IMPLEMENTED = (
    'sim_isaac is a stub: the Isaac Sim backend is not implemented yet. '
    'See docs/simulation.md for the backends and their status.')


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=NOT_IMPLEMENTED),
        EmitEvent(event=Shutdown(reason=NOT_IMPLEMENTED)),
    ])
