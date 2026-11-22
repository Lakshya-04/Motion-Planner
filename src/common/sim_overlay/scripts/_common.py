"""Shared node plumbing for the overlay nodes (imports rclpy; the logic is in sim_overlay/)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

DEFAULT_TOPICS = ['/mp_viz/plan_columns', '/mp_viz/stands']


def declare_common(node):
    p = node.declare_parameter
    return {
        'topics': list(p('topics', DEFAULT_TOPICS).value),
        'tf': [float(p(k, 0.0).value) for k in ('frame_x', 'frame_y', 'frame_z', 'frame_yaw')],
    }


def subscribe(node, topics, state, on_change):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import MarkerArray
    from sim_overlay.marker_io import array_to_dicts
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    for t in topics:
        def cb(msg, topic=t):
            state.update(topic, array_to_dicts(msg))
            on_change()
        node.create_subscription(MarkerArray, t, cb, qos)
