#!/usr/bin/env python3
"""Draws the marker topics in Gazebo through the gz-sim /marker service (``gz service``).

ROS mode (default): subscribes to the topics and re-issues the whole scene on every change.
Parameters: topics, frame_x/y/z/yaw, dry_run (print the gz commands instead of running them), service (/marker).
Offline mode, no ROS needed::

    gz_overlay.py --from-json /tmp/mp_sim_overlay.json [--dry-run] [--clear]

``--clear`` removes the overlay namespaces named in the file (or, with ROS mode, the ones this node drew) and exits.
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from sim_overlay.gz_commands import SERVICE, clear_request, gz_command, plan_commands, shell  # noqa: E402
from sim_overlay.mujoco_overlay import load_overlay  # noqa: E402


class GzDrawer:
    def __init__(self, dry_run=False, service=SERVICE, out=print):
        self.dry_run, self.service, self.out = dry_run, service, out
        self.ns = []

    def run(self, argv):
        if self.dry_run:
            self.out(shell(argv))
            return True
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.SubprocessError) as e:
            self.out('gz failed: %s' % e)
            return False
        if r.returncode != 0 or 'data: false' in r.stdout:
            self.out('gz service rejected: %s %s' % (r.stdout.strip(), r.stderr.strip()))
            return False
        return True

    def show(self, prims):
        cmds, self.ns = plan_commands(prims, self.ns, self.service)
        return sum(1 for c in cmds if self.run(c)), len(cmds)

    def clear(self):
        for n in self.ns:
            self.run(gz_command(clear_request(n), self.service))
        self.ns = []


def _ros_main(args):
    import rclpy
    from rclpy.node import Node
    from _common import declare_common, subscribe
    from sim_overlay.overlay import FrameTransform, OverlayState
    rclpy.init()
    node = Node('gz_overlay')
    c = declare_common(node)
    drawer = GzDrawer(bool(node.declare_parameter('dry_run', False).value or args.dry_run),
                      node.declare_parameter('service', SERVICE).value, out=node.get_logger().info)
    state = OverlayState(FrameTransform(*c['tf']))

    def draw():
        ok, n = drawer.show(state.primitives())
        node.get_logger().info('gz: %d/%d command(s) ok' % (ok, n))
    subscribe(node, c['topics'], state, draw)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    drawer.clear()
    node.destroy_node()
    rclpy.try_shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--from-json')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--clear', action='store_true')
    args, _ = ap.parse_known_args(argv)
    if not args.from_json:
        return _ros_main(args)
    prims = load_overlay(args.from_json)
    d = GzDrawer(args.dry_run)
    if args.clear:
        d.ns = sorted({p['ns'] or 'default' for p in prims})
        d.clear()
    else:
        d.show(prims)


if __name__ == '__main__':
    main()
