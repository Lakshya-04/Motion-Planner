#!/usr/bin/env python3
"""Sends one `execute_coverage` goal (coverage_task_msgs/ExecuteCoverage) to the manipulation manager, prints the
feedback as the state changes and the result at the end (run_multi_stand_pass.sh starts everything).

    execute_coverage_client.py [--multi-stand] [--columns 0 1 ..] [--tool-mode 0|1|2] [--cancel-after S]

With --cancel-after the goal is cancelled S seconds after it was accepted; the script then checks that the arm stopped
(joint velocities of the following second). Exit code: 0 when the goal succeeded (or, with --cancel-after, ended
CANCELED with the arm at rest), 1 otherwise.
"""
import argparse
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from coverage_task_msgs.action import ExecuteCoverage
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--coverage', default='', help='coverage_output.json (empty: the manager parameter)')
    ap.add_argument('--multi-stand', action='store_true')
    ap.add_argument('--columns', type=int, nargs='*', default=[])
    ap.add_argument('--tool-mode', type=int, default=0)
    ap.add_argument('--cancel-after', type=float, default=0.0)
    ap.add_argument('--timeout', type=float, default=3000.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node('execute_coverage_client')
    client = ActionClient(node, ExecuteCoverage, 'execute_coverage')
    vel = {'t': 0.0, 'max': 0.0}

    def on_joints(msg):
        vel['t'] = time.monotonic()
        vel['max'] = max([abs(v) for v in msg.velocity] or [0.0])
    node.create_subscription(JointState, '/joint_states', on_joints, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    t0 = time.monotonic()
    while not client.server_is_ready():
        if time.monotonic() - t0 > 120.0:
            print('client: execute_coverage server not up')
            os._exit(1)
        time.sleep(0.2)

    goal = ExecuteCoverage.Goal()
    goal.coverage_json_path = args.coverage
    goal.multi_stand = args.multi_stand
    goal.tool_mode = args.tool_mode
    goal.columns = args.columns
    last = {'state': None}

    def on_feedback(fb):
        f = fb.feedback
        key = (f.state, f.stand_index, f.column)
        if key != last['state']:
            last['state'] = key
            print(f'feedback: state {f.state} stand {f.stand_index}/{f.stand_count} column {f.column} '
                  f'done {f.columns_done} failed {f.columns_failed} peak {f.peak_force_n:.1f} N', flush=True)

    sent = client.send_goal_async(goal, feedback_callback=on_feedback)
    while not sent.done():
        time.sleep(0.05)
    handle = sent.result()
    if not handle.accepted:
        print('client: goal rejected')
        os._exit(1)
    t_acc = time.monotonic()
    result_future = handle.get_result_async()
    cancelled_at = None
    while not result_future.done() and time.monotonic() - t_acc < args.timeout:
        if args.cancel_after > 0 and cancelled_at is None and time.monotonic() - t_acc > args.cancel_after:
            print(f'client: cancelling after {args.cancel_after:.0f} s', flush=True)
            handle.cancel_goal_async()
            cancelled_at = time.monotonic()
        time.sleep(0.05)
    if not result_future.done():
        print('client: timed out')
        os._exit(1)
    wrapped = result_future.result()
    r = wrapped.result
    status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED',
              GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(wrapped.status, str(wrapped.status))
    print(f'result: status {status} success {r.success} message "{r.message}" peak_force_n {r.peak_force_n:.2f}')
    print(f'columns_done {list(r.columns_done)} partial {list(r.columns_partial)} failed {list(r.columns_failed)}')
    for s in r.stands:
        print(f'stand {s.stand_index}: reached {s.reached} done {list(s.columns_done)} partial {list(s.columns_partial)} '
              f'failed {list(s.columns_failed)} "{s.message}"')
    if cancelled_at is not None:
        print(f'cancel latency {time.monotonic() - cancelled_at:.2f} s')
        time.sleep(1.0)
        peak = 0.0
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            peak = max(peak, vel['max'])
            time.sleep(0.02)
        print(f'joint speed after the cancel (max over 1 s) {peak:.4f} rad/s (m/s)')
        os._exit(0 if status == 'CANCELED' and peak < 0.01 else 1)
    os._exit(0 if status == 'SUCCEEDED' and r.success else 1)


if __name__ == '__main__':
    main()
