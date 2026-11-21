#!/usr/bin/env python3
"""Optional MuJoCo viewer: shows the generated MJCF and follows /joint_states.

    viewer.py robot.xml joint1 joint2 ...

Needs a display and the `mujoco` Python package (4.x: mujoco-core + mujoco-studio). It mirrors the
joint states, it is not the simulation itself (the physics runs in the ros2_control plugin).
"""
import sys
import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.qos import qos_profile_sensor_data


VIEW_HZ = float(os.environ.get('SIM_MUJOCO_VIEWER_HZ', '15'))


def main():
    import mujoco
    import mujoco.viewer
    path, joints = sys.argv[1], sys.argv[2:]
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    adr = {j: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in joints}
    rclpy.init()
    node = Node('sim_mujoco_viewer')

    def on_state(msg):
        for name, pos in zip(msg.name, msg.position):
            if name in adr:
                data.qpos[adr[name]] = pos
    node.create_subscription(JointState, '/joint_states', on_state, 10)
    # The mimic lift stages (an MJCF <equality><joint> constraint) are not driven by /joint_states and mj_forward does
    # not solve constraints: without this the telescopic lift stays at its default and the tool floats off the wall.
    mimics = []
    for e in range(model.neq):
        if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT:
            mimics.append((int(model.jnt_qposadr[int(model.eq_obj1id[e])]),
                           int(model.jnt_qposadr[int(model.eq_obj2id[e])]), model.eq_data[e][:5].copy()))
    force = [0.0, 0.0]   # latest ground-truth |Fz| and the largest seen

    def on_ft(msg):
        force[0] = abs(msg.wrench.force.z)
        force[1] = max(force[1], force[0])
    node.create_subscription(WrenchStamped, '/sim_mujoco/ft_ground_truth', on_ft, qos_profile_sensor_data)
    # the driven base: /joint_states carries only the lift and the arm, the base pose (x, y, yaw in the odom frame) comes
    # from the plugin as a PoseStamped; without it the arm is drawn at the origin with yaw 0, away from the wall
    base_adr = [int(model.jnt_qposadr[j]) if j >= 0 else -1 for j in (
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ('base_x_joint', 'base_y_joint', 'base_yaw_joint'))]

    def on_base(msg):
        p, o = msg.pose.position, msg.pose.orientation
        for adr, v in zip(base_adr, (p.x, p.y, math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z)))):
            if adr >= 0:
                data.qpos[adr] = v
    node.create_subscription(PoseStamped, '/sim_mujoco/base_pose', on_base, qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            for a1, a2, c in mimics:
                x = data.qpos[a2]
                data.qpos[a1] = sum(c[k] * x ** k for k in range(5))
            mujoco.mj_forward(model, data)
            try:
                viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, 'contact force (N)  peak',
                                  f'{force[0]:6.1f}            {force[1]:6.1f}'))
            except Exception:
                pass
            viewer.sync()
            threading.Event().wait(1.0 / max(1.0, VIEW_HZ))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
