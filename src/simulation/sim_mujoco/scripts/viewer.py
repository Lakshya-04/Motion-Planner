#!/usr/bin/env python3
"""Optional MuJoCo viewer: shows the generated MJCF and follows /joint_states.

    viewer.py robot.xml joint1 joint2 ...

Needs a display and the `mujoco` Python package (4.x: mujoco-core + mujoco-studio). It mirrors the
joint states, it is not the simulation itself (the physics runs in the ros2_control plugin).
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


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
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            mujoco.mj_forward(model, data)
            viewer.sync()
            threading.Event().wait(0.03)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
