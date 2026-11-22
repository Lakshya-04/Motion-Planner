"""visualization_msgs/Marker -> marker dict, duck-typed (no ROS import, testable with fakes)."""
_TYPES = {0: 'ARROW', 1: 'CUBE', 2: 'SPHERE', 3: 'CYLINDER', 4: 'LINE_STRIP', 5: 'LINE_LIST', 6: 'CUBE_LIST',
          7: 'SPHERE_LIST', 8: 'POINTS', 9: 'TEXT_VIEW_FACING', 10: 'MESH_RESOURCE', 11: 'TRIANGLE_LIST'}
_ACTIONS = {0: 'ADD', 1: 'ADD', 2: 'DELETE', 3: 'DELETEALL'}


def marker_to_dict(m):
    p, o, s, c = m.pose.position, m.pose.orientation, m.scale, m.color
    return {'type': _TYPES.get(int(m.type), 'UNKNOWN'), 'ns': m.ns, 'id': int(m.id),
            'action': _ACTIONS.get(int(m.action), 'ADD'), 'frame': m.header.frame_id,
            'pose': (p.x, p.y, p.z, o.x, o.y, o.z, o.w), 'scale': (s.x, s.y, s.z),
            'color': (c.r, c.g, c.b, c.a), 'points': [(q.x, q.y, q.z) for q in m.points], 'text': m.text}


def array_to_dicts(msg):
    return [marker_to_dict(m) for m in msg.markers]
