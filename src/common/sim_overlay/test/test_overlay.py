import math
import os
import sys
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sim_overlay import overlay as ov  # noqa: E402
from sim_overlay.marker_io import marker_to_dict  # noqa: E402


def mk(type_, ns='a', id_=0, action='ADD', **kw):
    d = {'type': type_, 'ns': ns, 'id': id_, 'action': action, 'pose': (0, 0, 0, 0, 0, 0, 1),
         'scale': (0.02, 0.02, 0.02), 'color': (1, 0, 0, 1), 'points': [], 'text': ''}
    d.update(kw)
    return d


def test_line_strip_identity():
    p = ov.convert_marker(mk('LINE_STRIP', points=[(0, 0, 0), (1, 0, 0), (1, 1, 0)], scale=(0.03, 0, 0)))
    assert p['kind'] == 'polyline' and p['points'][2] == [1, 1, 0] and p['width'] == 0.03
    assert p['color'] == [1, 0, 0, 1]


def test_single_point_strip_dropped():
    assert ov.convert_marker(mk('LINE_STRIP', points=[(0, 0, 0)])) is None


def test_frame_transform_yaw_and_offset():
    tf = ov.FrameTransform(1, 2, 3, math.pi / 2)
    p = ov.convert_marker(mk('LINE_STRIP', points=[(1, 0, 0), (2, 0, 0)]), tf)
    assert [round(c, 6) for c in p['points'][0]] == [1, 3, 3]
    assert [round(c, 6) for c in p['points'][1]] == [1, 4, 3]


def test_arrow_from_pose_and_scale():
    q = (0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # yaw 90 deg
    p = ov.convert_marker(mk('ARROW', pose=(1, 1, 0) + q, scale=(0.5, 0.06, 0.06)))
    assert [round(c, 6) for c in p['start']] == [1, 1, 0] and [round(c, 6) for c in p['end']] == [1, 1.5, 0]
    tf = ov.FrameTransform(yaw=math.pi / 2)
    p = ov.convert_marker(mk('ARROW', pose=(1, 1, 0) + q, scale=(0.5, 0.06, 0.06)), tf)
    assert [round(c, 6) for c in p['end']] == [-1.5, 1, 0]


def test_box_and_edges():
    p = ov.convert_marker(mk('CUBE', pose=(1, 0, 0, 0, 0, 0, 1), scale=(2, 1, 0.5)))
    assert p['kind'] == 'box' and p['center'] == [1, 0, 0]
    assert len(ov.box_edges(p['center'], p['size'], p['quat'])) == 12
    poly = ov.box_polyline(p['center'], p['size'], p['quat'])
    assert len(poly) == 16
    xs = [c[0] for c in poly]
    assert min(xs) == 0 and max(xs) == 2


def test_box_polyline_covers_all_edges():
    c = ov.box_corners((0, 0, 0), (1, 1, 1), (0, 0, 0, 1))
    poly = ov.box_polyline((0, 0, 0), (1, 1, 1), (0, 0, 0, 1))
    seen = {frozenset((c.index(a), c.index(b))) for a, b in zip(poly[:-1], poly[1:])}
    assert seen >= {frozenset(e) for e in ov._EDGES}


def test_label_and_unknown():
    p = ov.convert_marker(mk('TEXT_VIEW_FACING', pose=(1, 2, 0.4, 0, 0, 0, 1), scale=(0, 0, 0.15), text='S1'))
    assert p['kind'] == 'label' and p['text'] == 'S1' and p['height'] == 0.15
    assert ov.convert_marker(mk('MESH_RESOURCE')) is None


def test_arrow_polyline_shape():
    pts = ov.arrow_polyline([0, 0, 0], [1, 0, 0])
    assert len(pts) == 5 and pts[1] == pts[3] == [1, 0, 0] and pts[2][0] < 1 and pts[2][1] * pts[4][1] < 0


def test_array_semantics_and_state():
    s = ov.OverlayState()
    s.update('/t', [mk('LINE_STRIP', id_=1, points=[(0, 0, 0), (1, 0, 0)]), mk('LINE_STRIP', id_=2,
                                                                                points=[(0, 0, 0), (0, 1, 0)])])
    assert len(s.primitives()) == 2
    s.update('/t', [mk('LINE_STRIP', id_=1, action='DELETE')])
    assert len(s.primitives()) == 1
    s.update('/t', [mk('X', action='DELETEALL'), mk('LINE_STRIP', id_=9, points=[(0, 0, 0), (1, 1, 1)])])
    ps = s.primitives()
    assert len(ps) == 1 and ps[0]['id'] == 9 and ps[0]['key'].startswith('t:')


def test_marker_to_dict_duck():
    m = NS(type=4, ns='plan_columns', id=3, action=0, header=NS(frame_id='map'),
           pose=NS(position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)), scale=NS(x=0.025, y=0, z=0),
           color=NS(r=1, g=0, b=0, a=1), points=[NS(x=0, y=0, z=0), NS(x=1, y=0, z=0)], text='')
    d = marker_to_dict(m)
    assert d['type'] == 'LINE_STRIP' and d['points'][1] == (1, 0, 0) and d['frame'] == 'map'
    assert ov.convert_marker(d)['kind'] == 'polyline'
