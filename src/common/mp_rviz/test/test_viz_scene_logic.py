"""Host tests of the pure logic of mp_viz_scene.py (no rclpy)."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import mp_viz_scene_logic as L  # noqa: E402

PTS = [(float(i), 0.0, 0.0) for i in range(10)]


def test_segments_split_and_colour():
    segs = [{'label': 'strip0', 'start_index': 0, 'end_index': 3},
            {'label': 'connector1', 'start_index': 3, 'end_index': 4},
            {'label': 'strip1', 'start_index': 4, 'end_index': 8}]
    out = L.segment_polylines(segs, PTS)
    assert [o['connector'] for o in out] == [False, True, False]
    assert out[1]['rgb'] == L.CONNECTOR_RGB
    assert out[0]['rgb'] == L.palette(0) and out[2]['rgb'] == L.palette(1)
    assert len(L.segment_polylines(segs, PTS, include_connectors=False)) == 2


def test_render_indices_widen_connector_and_bad_ranges_skipped():
    segs = [{'label': 'connector', 'start_index': 4, 'end_index': 4, 'render_start_index': 3, 'render_end_index': 5},
            {'label': 'bad', 'start_index': 5, 'end_index': 99},
            {'label': 'single', 'start_index': 2, 'end_index': 2}]
    out = L.segment_polylines(segs, PTS)
    assert len(out) == 1 and len(out[0]['points']) == 3


def test_path_fallback():
    assert len(L.path_fallback(PTS)) == 1
    assert L.path_fallback(PTS[:1]) == []


def test_subsample_keeps_ends():
    s = L.subsample(list(range(100)), 5)
    assert len(s) == 5 and s[0] == 0 and s[-1] == 99
    assert L.subsample([1, 2], 5) == [1, 2]


def test_wall_normal_from_tool():
    # identity: tool z = world z, so the normal is -z
    assert L.wall_normal_from_tool((0, 0, 0, 1)) == pytest.approx((0, 0, -1))
    # rotate +90 deg about y: tool z -> world +x, normal -> -x
    h = math.sqrt(0.5)
    assert L.wall_normal_from_tool((0, h, 0, h)) == pytest.approx((-1, 0, 0), abs=1e-9)


def test_stand_specs_labels():
    s = L.stand_specs([{'x': 1, 'y': 2, 'yaw': math.pi, 'travel_s': 2.5, 'target_indices': [0, 1]},
                       {'x': 0, 'y': 0, 'target_indices': []}])
    assert s[0]['label'] == '#0  2 tgt  2.5s'
    assert s[1]['label'] == '#1  0 tgt'
    assert s[0]['quat'][2] == pytest.approx(1.0)


def test_footprint_and_circle():
    fp = L.footprint_corners(0, 0, 0, 2.0, 1.0)
    assert len(fp) == 5 and fp[0] == fp[-1]
    assert max(p[0] for p in fp) == pytest.approx(1.0)
    rot = L.footprint_corners(0, 0, math.pi / 2, 2.0, 1.0)
    assert max(p[1] for p in rot) == pytest.approx(1.0)
    c = L.circle_points(1, 1, 2.0)
    assert all(math.hypot(x - 1, y - 1) == pytest.approx(2.0) for x, y, _ in c)


def test_reach_rgb_ramp():
    assert L.reach_rgb(0, 1) == (0.0, 0.0, 1.0)
    assert L.reach_rgb(1, 1) == (1.0, 0.0, 0.0)
    assert L.reach_rgb(5, 0) == (0.0, 0.0, 1.0)


def test_wall_corners_and_area():
    w = {'origin_xyz': [2, 0, 2], 'basis_u': [0, 1, 0], 'basis_v': [0, 0, -1], 'u_range_m': [-1.5, 1.5],
         'v_range_m': [0, 2]}
    c = L.wall_corners(w)
    assert c[0] == (2, -1.5, 2) and c[2] == (2, 1.5, 0)
    assert len(L.wall_quad_triangles(w)) == 6 and L.wall_outline(w)[0] == L.wall_outline(w)[-1]
    assert L.wall_area_m2(w) == pytest.approx(6.0)
    assert L.plan_bbox_area_m2([(2, 0, 1), (2, 0.9, 1.3)], w) == pytest.approx(0.27)


def test_plan_strip_area_skips_connectors():
    lines = [{'connector': False, 'points': [(0, 0, 0), (1, 0, 0)]}, {'connector': True, 'points': [(1, 0, 0), (1, 5, 0)]}]
    assert L.plan_strip_area_m2(lines, 0.1) == pytest.approx(0.1)


def test_densify_poses():
    poses = [(0, 0, 0, 0, 0, 0, 1), (1, 0, 0, 0, 0, 0, 1)]
    d = L.densify_poses(poses, [(0, 1)], 0.25)
    assert len(d) == 5 and d[2][0] == pytest.approx(0.5) and d[-1][0] == 1


def test_world_to_arm_base():
    p = L.world_to_arm_base((3, 1, 1.5, 0, 0, 0, 1), 1, 1, math.pi / 2, 0.5)
    assert p[0] == pytest.approx(0, abs=1e-9) and p[1] == pytest.approx(-2) and p[2] == pytest.approx(1.0)
    assert abs(p[5]) == pytest.approx(math.sin(math.pi / 4))


def test_reach_summary():
    assert L.reach_summary([True, False]) == '1/2 reachable (50%)'
    assert L.reach_colour(True) != L.reach_colour(False)


def test_transitions_between_strips():
    pts = [(0, 0, 0), (0, 0, 1), (0, 0, 1), (0, 0, 0), (0, 1, 1), (0, 1, 0)]
    segs = [{'label': 's0p0', 'start_index': 0, 'end_index': 1}, {'label': 's0p1', 'start_index': 2, 'end_index': 3},
            {'label': 's1p0', 'start_index': 4, 'end_index': 5}]
    joins, nums = L.transitions(segs, pts)
    assert joins == [{'a': (0, 0, 0), 'b': (0, 1, 1)}]
    assert [n['n'] for n in nums] == [1, 2]


# ---- the drawn wall matches the world wall --------------------------------------------------------------------
GZ_SDF_WALL_X = 0.915   # Gazebo world wall front face: smoke scene aligned to standoff 0.915 (sim_gazebo wall_world)
MJ_GEOM = dict(pos=(1.03, 0.75, 1.3), quat=(0.5, -0.5, -0.5, 0.5), half=(0.7, 1.0, 0.05))   # mujoco-fix-nav3 robot.xml wall_0


def test_mujoco_wall_geom_front_face():
    w = L.wall_from_box(MJ_GEOM['pos'], MJ_GEOM['quat'], MJ_GEOM['half'])
    assert L.wall_plane_distance(w) == pytest.approx(0.98, abs=1e-9)          # 1.03 - 0.05
    assert w['normal_outward'] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)
    assert w['thickness_m'] == pytest.approx(0.1)
    cs = L.wall_corners(w)
    assert {round(c[0], 9) for c in cs} == {0.98}
    ys = sorted(c[1] for c in cs)
    zs = sorted(c[2] for c in cs)
    assert (ys[0], ys[-1]) == pytest.approx((0.05, 1.45)) and (zs[0], zs[-1]) == pytest.approx((0.3, 2.3))
    assert {round(c[0], 9) for c in L.wall_back_outline(w)} == {1.08}


def test_mjcf_file_wall(tmp_path):
    f = tmp_path / 'r.xml'
    f.write_text('<mujoco><worldbody><geom name="wall_0" type="box" pos="1.03 0.75 1.3" quat="0.5 -0.5 -0.5 0.5" '
                 'size="0.7 1 0.05"/></worldbody></mujoco>')
    w = L.wall_from_mjcf(str(f))
    assert L.wall_plane_distance(w) == pytest.approx(0.98)
    with pytest.raises(KeyError):
        L.wall_from_mjcf(str(f), 'nope')


def test_gazebo_world_wall_scene_matches_world_wall():
    """The wall mp_viz_scene draws from the launch's rviz_world_wall.json is the generated world's wall: front plane
    x = standoff (0.915), 3 x 2 m, floor to 2 m, 0.1 m thick behind the face; the old drawing (the scene file as is) was at x = 2.0."""
    import json
    sg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'simulation', 'sim_gazebo')
    sys.path.insert(0, sg)
    from sim_gazebo import wall_world as ww
    smoke = os.path.join(sg, 'test', 'data', 'smoke_wall_scene.json')
    shapes = ww.load_shapes(smoke, standoff=GZ_SDF_WALL_X)
    drawn = ww.world_wall_scene(shapes)['walls'][0]
    sdf_wall = next(s for s in shapes if s.role == 'wall')
    sdf_front_x = sdf_wall.position[0] - sdf_wall.dims[2] / 2.0   # the SDF box's front face (normal -x)
    assert L.wall_plane_distance(drawn) == pytest.approx(GZ_SDF_WALL_X, abs=1e-6)
    assert drawn['origin_xyz'][0] == pytest.approx(sdf_front_x, abs=1e-6)
    cs = L.wall_corners(drawn)
    assert {round(c[0], 6) for c in cs} == {GZ_SDF_WALL_X}
    assert (min(c[1] for c in cs), max(c[1] for c in cs)) == pytest.approx((-1.5, 1.5))
    assert (min(c[2] for c in cs), max(c[2] for c in cs)) == pytest.approx((0.0, 2.0))
    assert drawn['thickness_m'] == pytest.approx(ww.WALL_THICKNESS)
    with open(smoke) as f:
        old = L.wall_from_scene(json.load(f))
    assert abs(L.wall_plane_distance(old) - GZ_SDF_WALL_X) == pytest.approx(1.085)   # the mismatch that was drawn
