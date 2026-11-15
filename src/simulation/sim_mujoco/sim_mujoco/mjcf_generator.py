"""URDF -> MJCF for the MuJoCo simulation backend.

Pure Python (standard library only), so it runs in any image. The URDF comes from the robot
description xacro (see `render_urdf`); the profile YAML supplies the xacro arguments, the home pose
and the frame names.

What is generated
  * one MJCF body per URDF link, one joint per revolute / continuous / prismatic URDF joint, with the
    URDF origins, axes, limits and inertials carried over unchanged;
  * collision geometry only: primitives stay primitives, meshes are copied next to the MJCF and
    referenced as `<mesh>` assets. MuJoCo collides meshes as their convex hull (limited with
    `maxhullvert`), so no separate hull files are needed;
  * `<exclude>` pairs for links within `ignore_hops` revolute joints of each other (the same rule the
    other collision checkers in this repo use), so overlapping neighbouring meshes do not fight;
  * `<equality><joint>` constraints for URDF `<mimic>` joints (the telescopic lift stages);
  * one position actuator per commanded joint. kp / kv come from the joint effort limit, the force
    range is the effort limit; the plugin drives them with position + velocity feed-forward;
  * a force and a torque sensor on the site `ft_sensor` (at the wrist, the origin of the tool mount, with
    the tool, its weight and inertia beyond it) and a site `tcp_site`,
    plus a thin contact pad at the TCP so a wall touch happens where coverage poses are defined;
  * a `home` keyframe from the profile;
  * optionally (`base_motion`, off by default so the MJCF is unchanged) a planar base: three joints
    `base_x_joint`, `base_y_joint` (slide) and `base_yaw_joint` (hinge) on the root body, each with a
    stiff position actuator the plugin drives from /cmd_vel, plus `<custom>` numerics carrying the drive
    type and the velocity limits. The wheels stay visual.
"""
import math
import pathlib
import shutil
import xml.etree.ElementTree as ET

ACTUATED = ('revolute', 'continuous', 'prismatic')


# ------------------------------------------------------------------ small math helpers
def _floats(text, default='0 0 0'):
    return [float(v) for v in (text if text is not None else default).split()]


def quat_from_rpy(r, p, y):
    """URDF rpy (extrinsic x, y, z = Rz Ry Rx) -> unit quaternion (w, x, y, z)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def _rpy_matrix(rpy):
    w, x, y, z = quat_from_rpy(*rpy)
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _rotated_inertia(inertia, rpy):
    """Inertia (ixx iyy izz ixy ixz iyz) given in a frame rotated by rpy, expressed in the parent."""
    w, x, y, z = quat_from_rpy(*rpy)
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    m = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
    rm = [[sum(r[i][k] * m[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    out = [[sum(rm[i][k] * r[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return [out[0][0], out[1][1], out[2][2], out[0][1], out[0][2], out[1][2]]


def _fmt(values):
    return ' '.join(f'{v:.9g}' for v in values)


# ------------------------------------------------------------------ URDF model
class UrdfLink:
    def __init__(self, elem):
        self.name = elem.get('name')
        self.collisions = []
        inertial = elem.find('inertial')
        self.inertial = None
        if inertial is not None:
            org = inertial.find('origin')
            i = inertial.find('inertia')
            self.inertial = {
                'mass': float(inertial.find('mass').get('value')),
                'pos': _floats(org.get('xyz') if org is not None else None),
                'rpy': _floats(org.get('rpy') if org is not None else None),
                'inertia': [float(i.get(k, 0.0)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')]
                if i is not None else [0.0] * 6,
            }
        for col in elem.findall('collision'):
            org = col.find('origin')
            geo = col.find('geometry')[0]
            self.collisions.append({
                'pos': _floats(org.get('xyz') if org is not None else None),
                'rpy': _floats(org.get('rpy') if org is not None else None),
                'kind': geo.tag, 'attrib': dict(geo.attrib)})


class UrdfJoint:
    def __init__(self, elem):
        self.name = elem.get('name')
        self.type = elem.get('type')
        self.parent = elem.find('parent').get('link')
        self.child = elem.find('child').get('link')
        org = elem.find('origin')
        self.pos = _floats(org.get('xyz') if org is not None else None)
        self.rpy = _floats(org.get('rpy') if org is not None else None)
        axis = elem.find('axis')
        self.axis = _floats(axis.get('xyz') if axis is not None else None, '1 0 0')
        lim = elem.find('limit')
        self.lower = float(lim.get('lower', 0.0)) if lim is not None else None
        self.upper = float(lim.get('upper', 0.0)) if lim is not None else None
        self.effort = float(lim.get('effort', 0.0)) if lim is not None else 0.0
        if self.type == 'continuous':
            self.lower = self.upper = None
        dyn = elem.find('dynamics')
        self.damping = float(dyn.get('damping', 0.0)) if dyn is not None else 0.0
        self.friction = float(dyn.get('friction', 0.0)) if dyn is not None else 0.0
        mimic = elem.find('mimic')
        self.mimic = None if mimic is None else (
            mimic.get('joint'), float(mimic.get('multiplier', 1.0)), float(mimic.get('offset', 0.0)))


class UrdfRobot:
    def __init__(self, urdf_text):
        root = ET.fromstring(urdf_text)
        self.name = root.get('name', 'robot')
        self.links = {l.get('name'): UrdfLink(l) for l in root.findall('link')}
        self.joints = [UrdfJoint(j) for j in root.findall('joint')]
        self.child_joints = {}
        children = set()
        for j in self.joints:
            self.child_joints.setdefault(j.parent, []).append(j)
            children.add(j.child)
        roots = [n for n in self.links if n not in children]
        if len(roots) != 1:
            raise ValueError(f'URDF must have exactly one root link, found {roots}')
        self.root = roots[0]
        self.joint_by_child = {j.child: j for j in self.joints}

    def group_of(self, link):
        """Representative link of the rigid group `link` belongs to (fixed joints merge links)."""
        while link in self.joint_by_child and self.joint_by_child[link].type == 'fixed':
            link = self.joint_by_child[link].parent
        return link

    def group_parent(self, group):
        j = self.joint_by_child.get(group)
        return None if j is None else self.group_of(j.parent)

    def tree_distance(self, a, b):
        def chain(g):
            out = [g]
            while self.group_parent(out[-1]) is not None:
                out.append(self.group_parent(out[-1]))
            return out
        ca, cb = chain(a), chain(b)
        common = next(x for x in ca if x in cb)
        return ca.index(common) + cb.index(common)

    def ignored_pairs(self, links, hops):
        """Pairs among `links` that may not collide: same rigid group or <= hops joints apart."""
        out = []
        links = sorted(links)
        for i, a in enumerate(links):
            for b in links[i + 1:]:
                ga, gb = self.group_of(a), self.group_of(b)
                if ga == gb or self.tree_distance(ga, gb) <= hops:
                    out.append((a, b))
        return out


# ------------------------------------------------------------------ mesh handling
def resolve_mesh_path(uri, urdf_dir):
    """package://pkg/rel -> file path (ament index when available), else relative to the URDF."""
    if uri.startswith('package://'):
        pkg, _, rel = uri[len('package://'):].partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
        except ImportError as exc:
            raise FileNotFoundError(
                f'{uri}: no ament index here; resolve package URIs first '
                f'(docker/tools/resolve_package_uris.py)') from exc
        return pathlib.Path(get_package_share_directory(pkg)) / rel
    if uri.startswith('file://'):
        return pathlib.Path(uri[len('file://'):])
    return pathlib.Path(urdf_dir) / uri


# ------------------------------------------------------------------ generator
DEFAULTS = dict(
    timestep=0.002,
    ignore_hops=2,
    maxhullvert=64,
    gravcomp=True,
    revolute_kp_per_effort=10.0,     # N m/rad per N m of effort limit
    prismatic_kp_per_effort=100.0,   # N/m per N of effort limit
    kv_over_kp=0.1,                  # seconds
    revolute_damping=2.0,
    prismatic_damping=100.0,
    armature_per_effort=0.0005,
    min_armature=0.01,
    prismatic_armature=2.0,
    tool_pad_radius=0.03,
    tool_pad_thickness=0.01,
    friction=(0.8, 0.005, 0.0001),
    base_motion=None,                # None, or dict(drive='omni'|'diff', max_linear=, max_angular=)
    base_kp=(2.0e4, 2.0e4, 1.0e4),   # N/m, N/m, N m/rad: stiff hold, so wall contact barely moves it
    base_kv_over_kp=0.1,             # seconds
    base_force_limit=(3000.0, 3000.0, 1500.0),
    # None, or dict(pos=(x, y, z), rpy=(r, p, y) in the root link frame; optional width, height, hfov_deg,
    # vfov_deg, min_range, max_range, rate_hz, noise_std): a ray-cast depth sensor the plugin publishes
    depth_sensor=None,
)

DEPTH_DEFAULTS = dict(width=96, height=64, hfov_deg=60.0, vfov_deg=45.0, min_range=0.1, max_range=5.0,
                      rate_hz=5.0, noise_std=0.0)

BASE_JOINTS = ('base_x_joint', 'base_y_joint', 'base_yaw_joint')


def urdf_to_mjcf(urdf_text, out_dir, urdf_dir='.', home=None, ft_link='tool_mount_link',
                 tcp_link='tcp_link', commanded=None, world_geoms=(), **options):
    """Convert URDF text to an MJCF file `out_dir/robot.xml` (meshes copied to `out_dir/meshes`).

    home       dict joint -> position for the `home` keyframe (mimic joints are derived)
    commanded  joint names that get an actuator, in order (default: all actuated, non-mimic joints)
    world_geoms  extra `<geom>` XML elements for the worldbody (see wall_world.py)
    Returns the path of the written file.
    """
    opt = dict(DEFAULTS)
    unknown = set(options) - set(opt)
    if unknown:
        raise TypeError(f'unknown options {sorted(unknown)}')
    opt.update(options)
    robot = UrdfRobot(urdf_text)
    out = pathlib.Path(out_dir)
    (out / 'meshes').mkdir(parents=True, exist_ok=True)
    home = dict(home or {})

    mj = ET.Element('mujoco', model=robot.name)
    ET.SubElement(mj, 'compiler', angle='radian', meshdir='meshes', autolimits='true',
                  boundmass='0.001', boundinertia='0.000001', balanceinertia='true')
    ET.SubElement(mj, 'option', timestep=str(opt['timestep']), integrator='implicitfast',
                  gravity='0 0 -9.81')
    default = ET.SubElement(mj, 'default')
    ET.SubElement(default, 'geom', contype='1', conaffinity='1', friction=_fmt(opt['friction']))
    asset = ET.SubElement(mj, 'asset')
    world = ET.SubElement(mj, 'worldbody')
    ET.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1', diffuse='0.8 0.8 0.8')

    mesh_assets = {}
    colliding_links = []
    tree_joints = []  # actuated joints in MuJoCo's qpos order (depth-first over the body tree)

    def add_geoms(body, link):
        for k, c in enumerate(link.collisions):
            a = c['attrib']
            common = dict(pos=_fmt(c['pos']), quat=_fmt(quat_from_rpy(*c['rpy'])),
                          name=f'{link.name}_col{k}')
            kind = c['kind']
            if kind == 'box':
                geom = ET.SubElement(body, 'geom', type='box',
                                     size=_fmt([v / 2 for v in _floats(a['size'])]), **common)
            elif kind == 'cylinder':
                geom = ET.SubElement(body, 'geom', type='cylinder',
                                     size=_fmt([float(a['radius']), float(a['length']) / 2]), **common)
            elif kind == 'sphere':
                geom = ET.SubElement(body, 'geom', type='sphere', size=_fmt([float(a['radius'])]),
                                     **common)
            elif kind == 'mesh':
                src = resolve_mesh_path(a['filename'], urdf_dir)
                if src.suffix.lower() not in ('.stl', '.obj'):
                    raise ValueError(f'{src}: MuJoCo reads STL/OBJ collision meshes only')
                scale = _fmt(_floats(a.get('scale'), '1 1 1'))
                key = (str(src), scale)
                if key not in mesh_assets:
                    mesh_assets[key] = f'mesh_{len(mesh_assets)}_{src.stem}'
                    dst = out / 'meshes' / f'{mesh_assets[key]}{src.suffix.lower()}'
                    shutil.copy2(src, dst)
                    ET.SubElement(asset, 'mesh', name=mesh_assets[key], file=dst.name, scale=scale,
                                  maxhullvert=str(opt['maxhullvert']))
                geom = ET.SubElement(body, 'geom', type='mesh', mesh=mesh_assets[key], **common)
            else:
                raise ValueError(f'unsupported collision geometry <{kind}> in link {link.name}')
        if link.collisions:
            colliding_links.append(link.name)

    def add_body(parent_elem, link_name, joint, in_tool=False):
        in_tool = in_tool or link_name == ft_link
        link = robot.links[link_name]
        body = ET.SubElement(parent_elem, 'body', name=link_name)
        if joint is not None:
            body.set('pos', _fmt(joint.pos))
            body.set('quat', _fmt(quat_from_rpy(*joint.rpy)))
        if opt['gravcomp'] and not in_tool:
            # Everything beyond the F/T site keeps its real weight so the sensor reads it.
            body.set('gravcomp', '1')
        if link.inertial and link.inertial['mass'] > 0:
            i = link.inertial
            ET.SubElement(body, 'inertial', pos=_fmt(i['pos']), mass=str(i['mass']),
                          fullinertia=_fmt(_rotated_inertia(i['inertia'], i['rpy'])))
        if joint is None and opt['depth_sensor'] is not None:
            ds = opt['depth_sensor']
            ET.SubElement(body, 'site', name='depth_sensor', size='0.01', group='4',
                          pos=_fmt(ds.get('pos', (0, 0, 0))),
                          quat=_fmt(quat_from_rpy(*ds.get('rpy', (0, 0, 0)))))
        if joint is None and opt['base_motion'] is not None:
            for name, kind, axis in zip(BASE_JOINTS, ('slide', 'slide', 'hinge'),
                                        ('1 0 0', '0 1 0', '0 0 1')):
                ET.SubElement(body, 'joint', name=name, type=kind, axis=axis, damping='1')
        if joint is not None and joint.type in ACTUATED:
            attrs = dict(name=joint.name, type='slide' if joint.type == 'prismatic' else 'hinge',
                         axis=_fmt(joint.axis))
            prismatic = joint.type == 'prismatic'
            if joint.lower is not None:
                attrs['range'] = _fmt([joint.lower, joint.upper])
                attrs['limited'] = 'true'
            attrs['damping'] = str(joint.damping or (opt['prismatic_damping'] if prismatic
                                                      else opt['revolute_damping']))
            attrs['armature'] = str(opt['prismatic_armature'] if prismatic else
                                    max(opt['min_armature'], opt['armature_per_effort'] * joint.effort))
            if joint.friction:
                attrs['frictionloss'] = str(joint.friction)
            ET.SubElement(body, 'joint', **attrs)
            tree_joints.append(joint)
        add_geoms(body, link)
        if link_name == ft_link:
            ET.SubElement(body, 'site', name='ft_sensor', size='0.01', group='4')
            if opt['tool_pad_radius'] > 0:
                # The TCP frame sits ahead of the tool body; a thin pad there makes the wall touch
                # happen at the point coverage poses target.
                tcp = robot.joint_by_child.get(tcp_link)
                if tcp is not None and tcp.type == 'fixed' and tcp.parent == ft_link:
                    t = opt['tool_pad_thickness']
                    # half a thickness back along the TCP's own z axis (the TCP may be tilted)
                    rm = _rpy_matrix(tcp.rpy)
                    pad_pos = [tcp.pos[i] - rm[i][2] * t / 2 for i in range(3)]
                    ET.SubElement(body, 'geom', name='tool_pad', type='cylinder', pos=_fmt(pad_pos),
                                  size=_fmt([opt['tool_pad_radius'], t / 2]),
                                  quat=_fmt(quat_from_rpy(*tcp.rpy)))
        if link_name == tcp_link:
            ET.SubElement(body, 'site', name='tcp_site', size='0.01', group='4')
        for child in robot.child_joints.get(link_name, []):
            add_body(body, child.child, child, in_tool)

    add_body(world, robot.root, None)
    for g in world_geoms:
        if opt['depth_sensor'] is not None:
            g.set('group', '1')  # the depth sensor scans group 1 only: the scene, not the robot
        world.append(g)

    # contacts between nearby links are meaningless for meshes that overlap at joints
    contact = ET.SubElement(mj, 'contact')
    for a, b in robot.ignored_pairs(colliding_links, opt['ignore_hops']):
        ET.SubElement(contact, 'exclude', body1=a, body2=b)

    mimics = [j for j in robot.joints if j.mimic and j.type in ACTUATED]
    if mimics:
        eq = ET.SubElement(mj, 'equality')
        for j in mimics:
            ET.SubElement(eq, 'joint', joint1=j.name, joint2=j.mimic[0],
                          polycoef=_fmt([j.mimic[2], j.mimic[1], 0, 0, 0]))

    mimic_names = {j.name for j in mimics}
    joint_by_name = {j.name: j for j in robot.joints}
    if commanded is None:
        commanded = [j.name for j in robot.joints if j.type in ACTUATED and j.name not in mimic_names]
    actuators = ET.SubElement(mj, 'actuator')
    for name in commanded:
        j = joint_by_name[name]
        prismatic = j.type == 'prismatic'
        effort = j.effort if j.effort > 0 else (1000.0 if prismatic else 50.0)
        kp = effort * (opt['prismatic_kp_per_effort'] if prismatic else opt['revolute_kp_per_effort'])
        ET.SubElement(actuators, 'position', name=f'act_{name}', joint=name, kp=_fmt([kp]),
                      kv=_fmt([kp * opt['kv_over_kp']]), forcerange=_fmt([-effort, effort]))

    if opt['base_motion'] is not None:
        bm = opt['base_motion']
        drive = bm.get('drive', 'omni')
        if drive not in ('omni', 'diff'):
            raise ValueError(f"base_motion drive must be 'omni' or 'diff', got {drive!r}")
        for name, kp, lim in zip(BASE_JOINTS, opt['base_kp'], opt['base_force_limit']):
            ET.SubElement(actuators, 'position', name=f'act_{name}', joint=name, kp=_fmt([kp]),
                          kv=_fmt([kp * opt['base_kv_over_kp']]), forcerange=_fmt([-lim, lim]))
        custom = ET.SubElement(mj, 'custom')
        ET.SubElement(custom, 'numeric', name='sim_mujoco_base_diff',
                      data='1' if drive == 'diff' else '0')
        ET.SubElement(custom, 'numeric', name='sim_mujoco_base_max_vel',
                      data=_fmt([bm.get('max_linear', 0.5), bm.get('max_angular', 1.0)]))

    if opt['depth_sensor'] is not None:
        d = dict(DEPTH_DEFAULTS, **opt['depth_sensor'])
        custom = mj.find('custom')
        if custom is None:
            custom = ET.SubElement(mj, 'custom')
        ET.SubElement(custom, 'numeric', name='sim_mujoco_depth', data=_fmt([
            d['width'], d['height'], math.radians(d['hfov_deg']), math.radians(d['vfov_deg']),
            d['min_range'], d['max_range'], d['rate_hz'], d['noise_std']]))

    sensors = ET.SubElement(mj, 'sensor')
    ET.SubElement(sensors, 'force', name='ft_force', site='ft_sensor')
    ET.SubElement(sensors, 'torque', name='ft_torque', site='ft_sensor')

    if home:
        qpos = [0.0] * 3 if opt['base_motion'] is not None else []
        for j in tree_joints:
            if j.mimic:
                base = home.get(j.mimic[0], 0.0)
                qpos.append(j.mimic[2] + j.mimic[1] * base)
            else:
                qpos.append(home.get(j.name, 0.0))
        keyframe = ET.SubElement(mj, 'keyframe')
        ET.SubElement(keyframe, 'key', name='home', qpos=_fmt(qpos))

    ET.indent(mj)
    path = out / 'robot.xml'
    ET.ElementTree(mj).write(path, encoding='utf-8', xml_declaration=False)
    return path
