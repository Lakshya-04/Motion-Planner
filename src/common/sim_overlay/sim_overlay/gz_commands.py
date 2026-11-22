"""gz-sim marker commands (Gazebo Harmonic: gz-sim 8, gz-transport 13). Pure python, builds strings.

Mechanism: the ``/marker`` service of the gz GUI MarkerManager plugin, request gz.msgs.Marker, reply gz.msgs.Boolean,
sent with ``gz service`` (``--reqtype/--reptype/--req``). Markers are identified by (ns, id): re-sending the same pair
modifies it (ADD_MODIFY); DELETE_ALL with an ns removes that namespace only.
"""
import shlex

SERVICE = '/marker'
NS_PREFIX = 'mp_overlay'


def _color(c):
    r, g, b, a = (float(x) for x in c)
    return '{r: %.3f, g: %.3f, b: %.3f, a: %.3f}' % (r, g, b, a)


def _material(c):
    return 'material: {ambient: %s, diffuse: %s, emissive: %s}' % (_color(c), _color(c), _color(c))


def _pt(p):
    return '{x: %.4f, y: %.4f, z: %.4f}' % tuple(float(v) for v in p[:3])


def _head(ns, i, typ):
    return 'ns: "%s/%s", id: %d, action: ADD_MODIFY, type: %s, layer: 0' % (NS_PREFIX, ns, i, typ)


def _strip(prim, pts):
    return '%s, %s, %s' % (_head(prim['ns'] or 'default', prim['id'], 'LINE_STRIP'), _material(prim['color']),
                           ', '.join('point: %s' % _pt(p) for p in pts))


def marker_requests(prim):
    """Primitive -> list of gz.msgs.Marker text requests (a box or arrow can need more than one id)."""
    from .overlay import arrow_polyline, box_polyline
    k = prim['kind']
    if k == 'polyline':
        pts = prim['points']
        if prim.get('list'):
            return [_strip(dict(prim, id=prim['id'] * 1000 + j), pts[j:j + 2]) for j in range(0, len(pts) - 1, 2)]
        return [_strip(prim, pts)]
    if k == 'arrow':
        return [_strip(prim, arrow_polyline(prim['start'], prim['end']))]
    if k == 'box':
        return [_strip(prim, box_polyline(prim['center'], prim['size'], prim['quat']))]
    if k == 'tiles':
        # one TRIANGLE_LIST for all quads, both windings (gz draws single-sided triangles)
        tri = []
        for q in prim['quads']:
            tri += [q[0], q[1], q[2], q[0], q[2], q[3], q[0], q[2], q[1], q[0], q[3], q[2]]
        return ['%s, %s, %s' % (_head(prim['ns'] or 'default', prim['id'], 'TRIANGLE_LIST'), _material(prim['color']),
                                ', '.join('point: %s' % _pt(p) for p in tri))]
    if k == 'sphere':
        return ['%s, %s, scale: {x: %.3f, y: %.3f, z: %.3f}, pose: {position: %s}' % (
            _head(prim['ns'] or 'default', prim['id'], 'SPHERE'), _material(prim['color']),
            prim['radius'] * 2, prim['radius'] * 2, prim['radius'] * 2, _pt(prim['center']))]
    if k == 'label':
        # gz.msgs.Marker has TEXT (no billboard type in Harmonic); scale.x is the character height.
        return ['%s, %s, text: "%s", scale: {x: %.3f, y: %.3f, z: %.3f}, pose: {position: %s}' % (
            _head(prim['ns'] or 'default', prim['id'], 'TEXT'), _material(prim['color']),
            prim['text'].replace('"', "'"), prim['height'], prim['height'], prim['height'], _pt(prim['position']))]
    return []


def clear_request(ns):
    return 'ns: "%s/%s", action: DELETE_ALL' % (NS_PREFIX, ns or 'default')


def gz_command(req, service=SERVICE, timeout_ms=1000):
    """argv list for ``gz service``; the exact text a user can paste is shlex.join of it."""
    return ['gz', 'service', '-s', service, '--reqtype', 'gz.msgs.Marker', '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms), '--req', req]


def plan_commands(prims, previous_ns=(), service=SERVICE):
    """Commands that make the scene equal ``prims``: clear every namespace used before or now, then add all.
    Returns (argv_lists, namespaces_now)."""
    ns_now = sorted({p['ns'] or 'default' for p in prims})
    cmds = [gz_command(clear_request(n), service) for n in sorted(set(previous_ns) | set(ns_now))]
    for p in prims:
        cmds += [gz_command(r, service) for r in marker_requests(p)]
    return cmds, ns_now


def shell(argv):
    return ' '.join(shlex.quote(a) for a in argv)
