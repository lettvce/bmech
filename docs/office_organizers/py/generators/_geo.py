# Shared pure-geometry helpers used by multiple generators. No bpy imports.

import math


class VertPool:
    """Deduplicating vertex pool keyed on rounded coordinates - lets
    independently-built faces reference the exact same vertex index
    wherever their coordinates coincide, which is what keeps generated
    meshes watertight without an explicit merge-by-distance pass."""

    def __init__(self):
        self.verts = []
        self._index = {}

    def add(self, x, y, z):
        key = (round(x, 6), round(y, 6), round(z, 6))
        i = self._index.get(key)
        if i is None:
            i = len(self.verts)
            self._index[key] = i
            self.verts.append((x, y, z))
        return i


def grid_boundaries(total_mm, n, wall_mm, pocket_mm):
    """2n+2 boundary coords alternating wall/pocket/wall/.../wall (n
    pockets, n+1 walls), starting and ending exactly at 0 and total_mm."""
    xs = [0.0]
    x = 0.0
    for _ in range(n):
        x += wall_mm
        xs.append(x)
        x += pocket_mm
        xs.append(x)
    xs.append(total_mm)  # final outer wall, pinned to avoid float drift
    return xs


def variable_grid_boundaries(sizes_mm, wall_mm):
    """2n+2 boundary coords alternating wall/pocket/wall/.../wall, same
    layout as grid_boundaries but each of the n pockets has its OWN
    size instead of one uniform pocket_mm repeated n times. Starts at
    0; the total (unlike grid_boundaries) is derived from the sizes
    rather than an independently-supplied total_mm to pin against."""
    xs = [0.0]
    x = 0.0
    for size in sizes_mm:
        x += wall_mm
        xs.append(x)
        x += size
        xs.append(x)
    x += wall_mm
    xs.append(x)
    return xs


def face_z(pool, x0, x1, y0, y1, z, up):
    pts = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    if not up:
        pts.reverse()
    return tuple(pool.add(*p) for p in pts)


def face_x(pool, x, y0, y1, z0, z1, plus):
    pts = [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]
    if not plus:
        pts.reverse()
    return tuple(pool.add(*p) for p in pts)


def face_y(pool, y, x0, x1, z0, z1, plus):
    pts = [(x0, y, z0), (x0, y, z1), (x1, y, z1), (x1, y, z0)]
    if not plus:
        pts.reverse()
    return tuple(pool.add(*p) for p in pts)


def polygon_area(radius, angles):
    """Area of the polygon inscribed at `radius` with vertex `angles`
    (radians, CCW order). Handles non-uniform angle spacing, so it works
    for both regular N-gons and the irregular cell-to-circle boundaries
    used when a circle is mapped onto a rectangular cell's corners."""
    area = 0.0
    n = len(angles)
    for k in range(n):
        d = angles[(k + 1) % n] - angles[k]
        area += math.sin(d % (2.0 * math.pi))
    return 0.5 * radius * radius * area


def ramp(a, b, n):
    """n points from a to b, excluding b (so consecutive ramps
    concatenate into a ring without duplicate joints)."""
    return [a + (b - a) * i / n for i in range(n)]


def rect_perimeter(x0, x1, y0, y1, e_per_side):
    """CCW (viewed from +Z) boundary points of a rectangle, e_per_side
    points per edge, starting at (x0, y0) and NOT including the closing
    point - the ring wraps back to point 0 implicitly."""
    pts = []
    for e in range(e_per_side):
        pts.append((x0 + (x1 - x0) * e / e_per_side, y0))       # bottom, ->
    for e in range(e_per_side):
        pts.append((x1, y0 + (y1 - y0) * e / e_per_side))        # right, up
    for e in range(e_per_side):
        pts.append((x1 - (x1 - x0) * e / e_per_side, y1))        # top, <-
    for e in range(e_per_side):
        pts.append((x0, y1 - (y1 - y0) * e / e_per_side))        # left, down
    return pts


def clean_face(idxs):
    """Drop consecutive duplicate vertex indices from a face. Needed
    when a generated point exactly coincides with its neighbor - e.g. a
    circle inscribed with zero margin in its cell is exactly tangent at
    each edge's midpoint, so a boundary tick landing there degenerates
    what should be a quad into a triangle (one repeated corner). Returns
    None if fewer than 3 distinct vertices remain."""
    out = []
    for idx in idxs:
        if not out or out[-1] != idx:
            out.append(idx)
    if len(out) > 1 and out[0] == out[-1]:
        out.pop()
    if len(out) < 3:
        return None
    return tuple(out)


def extrude_profile_along_x(pool, outline_yz, x0, x1):
    """Extrude a closed 2D polygon (list of (y, z) points, CCW as
    normally drawn with y right / z up) into a prism from x0 to x1.

    NOT a simple axis-relabel of extrude_profile_along_y: swapping which
    two axes are "in-plane" is a reflection (determinant -1), so the
    sign conventions differ and were re-derived/verified directly
    (unit-square profile -> unit cube, checked for volume=+1 and zero
    open edges) rather than assumed from the Y-extrusion case."""
    near = [pool.add(x0, y, z) for (y, z) in outline_yz]
    far = [pool.add(x1, y, z) for (y, z) in outline_yz]
    faces = [tuple(reversed(near)), tuple(far)]
    n = len(outline_yz)
    for k in range(n):
        k1 = (k + 1) % n
        faces.append((near[k], near[k1], far[k1], far[k]))
    return faces


def extrude_profile_along_z(pool, outline_xy, z0, z1):
    """Extrude a closed 2D polygon (list of (x, y) points, CCW as
    normally drawn with x right / y up) into a prism from z0 to z1.

    Verified the same way as extrude_profile_along_x (unit-square
    profile -> unit cube, volume=+1, zero open edges) rather than
    assumed - a third axis is a third independent reflection case, not
    a free relabeling of either of the other two."""
    near = [pool.add(x, y, z0) for (x, y) in outline_xy]
    far = [pool.add(x, y, z1) for (x, y) in outline_xy]
    faces = [tuple(reversed(near)), tuple(far)]
    n = len(outline_xy)
    for k in range(n):
        k1 = (k + 1) % n
        faces.append((near[k], near[k1], far[k1], far[k]))
    return faces


def combine_pydata(*pieces):
    """Concatenate multiple (verts, edges, faces) pydata pieces into
    one, offsetting indices - no vertex welding. For independently-built
    solids meant to sit touching (not literally share topology), which
    prints fine as one fused part - same principle already used for the
    cable clip's hoops + base."""
    verts, edges, faces = [], [], []
    for pv, pe, pf in pieces:
        offset = len(verts)
        verts.extend(pv)
        edges.extend(tuple(i + offset for i in e) for e in pe)
        faces.extend(tuple(i + offset for i in f) for f in pf)
    return verts, edges, faces


def extrude_profile_along_y(pool, outline_xz, y0, y1):
    """Extrude a closed 2D polygon (list of (x, z) points, CCW as normally
    drawn with x right / z up) into a prism from y0 to y1. Returns a list
    of faces: the two end caps plus the side wall ring.

    The outline is used as-is for the y0 cap (normal -Y) and reversed for
    the y1 cap (normal +Y) - Newell's formula on a planar loop at constant
    Y depends only on the (x, z) winding, not on which Y it sits at, so
    the same two orderings work regardless of the profile's shape."""
    front = [pool.add(x, y0, z) for (x, z) in outline_xz]
    back = [pool.add(x, y1, z) for (x, z) in outline_xz]
    faces = [tuple(front), tuple(reversed(back))]
    n = len(outline_xz)
    for k in range(n):
        k1 = (k + 1) % n
        faces.append((front[k], back[k], back[k1], front[k1]))
    return faces
