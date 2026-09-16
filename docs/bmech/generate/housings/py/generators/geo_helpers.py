# geo_helpers.py - standalone pure-Python geometry DSL for mesh generation.
#
# Abstracts vertex/edge/face arithmetic into high-level CAD operations:
# Loops (named rings/curves) flow through composable operations (bridge,
# loft, lathe, extrude_path, ...) and materialize as Mesh objects whose
# .to_pydata() feeds mesh.from_pydata() directly.  No bpy imports, no
# third-party deps - the same code runs in unit tests and inside Blender.
#
# Conventions:
#   * Units are mm.  Planar loops default to the XY plane, wound CCW as
#     seen from +Z ("normal" +Z by the right-hand rule).
#   * bridge(bottom, top) emits outward-facing quads when both rings are
#     CCW and top sits along the bottom ring's normal.
#   * cap(loop) fans in the loop's own winding: a CCW-from-+Z loop caps
#     facing +Z.  Cap a solid's underside with cap(loop.reversed()) or
#     cap(loop, reverse=True).
#   * Mesh + Mesh welds coincident vertices (1e-6 mm), so composing
#     walls + caps from shared rings yields a watertight solid without a
#     separate merge pass.

import math
import bisect

_WELD_DIGITS = 6          # rounding for automatic vertex welding (mm)
_EPS = 1e-9

# FDM guardrails (Bambu/Prusa class printers)
MIN_WALL_MM = 1.2
MIN_FILLET_MM = 0.8


# ---------------------------------------------------------------------------
# small vector helpers (module-private)
# ---------------------------------------------------------------------------

def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _length(a):
    return math.sqrt(_dot(a, a))


def _normalize(a):
    l = _length(a)
    if l < _EPS:
        raise ValueError("cannot normalize zero-length vector")
    return (a[0] / l, a[1] / l, a[2] / l)


def _basis_from_normal(n):
    """Right-handed orthonormal basis (u, v, w) with w = normalize(n).
    For n = +Z this returns exactly ((1,0,0), (0,1,0), (0,0,1)) so
    XY-plane geometry is unchanged."""
    w = _normalize(n)
    a = (1.0, 0.0, 0.0) if abs(w[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_sub(a, _mul(w, _dot(a, w))))   # Gram-Schmidt
    v = _cross(w, u)
    return u, v, w


def _rotate_about(p, axis_unit, angle_rad, center=(0.0, 0.0, 0.0)):
    """Rodrigues rotation of point p about a unit axis through center."""
    q = _sub(p, center)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    k = axis_unit
    term1 = _mul(q, c)
    term2 = _mul(_cross(k, q), s)
    term3 = _mul(k, _dot(k, q) * (1.0 - c))
    return _add(center, _add(term1, _add(term2, term3)))


def _axis_vector(axis):
    """Accept "x"/"y"/"z" or a 3-tuple; return a unit vector."""
    if isinstance(axis, str):
        table = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0),
                 "z": (0.0, 0.0, 1.0)}
        key = axis.lower()
        if key not in table:
            raise ValueError("axis must be 'x', 'y', 'z' or a 3-tuple")
        return table[key]
    return _normalize(tuple(float(c) for c in axis))


def _loop_plane_normal(verts):
    """Newell's method: unit normal of a (near-)planar vertex ring,
    aligned with the winding by the right-hand rule."""
    nx = ny = nz = 0.0
    n = len(verts)
    for k in range(n):
        x0, y0, z0 = verts[k]
        x1, y1, z1 = verts[(k + 1) % n]
        nx += (y0 - y1) * (z0 + z1)
        ny += (z0 - z1) * (x0 + x1)
        nz += (x0 - x1) * (y0 + y1)
    l = math.sqrt(nx * nx + ny * ny + nz * nz)
    if l < _EPS:
        raise ValueError("loop has no well-defined plane normal "
                         "(degenerate or collinear vertices)")
    return (nx / l, ny / l, nz / l)


def _check_positive(**kwargs):
    for name, value in kwargs.items():
        if not (value > 0.0):
            raise ValueError("%s must be positive (got %r)" % (name, value))


# ---------------------------------------------------------------------------
# core data structures
# ---------------------------------------------------------------------------

class Loop:
    """An ordered ring or curve of vertices - the composable unit.

    verts:     list of (x, y, z) tuples.
    is_closed: True when the last vert connects back to the first.
    name:      human-readable identifier ("bottom_ring", "profile", ...).
    """

    def __init__(self, verts, is_closed=True, name=""):
        self.verts = [tuple(float(c) for c in v) for v in verts]
        self.is_closed = bool(is_closed)
        self.name = name

    def length(self):
        """Number of vertices in this loop."""
        return len(self.verts)

    def as_indices(self, vert_index_offset=0):
        """Indices of this loop's verts relative to an offset."""
        return list(range(vert_index_offset,
                          vert_index_offset + len(self.verts)))

    def reversed(self):
        """Copy with reversed vertex order (flips the winding normal)."""
        return Loop(list(reversed(self.verts)), self.is_closed, self.name)

    def copy(self, name=None):
        return Loop(self.verts, self.is_closed,
                    self.name if name is None else name)

    def plane_normal(self):
        """Unit normal of the loop's plane, aligned with its winding."""
        return _loop_plane_normal(self.verts)

    def __len__(self):
        return len(self.verts)

    def __repr__(self):
        return "Loop(%r, %d verts, %s)" % (
            self.name, len(self.verts),
            "closed" if self.is_closed else "open")


class ValidationReport:
    """Result of Mesh.validate().  is_valid means no hard errors;
    warnings flag soft issues (open boundaries, thin features)."""

    def __init__(self, errors=None, warnings=None, volume=None):
        self.errors = errors or []
        self.warnings = warnings or []
        self.volume = volume

    @property
    def is_valid(self):
        return not self.errors

    def __bool__(self):
        return self.is_valid

    def __repr__(self):
        return ("ValidationReport(valid=%s, errors=%r, warnings=%r)"
                % (self.is_valid, self.errors, self.warnings))


class Mesh:
    """Container for verts, edges, faces.  to_pydata() feeds
    mesh.from_pydata() directly.  mesh_a + mesh_b concatenates with
    automatic index offsetting AND welds coincident vertices, so parts
    built from shared rings fuse watertight."""

    def __init__(self, verts=None, edges=None, faces=None):
        self.verts = list(verts) if verts else []
        self.edges = [tuple(e) for e in edges] if edges else []
        self.faces = [tuple(f) for f in faces] if faces else []

    def to_pydata(self):
        """Return (verts, edges, faces) for mesh.from_pydata()."""
        return (self.verts, self.edges, self.faces)

    def __add__(self, other):
        return append_mesh(self, other)

    def bounds(self):
        """(min_x, max_x, min_y, max_y, min_z, max_z)."""
        if not self.verts:
            raise ValueError("bounds() on empty mesh")
        xs = [v[0] for v in self.verts]
        ys = [v[1] for v in self.verts]
        zs = [v[2] for v in self.verts]
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    def signed_volume(self):
        """Signed volume via divergence theorem (positive = outward
        winding).  Faces fan-triangulated from their first vertex."""
        vol = 0.0
        for f in self.faces:
            x0, y0, z0 = self.verts[f[0]]
            for k in range(1, len(f) - 1):
                x1, y1, z1 = self.verts[f[k]]
                x2, y2, z2 = self.verts[f[k + 1]]
                vol += (x0 * (y1 * z2 - y2 * z1)
                        - y0 * (x1 * z2 - x2 * z1)
                        + z0 * (x1 * y2 - x2 * y1))
        return vol / 6.0

    def validate(self, require_closed=False):
        """DFM/structural audit.  Errors (block export): non-finite
        coords, bad indices, degenerate faces, inconsistent winding,
        inverted closed solids.  Warnings: boundary edges (open mesh),
        near-duplicate verts, thin overall bounding dimension.
        require_closed=True promotes boundary edges to errors."""
        errors = []
        warnings = []

        for vi, v in enumerate(self.verts):
            if len(v) != 3:
                errors.append("vert %d is not a 3-tuple" % vi)
            elif not all(math.isfinite(c) for c in v):
                errors.append("vert %d has non-finite coordinate" % vi)
        nv = len(self.verts)
        for fi, f in enumerate(self.faces):
            if len(f) < 3:
                errors.append("face %d has fewer than 3 vertices" % fi)
                continue
            if len(set(f)) != len(f):
                errors.append("face %d repeats a vertex index" % fi)
            for idx in f:
                if not (0 <= idx < nv):
                    errors.append("face %d index %d out of bounds"
                                  % (fi, idx))
                    break
        if errors:
            return ValidationReport(errors, warnings)

        # near-duplicate verts (should have been welded)
        seen = {}
        dup = 0
        for vi, v in enumerate(self.verts):
            key = (round(v[0], _WELD_DIGITS), round(v[1], _WELD_DIGITS),
                   round(v[2], _WELD_DIGITS))
            if key in seen:
                dup += 1
            else:
                seen[key] = vi
        if dup:
            warnings.append("%d near-duplicate vertices (consider "
                            "weld_duplicates)" % dup)

        # winding consistency + closedness via directed edges
        directed = {}
        winding_bad = False
        for fi, f in enumerate(self.faces):
            for k in range(len(f)):
                e = (f[k], f[(k + 1) % len(f)])
                if e in directed:
                    errors.append("directed edge %s used twice (faces %d,"
                                  " %d) - inconsistent winding or "
                                  "duplicate face" % (e, directed[e], fi))
                    winding_bad = True
                    break
                directed[e] = fi
            if winding_bad:
                break
        boundary = 0
        if not winding_bad:
            for (a, b) in directed:
                if (b, a) not in directed:
                    boundary += 1
            if boundary:
                msg = ("%d boundary edge(s) - mesh is open (not "
                       "watertight)" % boundary)
                if require_closed:
                    errors.append(msg)
                else:
                    warnings.append(msg)

        volume = None
        if not errors and not boundary:
            volume = self.signed_volume()
            if volume <= 0.0:
                errors.append("signed volume %.4f <= 0 - normals are "
                              "inverted (use flip_normals)" % volume)

        # rough thin-feature estimate: any nonzero bbox dimension under
        # the FDM minimum wall
        if self.verts:
            b = self.bounds()
            for lo, hi, name in ((b[0], b[1], "X"), (b[2], b[3], "Y"),
                                 (b[4], b[5], "Z")):
                d = hi - lo
                if _EPS < d < MIN_WALL_MM:
                    warnings.append("overall %s dimension %.2fmm is under "
                                    "the %.1fmm FDM minimum wall"
                                    % (name, d, MIN_WALL_MM))

        return ValidationReport(errors, warnings, volume)

    def __repr__(self):
        return "Mesh(%d verts, %d faces)" % (len(self.verts),
                                             len(self.faces))


class _VertPool:
    """Deduplicating vertex pool keyed on rounded coordinates - keeps
    composed meshes watertight without an explicit merge pass."""

    def __init__(self):
        self.verts = []
        self._index = {}

    def add(self, v):
        key = (round(v[0], _WELD_DIGITS), round(v[1], _WELD_DIGITS),
               round(v[2], _WELD_DIGITS))
        i = self._index.get(key)
        if i is None:
            i = len(self.verts)
            self._index[key] = i
            self.verts.append(tuple(v))
        return i


# ---------------------------------------------------------------------------
# 3.1 primitives
# ---------------------------------------------------------------------------

def circle(radius, segments=32, center=(0, 0, 0), normal=(0, 0, 1)):
    """Planar circle (closed loop), CCW around `normal`."""
    _check_positive(radius=radius)
    if segments < 3:
        raise ValueError("circle needs segments >= 3")
    u, v, _ = _basis_from_normal(normal)
    verts = []
    for k in range(segments):
        a = 2.0 * math.pi * k / segments
        p = _add(center, _add(_mul(u, radius * math.cos(a)),
                              _mul(v, radius * math.sin(a))))
        verts.append(p)
    return Loop(verts, True, "circle")


def arc(radius, angle_deg, segments=16, center=(0, 0, 0), normal=(0, 0, 1)):
    """Circular arc (open loop) from angle 0 to angle_deg, CCW."""
    _check_positive(radius=radius)
    if not (0.0 < angle_deg <= 360.0):
        raise ValueError("arc angle_deg must be in (0, 360]")
    if segments < 1:
        raise ValueError("arc needs segments >= 1")
    u, v, _ = _basis_from_normal(normal)
    verts = []
    for k in range(segments + 1):
        a = math.radians(angle_deg) * k / segments
        p = _add(center, _add(_mul(u, radius * math.cos(a)),
                              _mul(v, radius * math.sin(a))))
        verts.append(p)
    return Loop(verts, False, "arc")


def ellipse(rx, ry, segments=32, center=(0, 0, 0), normal=(0, 0, 1)):
    """Planar ellipse (closed loop)."""
    _check_positive(rx=rx, ry=ry)
    if segments < 3:
        raise ValueError("ellipse needs segments >= 3")
    u, v, _ = _basis_from_normal(normal)
    verts = []
    for k in range(segments):
        a = 2.0 * math.pi * k / segments
        p = _add(center, _add(_mul(u, rx * math.cos(a)),
                              _mul(v, ry * math.sin(a))))
        verts.append(p)
    return Loop(verts, True, "ellipse")


def rectangle(width, height, center=(0, 0, 0)):
    """Closed 4-vertex rectangle in the XY plane, CCW from +Z."""
    _check_positive(width=width, height=height)
    cx, cy, cz = center
    w, h = width / 2.0, height / 2.0
    return Loop([(cx - w, cy - h, cz), (cx + w, cy - h, cz),
                 (cx + w, cy + h, cz), (cx - w, cy + h, cz)],
                True, "rectangle")


def polygon(sides, radius, center=(0, 0, 0)):
    """Regular polygon (closed), first vertex on +X."""
    if not (3 <= sides <= 32):
        raise ValueError("polygon sides must be 3..32")
    return Loop(circle(radius, sides, center).verts, True, "polygon")


def spiral(turns, radius_start, radius_end, height, segments=128):
    """Planar-to-vertical spiral curve (open): radius sweeps
    radius_start -> radius_end while z rises 0 -> height."""
    _check_positive(turns=turns, radius_start=radius_start,
                    radius_end=radius_end)
    if segments < 2:
        raise ValueError("spiral needs segments >= 2")
    verts = []
    for k in range(segments + 1):
        t = k / segments
        a = 2.0 * math.pi * turns * t
        r = radius_start + (radius_end - radius_start) * t
        verts.append((r * math.cos(a), r * math.sin(a), height * t))
    return Loop(verts, False, "spiral")


def helix(turns, radius, pitch, segments=128):
    """Helical curve (open).  pitch = vertical rise per turn."""
    _check_positive(turns=turns, radius=radius, pitch=pitch)
    if segments < 2:
        raise ValueError("helix needs segments >= 2")
    verts = []
    for k in range(segments + 1):
        t = k / segments
        a = 2.0 * math.pi * turns * t
        verts.append((radius * math.cos(a), radius * math.sin(a),
                      pitch * turns * t))
    return Loop(verts, False, "helix")


# ---------------------------------------------------------------------------
# 3.3 transformations (Loop -> Loop)
# ---------------------------------------------------------------------------

def translate(loop, vec):
    """Move loop by (dx, dy, dz)."""
    return Loop([_add(v, vec) for v in loop.verts], loop.is_closed,
                loop.name)


def translate_mesh(mesh, vec):
    """Move every vertex of a Mesh by (dx, dy, dz) - the Mesh-level
    analogue of translate() (which only operates on a Loop). Useful for
    positioning a self-contained piece (built at its own local origin,
    e.g. a helical_sweep() thread segment that always starts at local
    z=0) into an assembly at a specific offset before combining with +."""
    return Mesh([_add(v, vec) for v in mesh.verts], mesh.edges, mesh.faces)


def rotate(loop, axis, degrees=None, center=(0, 0, 0)):
    """Rotate loop about an axis ("x"/"y"/"z" or 3-tuple) through
    `center` by `degrees`.  Alternatively pass a 3x3 or 4x4 matrix
    (nested sequences) as `axis` with degrees omitted."""
    if degrees is None:
        m = axis
        if not (hasattr(m, "__len__") and len(m) in (3, 4)
                and hasattr(m[0], "__len__")):
            raise ValueError("rotate: pass (axis, degrees) or a matrix")
        out = []
        for p in loop.verts:
            x = m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2]
            y = m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2]
            z = m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2]
            if len(m) == 4:
                x += m[0][3]
                y += m[1][3]
                z += m[2][3]
            out.append((x, y, z))
        return Loop(out, loop.is_closed, loop.name)
    k = _axis_vector(axis)
    a = math.radians(degrees)
    return Loop([_rotate_about(p, k, a, center) for p in loop.verts],
                loop.is_closed, loop.name)


def scale(loop, factor, center=(0, 0, 0)):
    """Scale loop by a uniform factor or per-axis (sx, sy, sz) about
    `center`."""
    if hasattr(factor, "__len__"):
        sx, sy, sz = factor
    else:
        sx = sy = sz = float(factor)
    cx, cy, cz = center
    return Loop([(cx + (p[0] - cx) * sx, cy + (p[1] - cy) * sy,
                  cz + (p[2] - cz) * sz) for p in loop.verts],
                loop.is_closed, loop.name)


def mirror(loop, axis, point=(0, 0, 0)):
    """Mirror loop across a plane through `point`.  axis: "x"/"y"/"z"
    (plane normal) or a normal tuple.  NOTE: mirroring reverses the
    apparent winding - follow with .reversed() when the loop's
    orientation matters."""
    n = _axis_vector(axis)
    out = []
    for p in loop.verts:
        d = _dot(_sub(p, point), n)
        out.append(_sub(p, _mul(n, 2.0 * d)))
    return Loop(out, loop.is_closed, loop.name)


def offset(loop, distance, normal=None):
    """Offset loop within its own plane: positive distance grows the
    loop outward (relative to its winding), negative shrinks it.
    Corners are mitered; miters longer than 4x|distance| are beveled
    into two points (which changes the vertex count).  For straight
    open polylines pass `normal` explicitly (their plane is ambiguous).
    """
    if distance == 0.0:
        return loop.copy()
    n = _axis_vector(normal) if normal is not None else loop.plane_normal()
    u, v, w = _basis_from_normal(n)
    pts = [(_dot(p, u), _dot(p, v)) for p in loop.verts]
    lift = [_dot(p, w) for p in loop.verts]           # out-of-plane part
    m = len(pts)
    if m < 2:
        raise ValueError("offset needs at least 2 vertices")

    def edge_normal(i):
        # outward normal of edge i -> i+1 for a CCW loop: (dy, -dx)
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % m]
        dx, dy = bx - ax, by - ay
        l = math.hypot(dx, dy)
        if l < _EPS:
            return None
        return (dy / l, -dx / l)

    new_pts = []
    new_lift = []
    rng = range(m) if loop.is_closed else range(m)
    for i in rng:
        prev_n = edge_normal((i - 1) % m) if (loop.is_closed or i > 0) \
            else None
        next_n = edge_normal(i) if (loop.is_closed or i < m - 1) else None
        if prev_n is None and next_n is None:
            raise ValueError("offset: degenerate loop edge")
        if prev_n is None or next_n is None:
            nn = next_n or prev_n
            new_pts.append((pts[i][0] + nn[0] * distance,
                            pts[i][1] + nn[1] * distance))
            new_lift.append(lift[i])
            continue
        # miter: intersect the two offset edge lines
        px, py = pts[i]
        cross = prev_n[0] * next_n[1] - prev_n[1] * next_n[0]
        if abs(cross) < 1e-9:                          # collinear edges
            new_pts.append((px + next_n[0] * distance,
                            py + next_n[1] * distance))
            new_lift.append(lift[i])
            continue
        # solve p + d*prev_n + t*prev_dir == p + d*next_n + s*next_dir
        ax, ay = px + prev_n[0] * distance, py + prev_n[1] * distance
        bx, by = px + next_n[0] * distance, py + next_n[1] * distance
        d1 = (-prev_n[1], prev_n[0])                   # prev edge dir
        d2 = (-next_n[1], next_n[0])
        det = d1[0] * (-d2[1]) - d1[1] * (-d2[0])
        t = ((bx - ax) * (-d2[1]) - (by - ay) * (-d2[0])) / det
        mx, my = ax + d1[0] * t, ay + d1[1] * t
        miter_len = math.hypot(mx - px, my - py)
        if miter_len > 4.0 * abs(distance):            # bevel sharp spike
            new_pts.append((ax, ay))
            new_lift.append(lift[i])
            new_pts.append((bx, by))
            new_lift.append(lift[i])
        else:
            new_pts.append((mx, my))
            new_lift.append(lift[i])
    verts = [_add(_add(_mul(u, x), _mul(v, y)), _mul(w, l))
             for (x, y), l in zip(new_pts, new_lift)]
    return Loop(verts, loop.is_closed, loop.name)


# ---------------------------------------------------------------------------
# 3.2 lofting & surface operations (Loop(s) -> Mesh)
# ---------------------------------------------------------------------------

def bridge(loop1, loop2, triangulate=False):
    """Connect two same-count loops with a band of faces (quads, or
    triangles when triangulate=True).  Outward normals when both loops
    are CCW and loop2 lies along loop1's winding normal."""
    n = len(loop1.verts)
    if len(loop2.verts) != n:
        raise ValueError("bridge: loops have %d vs %d verts"
                         % (n, len(loop2.verts)))
    if loop1.is_closed != loop2.is_closed:
        raise ValueError("bridge: cannot mix open and closed loops")
    if n < 2:
        raise ValueError("bridge needs loops with >= 2 verts")
    verts = list(loop1.verts) + list(loop2.verts)
    faces = []
    count = n if loop1.is_closed else n - 1
    for i in range(count):
        j = (i + 1) % n
        a, b, c, d = i, j, n + j, n + i
        if triangulate:
            faces.append((a, b, c))
            faces.append((a, c, d))
        else:
            faces.append((a, b, c, d))
    return Mesh(verts, [], faces)


def loft(loops_list, triangulate=False):
    """Skin N >= 2 same-count loops in order (rings shared between
    successive bands, so the result welds into one surface)."""
    if len(loops_list) < 2:
        raise ValueError("loft needs at least 2 loops")
    n = len(loops_list[0].verts)
    closed = loops_list[0].is_closed
    for lp in loops_list[1:]:
        if len(lp.verts) != n:
            raise ValueError("loft: all loops need the same vert count")
        if lp.is_closed != closed:
            raise ValueError("loft: cannot mix open and closed loops")
    verts = []
    for lp in loops_list:
        verts.extend(lp.verts)
    faces = []
    count = n if closed else n - 1
    for ring in range(len(loops_list) - 1):
        o1 = ring * n
        o2 = (ring + 1) * n
        for i in range(count):
            j = (i + 1) % n
            a, b, c, d = o1 + i, o1 + j, o2 + j, o2 + i
            if triangulate:
                faces.append((a, b, c))
                faces.append((a, c, d))
            else:
                faces.append((a, b, c, d))
    return Mesh(verts, [], faces)


def cap(loop, fill_mode="fan", reverse=False):
    """Close a loop with faces.  fill_mode "fan" adds a centroid vertex
    (right for convex/star-shaped rings); "earclip" triangulates the
    ring itself (handles concave outlines); "ngon" uses the loop's own
    points as a single face - no new vertices, no splitting, the
    leanest topology available (a hand-modeled cube's flat top/bottom
    quad is this).  Only sound for planar, simple (non-self-
    intersecting) loops: Blender accepts a concave n-gon face fine, but
    a non-planar one will shade/triangulate unpredictably at render
    time - "fan"/"earclip" degrade more gracefully there since every
    face they emit is a triangle.  The cap faces along the loop's
    winding normal; reverse=True flips it (e.g. the underside of a
    solid)."""
    if not loop.is_closed:
        raise ValueError("cap requires a closed loop")
    lp = loop.reversed() if reverse else loop
    n = len(lp.verts)
    if n < 3:
        raise ValueError("cap needs >= 3 verts")
    if fill_mode == "fan":
        c = centroid(lp.verts)
        verts = list(lp.verts) + [c]
        faces = [(n, i, (i + 1) % n) for i in range(n)]
        return Mesh(verts, [], faces)
    if fill_mode == "earclip":
        tris = triangulate(lp)
        return Mesh(list(lp.verts), [], tris)
    if fill_mode == "ngon":
        return Mesh(list(lp.verts), [], [tuple(range(n))])
    raise ValueError("cap fill_mode must be 'fan', 'earclip' or 'ngon'")


# --- thru-hole helpers -------------------------------------------------
# A thru-hole is not a boolean: it's two annular caps (cap_annulus, the
# non-solid analogue of cap()) plus a tunnel wall (an ordinary bridge()
# between the two hole rings), all built with matching, correctly-wound
# loops so composition with `+` welds them watertight - the same trick
# office_organizers' desk_caddy uses for its rectangular-cell pockets,
# generalized to any pair of star-shaped outer/hole loops:
#
#   outer_bot = circle(15, 48);  outer_top = translate(outer_bot, (0,0,20))
#   hole_top = project_radial(outer_top, 6, center=(0,0,20))
#   hole_bot = project_radial(outer_bot, 6, center=(0,0,0))
#   solid = (bridge(outer_bot, outer_top)              # outer wall
#            + cap_annulus(outer_top, hole_top)         # top face w/ hole
#            + cap_annulus(outer_bot, hole_bot, reverse=True)  # bottom
#            + bridge(hole_top, hole_bot))               # tunnel wall
#
# When the outer loop's own point count is too coarse for a smooth hole
# (e.g. a 4-point rectangle), resample_loop() it first so project_radial
# has enough points to distribute around the hole.

def resample_loop(loop, per_edge):
    """Subdivide every edge of a closed loop into `per_edge` equal
    parameter-space segments, preserving the original corners exactly.
    Generalizes the classic "N points per side" boundary discretization
    to any polygon loop, so a low-poly outer shape (a rectangle) can be
    given enough points to match a hole's segment count before calling
    project_radial."""
    if not loop.is_closed:
        raise ValueError("resample_loop requires a closed loop")
    if per_edge < 1:
        raise ValueError("resample_loop needs per_edge >= 1")
    verts = loop.verts
    n = len(verts)
    out = []
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        for k in range(per_edge):
            out.append(lerp(a, b, k / per_edge))
    return Loop(out, True, "resample_loop")


def project_radial(loop, radius, center):
    """Loop with the same vertex count and order as `loop`, each point
    placed at `radius` from `center` along the direction toward the
    corresponding point of `loop` (measured in loop's own plane).  This
    is what lets a hole boundary match an arbitrarily-shaped outer loop
    point-for-point so bridge()/cap_annulus() connect them without
    twisting, instead of needing equal, uniformly-spaced points on both
    sides.

    `loop` must be star-shaped around `center`'s projection into its
    own plane (every ray from center crosses the loop exactly once) -
    true for convex outlines (rectangles, circles, regular polygons),
    which covers the common case of a hole through an organizer wall.
    `center`'s own out-of-plane coordinate becomes every output point's
    height, so passing a different z than `loop` sits at is how you
    place the hole ring on a different face (e.g. project the *top*
    outer loop but center the *bottom* face's height)."""
    _check_positive(radius=radius)
    n = loop.plane_normal()
    u, v, _ = _basis_from_normal(n)
    c = tuple(float(x) for x in center)
    out = []
    for p in loop.verts:
        d = _sub(p, c)
        du, dv = _dot(d, u), _dot(d, v)
        length = math.hypot(du, dv)
        if length < _EPS:
            raise ValueError("project_radial: a loop point coincides "
                             "with center - loop is not star-shaped "
                             "around this center")
        out.append(_add(c, _add(_mul(u, radius * du / length),
                                _mul(v, radius * dv / length))))
    return Loop(out, True, "project_radial")


def cap_annulus(outer_loop, inner_loop, reverse=False):
    """Close a face that has a hole in it: bridge two same-count loops
    into a ring instead of a solid disk - the direct analogue of cap()
    for a face that isn't solid.  Point i of inner_loop must correspond
    to point i of outer_loop (build a matching inner_loop with
    project_radial, after resample_loop if the outer loop needs more
    points first).  Faces along outer_loop's own winding normal, same
    convention as cap(); reverse=True flips both loops together for the
    underside of a hole-bearing face - e.g. the exit face of a
    thru-hole."""
    if not (outer_loop.is_closed and inner_loop.is_closed):
        raise ValueError("cap_annulus requires closed loops")
    if len(outer_loop.verts) != len(inner_loop.verts):
        raise ValueError(
            "cap_annulus: outer/inner loop vertex counts differ (%d vs "
            "%d) - build a matching inner loop with project_radial "
            "(after resample_loop if the outer loop needs more points)"
            % (len(outer_loop.verts), len(inner_loop.verts)))
    if reverse:
        return bridge(outer_loop.reversed(), inner_loop.reversed())
    return bridge(outer_loop, inner_loop)


def project_onto_loop(loop, target, center):
    """Loop with the same vertex count and order as `loop`, each point
    relocated to wherever `loop`'s own bearing (angle around `center`,
    measured in `loop`'s own plane) crosses `target`'s boundary - the
    direct generalization of project_radial() for a star-shaped TARGET
    loop instead of a constant radius. Use this over project_radial()
    when the inner boundary is another irregular star-shaped profile
    (e.g. a differently-sized gear profile at a cluster gear's
    shoulder) rather than a circle, and use cap_with_hole() instead of
    either when the loops aren't star-shaped around a shared center.

    `target` must be star-shaped around `center`'s projection into its
    own plane (same requirement project_radial() has for its own
    `loop` argument). Runs in O(n_loop*log(n_target)) via a
    bearing-sorted binary search over target's own points, not
    O(n_loop*n_target) - scales the way project_radial()+cap_annulus()
    does, unlike cap_with_hole()'s ear-clip triangulation (which does
    not scale to gear-profile point counts - see spur_gear.py's own
    module docstring for the measured comparison)."""
    n = loop.plane_normal()
    u, v, _ = _basis_from_normal(n)
    c = tuple(float(x) for x in center)

    def bearing_uv(p):
        d = _sub(p, c)
        du, dv = _dot(d, u), _dot(d, v)
        length = math.hypot(du, dv)
        if length < _EPS:
            raise ValueError("project_onto_loop: a point coincides with "
                              "center - loop is not star-shaped around it")
        return (math.atan2(dv, du) % (2.0 * math.pi)), du, dv

    tgt = sorted((bearing_uv(p) for p in target.verts), key=lambda e: e[0])
    m = len(tgt)
    if m < 3:
        raise ValueError("project_onto_loop: target needs >= 3 verts")
    bearings = [e[0] for e in tgt]

    out = []
    for p in loop.verts:
        th, _, _ = bearing_uv(p)
        i = bisect.bisect_right(bearings, th) % m
        j = (i - 1) % m
        _, du1, dv1 = tgt[j]
        _, du2, dv2 = tgt[i]
        cx, cy = math.cos(th), math.sin(th)
        ex, ey = du2 - du1, dv2 - dv1
        cross_d_e = cx * ey - cy * ex
        if abs(cross_d_e) < _EPS:
            s = math.hypot(du1, dv1)          # ray parallel to the edge - fall back
        else:
            s = (du1 * dv2 - dv1 * du2) / cross_d_e
        out.append(_add(c, _add(_mul(u, s * cx), _mul(v, s * cy))))
    return Loop(out, True, "project_onto_loop")


def _segments_cross_2d(p1, p2, p3, p4, eps=1e-9):
    """True only for a PROPER crossing of segment p1-p2 with p3-p4 (2D) -
    touching at a shared endpoint is not a crossing, since keyhole bridge
    candidates deliberately touch a loop vertex at each end."""
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    return (((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and
            ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)))


def _keyhole_bridge_is_clear(outer_pts, inner_pts, oi, ii):
    """Does the candidate bridge segment outer_pts[oi] -> inner_pts[ii]
    avoid crossing either loop's own boundary (its two edges touching
    the bridge vertex itself are skipped - touching there is expected,
    not a crossing)?"""
    a, b = outer_pts[oi], inner_pts[ii]
    no, ni = len(outer_pts), len(inner_pts)
    for k in range(no):
        if k == oi or (k + 1) % no == oi:
            continue
        if _segments_cross_2d(a, b, outer_pts[k], outer_pts[(k + 1) % no]):
            return False
    for k in range(ni):
        if k == ii or (k + 1) % ni == ii:
            continue
        if _segments_cross_2d(a, b, inner_pts[k], inner_pts[(k + 1) % ni]):
            return False
    return True


def _find_keyhole_bridge(outer_pts, inner_pts, max_candidates=64):
    """Nearest-first search for a mutually-visible (outer_idx, inner_idx)
    pair, each inner point paired with its single nearest outer point,
    tried closest-first until one has a clear line of sight - cheap in
    practice since the globally nearest pair almost always qualifies,
    and correctness comes from the explicit crossing check, not the
    heuristic ordering."""
    no, ni = len(outer_pts), len(inner_pts)
    candidates = []
    for ii in range(ni):
        ix, iy = inner_pts[ii]
        best_oi, best_d2 = 0, None
        for oi in range(no):
            ox, oy = outer_pts[oi]
            d2 = (ox - ix) ** 2 + (oy - iy) ** 2
            if best_d2 is None or d2 < best_d2:
                best_d2, best_oi = d2, oi
        candidates.append((best_d2, best_oi, ii))
    candidates.sort(key=lambda c: c[0])

    for _, oi, ii in candidates[:max_candidates]:
        if _keyhole_bridge_is_clear(outer_pts, inner_pts, oi, ii):
            return oi, ii
    raise ValueError(
        "cap_with_hole: no unobstructed bridge found between the outer "
        "boundary and the hole after %d nearest candidates" % max_candidates)


def cap_with_hole(outer_loop, inner_loop, fill_mode="earclip", reverse=False):
    """Close a face whose hole doesn't share point count or spacing with
    the outer boundary - the general form of cap_annulus for a hole
    that's irregular or star-shaped in its own right (a gear-tooth
    profile, say), where forcing point-for-point correspondence via
    project_radial would put unrelated points on the same ray and
    produce zero-area faces (this is a real, previously-hit failure
    mode, not a hypothetical one).

    Slits the region into a single simple polygon with the standard
    "keyhole" technique: bridge the outer and inner boundaries at a
    mutually-visible vertex pair (found automatically, nearest first,
    checked against both loops' own edges so it can't cut across a
    concave outer boundary or an irregular hole), walk the hole in the
    opposite direction from the outer boundary, then triangulate the
    result with the existing single-loop ear-clipper.

    The bridge visits each of its two vertices twice (arrival and
    return) - naively at the EXACT same coordinate both times, which
    measurably breaks the ear-clipper: its inside() test sees a
    duplicate's other occurrence sitting exactly on/at a nearby
    candidate ear and rejects it as blocked, even though it's the same
    point under a different index. That starves the strict ear test
    near both pinch points and pushes triangulate() into its
    least-reflex-corner fallback there, which doesn't verify global
    simplicity - confirmed empirically (not hypothetically: a
    circle-in-circle washer's total UNSIGNED triangle area came out
    ~44% high, while the SIGNED total matched the true annulus area
    exactly, pinpointing a cluster of locally inverted-winding ears
    right at the pinch, not a wrong triangle count or wrong net area).
    Fixed by nudging only the RETURN occurrence of each bridge vertex a
    tiny relative distance toward the opposite bridge point - still on
    the true bridge line, just no longer bit-identical to its own
    departure twin, which is enough for the ear-clipper to tell the two
    apart. Negligible geometric effect (a 1e-4 fraction of the bridge
    length) compared to FDM tolerances.

    fill_mode: only "earclip" is supported - the seam makes "fan" and
    "ngon" unsound in general (a concave outer boundary breaks fan;
    ngon would bake the seam in as a visible degenerate edge pair)."""
    if not (outer_loop.is_closed and inner_loop.is_closed):
        raise ValueError("cap_with_hole requires closed loops")
    if fill_mode != "earclip":
        raise ValueError("cap_with_hole only supports fill_mode='earclip'")

    lp_outer = outer_loop.reversed() if reverse else outer_loop
    lp_inner = inner_loop.reversed() if reverse else inner_loop

    nrm = lp_outer.plane_normal()
    u, v, _ = _basis_from_normal(nrm)
    outer_pts = [(_dot(p, u), _dot(p, v)) for p in lp_outer.verts]
    inner_pts = [(_dot(p, u), _dot(p, v)) for p in lp_inner.verts]

    oi, ii = _find_keyhole_bridge(outer_pts, inner_pts)
    no, ni = len(outer_pts), len(inner_pts)

    a2, b2 = outer_pts[oi], inner_pts[ii]
    nudge = 1e-4
    a2_ret = (a2[0] + (b2[0] - a2[0]) * nudge, a2[1] + (b2[1] - a2[1]) * nudge)
    b2_ret = (b2[0] + (a2[0] - b2[0]) * nudge, b2[1] + (a2[1] - b2[1]) * nudge)
    a3, b3 = lp_outer.verts[oi], lp_inner.verts[ii]
    a3_ret = lerp(a3, b3, nudge)
    b3_ret = lerp(b3, a3, nudge)

    outer_run_2d = [outer_pts[(oi + k) % no] for k in range(no)] + [a2_ret]
    inner_run_2d = [inner_pts[(ii - k) % ni] for k in range(ni)] + [b2_ret]
    outer_run_3d = [lp_outer.verts[(oi + k) % no] for k in range(no)] + [a3_ret]
    inner_run_3d = [lp_inner.verts[(ii - k) % ni] for k in range(ni)] + [b3_ret]

    slit_2d = outer_run_2d + inner_run_2d
    slit_verts_3d = outer_run_3d + inner_run_3d

    slit_loop = Loop([(x, y, 0.0) for x, y in slit_2d], True, "keyhole")
    tris = triangulate(slit_loop)
    return Mesh(slit_verts_3d, [], tris)


def cap_with_holes(outer_loop, inner_loops, fill_mode="earclip", reverse=False):
    """cap_with_hole generalized to MULTIPLE holes (a floor with two
    separate boss/bore cutouts in it, say) - none of which need to share
    point count/spacing with the outer boundary OR with each other.

    Merges holes into the boundary ONE AT A TIME with the same keyhole-
    slit technique cap_with_hole uses for a single hole, growing the
    combined boundary after each merge, then triangulates once at the
    end - not N independent single-hole caps, which would have no way to
    keep the pieces from overlapping. Bridging the SECOND hole against
    the ALREADY-MERGED (outer + first hole) boundary is what keeps its
    own bridge from cutting across the first hole's own slit: the
    crossing check (_keyhole_bridge_is_clear) is run against whatever
    the CURRENT combined boundary is, and after one merge that boundary
    already includes the first hole's slit as ordinary edges - the same
    check that protects a single hole from a self-crossing bridge
    protects a later hole from crossing an earlier one, with no special
    multi-hole logic needed beyond "merge, then treat the result as the
    new outer boundary."

    Empty inner_loops degenerates to a plain cap() (fan fill, no seam
    needed since there's nothing to bridge to)."""
    if not outer_loop.is_closed or not all(lp.is_closed for lp in inner_loops):
        raise ValueError("cap_with_holes requires closed loops")
    if fill_mode != "earclip":
        raise ValueError("cap_with_holes only supports fill_mode='earclip'")
    if not inner_loops:
        return cap(outer_loop, fill_mode="fan", reverse=reverse)

    lp_outer = outer_loop.reversed() if reverse else outer_loop
    lp_inners = [lp.reversed() if reverse else lp for lp in inner_loops]

    nrm = lp_outer.plane_normal()
    u, v, _ = _basis_from_normal(nrm)

    combined_2d = [(_dot(p, u), _dot(p, v)) for p in lp_outer.verts]
    combined_3d = list(lp_outer.verts)
    nudge = 1e-4

    for lp_inner in lp_inners:
        inner_pts = [(_dot(p, u), _dot(p, v)) for p in lp_inner.verts]
        oi, ii = _find_keyhole_bridge(combined_2d, inner_pts)
        no, ni = len(combined_2d), len(inner_pts)

        a2, b2 = combined_2d[oi], inner_pts[ii]
        a2_ret = (a2[0] + (b2[0] - a2[0]) * nudge, a2[1] + (b2[1] - a2[1]) * nudge)
        b2_ret = (b2[0] + (a2[0] - b2[0]) * nudge, b2[1] + (a2[1] - b2[1]) * nudge)
        a3, b3 = combined_3d[oi], lp_inner.verts[ii]
        a3_ret = lerp(a3, b3, nudge)
        b3_ret = lerp(b3, a3, nudge)

        outer_run_2d = [combined_2d[(oi + k) % no] for k in range(no)] + [a2_ret]
        inner_run_2d = [inner_pts[(ii - k) % ni] for k in range(ni)] + [b2_ret]
        outer_run_3d = [combined_3d[(oi + k) % no] for k in range(no)] + [a3_ret]
        inner_run_3d = [lp_inner.verts[(ii - k) % ni] for k in range(ni)] + [b3_ret]

        combined_2d = outer_run_2d + inner_run_2d
        combined_3d = outer_run_3d + inner_run_3d

    slit_loop = Loop([(x, y, 0.0) for x, y in combined_2d], True, "keyhole_multi")
    tris = triangulate(slit_loop)
    return Mesh(combined_3d, [], tris)


def extrude(loop, vector):
    """Extrude a loop along a vector: the band of side faces between the
    loop and its translated copy (no end caps - add cap() calls, e.g.
    extrude(sq, v) + cap(sq, reverse=True) + cap(translate(sq, v)))."""
    return bridge(loop, translate(loop, vector))


def extrude_normal(loop, distance):
    """Extrude a planar loop along its own winding normal."""
    n = loop.plane_normal()
    return extrude(loop, _mul(n, distance))


def extrude_path(profile_loop, path_loop, twist_degrees=0.0):
    """Sweep a planar profile (XY plane, centered near the origin) along
    a path using rotation-minimizing (parallel transport) frames.
    Closed path -> the tube wraps into a torus-like solid; open path ->
    an open tube (cap the ends or accept the boundary warning).
    twist_degrees: profile rotation accumulated over the path."""
    prof = profile_loop.verts
    path = path_loop.verts
    if len(path) < 2:
        raise ValueError("extrude_path needs a path with >= 2 verts")
    if len(prof) < 2:
        raise ValueError("extrude_path needs a profile with >= 2 verts")
    # profile 2D coords in its own plane
    pn = profile_loop.plane_normal() if len(prof) >= 3 else (0.0, 0.0, 1.0)
    pu, pv, _ = _basis_from_normal(pn)
    prof2d = [(_dot(p, pu), _dot(p, pv)) for p in prof]

    m = len(path)
    closed_path = path_loop.is_closed

    def tangent(k):
        if closed_path:
            a = path[(k - 1) % m]
            b = path[(k + 1) % m]
        else:
            a = path[max(k - 1, 0)]
            b = path[min(k + 1, m - 1)]
        return _normalize(_sub(b, a))

    tangents = [tangent(k) for k in range(m)]

    # parallel-transport the frame along the path
    t0 = tangents[0]
    a = (1.0, 0.0, 0.0) if abs(t0[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _normalize(_sub(a, _mul(t0, _dot(a, t0))))
    frames = []
    prev_t = t0
    for k in range(m):
        tk = tangents[k]
        axis = _cross(prev_t, tk)
        s = _length(axis)
        c = max(-1.0, min(1.0, _dot(prev_t, tk)))
        if s > 1e-12:
            u = _rotate_about(u, _mul(axis, 1.0 / s), math.atan2(s, c))
        u = _normalize(_sub(u, _mul(tk, _dot(u, tk))))   # re-orthogonalize
        frames.append((u, _cross(tk, u), tk))
        prev_t = tk

    denom = m if closed_path else m - 1
    verts = []
    for k in range(m):
        fu, fv, ft = frames[k]
        tw = math.radians(twist_degrees) * k / denom
        cu, su = math.cos(tw), math.sin(tw)
        for (x, y) in prof2d:
            xr = x * cu - y * su
            yr = x * su + y * cu
            verts.append(_add(path[k], _add(_mul(fu, xr), _mul(fv, yr))))

    np = len(prof2d)
    prof_closed = profile_loop.is_closed
    faces = []
    ring_count = m if closed_path else m - 1
    seg_count = np if prof_closed else np - 1
    for k in range(ring_count):
        o1 = k * np
        o2 = ((k + 1) % m) * np
        for i in range(seg_count):
            j = (i + 1) % np
            faces.append((o1 + i, o1 + j, o2 + j, o2 + i))
    return Mesh(verts, [], faces)


def lathe(profile_loop, axis=(0, 0, 1), steps=32, angle_deg=360.0):
    """Revolve a profile around an axis through the origin.

    Open profile (e.g. a vase silhouette in the XZ plane, ascending z,
    x = radius): produces the surface of revolution; endpoints on the
    axis collapse into shared pole vertices, so a profile that starts
    and ends on the axis yields a closed solid (e.g. a sphere).
    Closed profile (a cross-section, CCW as drawn in the XZ plane):
    yields a torus-like solid; with angle_deg < 360 the two swept ends
    are capped with the profile face, giving a watertight wedge."""
    if not (0.0 < angle_deg <= 360.0):
        raise ValueError("lathe angle_deg must be in (0, 360]")
    full = abs(angle_deg - 360.0) < 1e-9
    if steps < (3 if full else 1):
        raise ValueError("lathe needs steps >= 3 (full) or >= 1 (partial)")
    k = _axis_vector(axis)
    prof = profile_loop.verts
    if len(prof) < 2:
        raise ValueError("lathe needs a profile with >= 2 verts")

    ring_steps = steps if full else steps + 1
    verts = []
    rings = []                       # per profile point: list of indices
    for p in prof:
        radial = _sub(p, _mul(k, _dot(p, k)))
        if _length(radial) < 1e-9:                     # on-axis pole
            verts.append(p)
            rings.append([len(verts) - 1] * ring_steps)
            continue
        ring = []
        for s in range(ring_steps):
            a = math.radians(angle_deg) * s / steps
            verts.append(_rotate_about(p, k, a))
            ring.append(len(verts) - 1)
        rings.append(ring)

    faces = []
    npts = len(prof)
    seg_count = npts if profile_loop.is_closed else npts - 1
    for s in range(steps):
        s2 = (s + 1) % ring_steps if full else s + 1
        for i in range(seg_count):
            i2 = (i + 1) % npts
            quad_idx = (rings[i][s], rings[i][s2],
                        rings[i2][s2], rings[i2][s])
            # collapse pole degeneracies to triangles
            face = []
            for idx in quad_idx:
                if not face or face[-1] != idx:
                    face.append(idx)
            if len(face) > 1 and face[0] == face[-1]:
                face.pop()
            if len(face) >= 3:
                faces.append(tuple(face))

    # cap the swept ends of a partial revolution of a closed profile
    if profile_loop.is_closed and not full:
        start = [rings[i][0] for i in range(npts)]
        end = [rings[i][steps] for i in range(npts)]
        # start cap must face against the sweep direction; probe with
        # the first profile point that sits off the axis
        pn = _loop_plane_normal(prof)
        probe = None
        for p in prof:
            radial = _sub(p, _mul(k, _dot(p, k)))
            if _length(radial) > 1e-9:
                probe = radial
                break
        if probe is None:
            raise ValueError("closed lathe profile lies entirely on the "
                             "axis")
        sweep0 = _cross(k, _normalize(probe))
        if _dot(pn, sweep0) > 0.0:
            start = list(reversed(start))
        else:
            end = list(reversed(end))
        faces.append(tuple(start))
        faces.append(tuple(end))
    return Mesh(verts, [], faces)


def helical_sweep(profile, pitch, height, res=64):
    """Sweep a CLOSED (r, dz) cross-section profile helically around +Z,
    advancing `pitch` per full turn, over a total Z span of `height`.

    NOTE - this produces a thin spiral RIBBON, not a solid rod: closing
    each profile back to its own ring wraps a surface around empty space
    (measured, not assumed - the enclosed volume comes out far below even
    a plain cylinder at the root radius).  Anything that needs a genuinely
    solid threaded rod wants helical_solid() instead, which closes the
    profile to the ring one full revolution later.  Keep using this one
    for a bare thread surface the caller intends to cap or combine itself.

    `profile` is either a fixed Loop, or a CALLABLE `abs_z -> Loop` for a
    profile that varies along the sweep - e.g. a thread "runout" that
    blends flat (all points pinned to one radius) near each end, so the
    sweep welds cleanly into neighboring plain-cylinder geometry with no
    boolean/clip needed (see the fasteners family's own use of this).
    The callable is evaluated once per ring at that ring's own absolute
    Z (`zb + dz` for each point, i.e. the same Z a caller would use to
    decide how far into a runout that ring sits) - NOT the ring index,
    so runout blending lands at an exact, resolution-independent Z. Only
    the RADII may vary between evaluations; every evaluation must return
    the same vertex count and the same `dz` per index (only `r` varies)
    - `profile_span`/`n` are computed once, from the first evaluation.

    Either way, `profile.verts` are (r, 0, dz) triples in the XZ plane -
    the same convention `lathe()`'s profile uses (x = radius) - where
    `dz` is the axial offset WITHIN one profile repeat (e.g. root ->
    flank -> crest -> flank, wrapping back to root at the start of the
    next ring), not an absolute height. The profile is treated as closed
    the same way bridge()/loft() treat a closed loop: point n-1 connects
    back to point 0 every ring-step, which is what traces the smooth
    cylindrical "root" surface between ridges when those two points
    share a radius (the classic thread cross-section: root - flank -
    crest - flank, with root repeated as both the first and last point).

    Oversweeps `height` slightly (ring count rounds UP) rather than
    landing exactly on it, since exact axial placement is the caller's
    job - see the fasteners family's own thread "runout" technique for
    how to blend a profile flat at a specific Z before capping it there.

    Returns an OPEN Mesh: the flank/crest strip faces between
    consecutive rings, including the closing "root" faces - no end caps,
    same convention as extrude()/loft()/bridge(). Cap the two ends
    yourself (typically after flattening them to a plain circle first)
    with `cap(first_ring, reverse=False)` for the sweep's own START ring
    (i=0) and `cap(last_ring, reverse=True)` for its END ring - the
    OPPOSITE of a plain vertical loft's usual start/end reverse
    convention, because a helix's simultaneous rotation-and-rise flips
    which ring-traversal direction reads as "outward" relative to a
    non-rotating stack. Confirmed empirically (signed volume, both
    reverse combinations tried) before relying on it, not assumed from
    the vertical-loft convention alone.

    IMPORTANT - like every other geo_helpers operation, the result
    follows the PROFILE's own winding: the cap convention above is for
    a profile that's CCW in its own (r, dz) plane. A profile that's CW
    there (e.g. a thread's root/crest points swapped, like an internal
    -thread cutter shape built as the mirror image of an external one)
    produces the mirror-image solid and needs the OPPOSITE result - call
    `profile.reversed()` before sweeping it, not opposite cap flags
    (confirmed empirically: swapping cap reverse flags alone produces a
    genuine winding CONFLICT, not just an inverted-but-valid volume,
    because only the closing "root" faces would flip - reversing the
    profile flips the flank/crest strips too, which is what's actually
    needed). Mirror-image profiles are common in this family (an
    external and internal thread cross-section are literally each
    other's major_r/minor_r swap) - check which one you have, don't
    assume CCW."""
    _check_positive(pitch=pitch, height=height)
    if res < 3:
        raise ValueError("helical_sweep needs res >= 3")
    varying = callable(profile)
    probe = profile(0.0) if varying else profile
    if not probe.is_closed:
        raise ValueError("helical_sweep requires a closed profile loop "
                         "(point n-1 wraps back to point 0 every "
                         "ring-step, same as bridge()/loft())")
    n = len(probe.verts)
    if n < 2:
        raise ValueError("helical_sweep profile needs >= 2 verts")

    dzs = [p[2] for p in probe.verts]
    profile_span = max(dzs) - min(dzs)
    steps = max(2, int(math.ceil((height - profile_span) * res / pitch)) + 1)

    verts = []
    rings = []
    for i in range(steps):
        ang = 2.0 * math.pi * i / res
        zb = pitch * i / res
        ring = []
        prof_i = profile(zb).verts if varying else profile.verts
        for (r, _, dz) in prof_i:
            verts.append((r * math.cos(ang), r * math.sin(ang), zb + dz))
            ring.append(len(verts) - 1)
        rings.append(ring)

    # Reversed relative to a plain loft's own (a, b, b2, a2) quad order -
    # a helix's simultaneous rotation-and-rise flips which ring-traversal
    # direction reads as outward, confirmed empirically (signed volume)
    # rather than assumed from the vertical-loft convention. See this
    # function's own docstring for the resulting end-cap convention.
    #
    # The WRAPAROUND edge (k = n-1, k2 = 0) is a genuinely different
    # transition from every other k - it closes the profile's own LAST
    # point back to its FIRST (tooth-end back to tooth-start), not a
    # sequential in-profile step - and needs the OPPOSITE vertex order
    # from the rest to keep winding consistent. Confirmed empirically,
    # not assumed: reusing the same formula for it passes topological
    # checks (looks closed/manifold) but reports near-zero volume for a
    # degenerate constant-radius profile that should trivially reduce to
    # a plain cylinder - the wraparound faces' own normals point exactly
    # opposite the regular faces', silently cancelling most of the
    # enclosed volume instead of raising an error.
    faces = []
    for i in range(len(rings) - 1):
        for k in range(n):
            k2 = (k + 1) % n
            if k == n - 1:
                faces.append((rings[i + 1][k2], rings[i][k2],
                              rings[i][k], rings[i + 1][k]))
            else:
                faces.append((rings[i + 1][k], rings[i + 1][k2],
                              rings[i][k2], rings[i][k]))
    return Mesh(verts, [], faces)


def helical_solid(profile, pitch, height, res=64, cap_start=True,
                  cap_end=True):
    """Sweep a thread profile helically around +Z and close it into a
    SOLID rod - one watertight manifold with material all the way to the
    axis, ready to print as-is.  No core cylinder, no second overlapping
    shell, no boolean.

    THE DIFFERENCE FROM helical_sweep(), and it is the whole ballgame:
    helical_sweep() closes each profile back to ITS OWN ring (point n-1
    wraps to point 0 of the same ring), which traces a tall wall spanning
    the tooth's own height and wraps a thin spiral RIBBON around empty
    space - a slinky, not a rod.  Here the profile's last point closes to
    the FIRST point of the ring one full revolution later
    (`rings[i][n-1] -> rings[i+res][0]`).  That band is the root-flat
    cylinder BETWEEN consecutive thread turns, and it is what actually
    encloses the core.  Confirmed by reconstructing a known-good solid
    rod vertex-for-vertex: same vertex count, same face count, same
    face-size histogram, and 720/720 rays cast inward from outside cross
    the boundary an ODD number of times (material present all the way to
    the axis).

    `profile` is a closed Loop of (r, 0, dz) triples - the same (x =
    radius) convention lathe()/helical_sweep() use - listing ONE tooth
    from its bottom root, out over the crest, back to its top root:
    e.g. [(minor_r, 0, 0), (major_r, 0, fdz), (major_r, 0, fdz + crest),
    (minor_r, 0, 2*fdz + crest)].  Point 0 and point n-1 MUST share a
    radius (that shared value is the root radius, and the root band above
    is what connects them turn to turn), and the profile's own dz span
    must be strictly less than `pitch` - the leftover is the root flat.

    Both ends are closed with a flat disk at the root radius plus a
    rising skirt, and the seam where the helix meets each disk is filled
    with TWO faces, not one: the tooth's end cross-section (a planar face
    spanning root->crest->root) and a separate wedge that stays entirely
    on the root cylinder.  Filling that seam with a single n-gon instead
    is topologically valid but leaves a non-planar face bulging out to
    the crest radius, which Blender fan-triangulates arbitrarily into a
    visible artifact at both ends - a real defect caught by eye on a
    rendered mesh, not a theoretical one.

    `height` is quantized to whole ring steps (pitch / res), so the built
    rod lands within one step of what was asked; measure the result with
    Mesh.bounds() if the exact value matters.  A rod shorter than about
    one pitch plus the profile's own span has no room for a single
    turn-to-turn root band, so `height` is raised to that floor.

    Returns `(mesh, start_ring, end_ring)`.  The two rings are the flat
    root-radius circles the end disks are built on, in ordinary
    increasing-angle order (the same order circle() emits, so they drop
    straight into bridge()/cap_annulus()).  They are returned whether or
    not the ends were capped, so a caller can always measure the built
    height off `end_ring` without re-deriving the quantization.

    `cap_start` / `cap_end` control only whether those disks are FILLED.
    Leave one off to weld this thread into surrounding geometry instead
    of sealing it: the open boundary is then exactly that flat ring, so
    an adjoining wall that ends on the identical ring fuses with it
    through ordinary Mesh `+` vertex welding - no bridge, no overlap
    margin, no separate shell.  That is what lets a threaded section sit
    INSIDE a longer part (a bolt's shank-thread-tip chain) or serve as
    the bore of a hollow one (cap both ends off and close the rings with
    cap_annulus() out to an outer wall) while staying a single manifold.
    With both caps off the result is an open thread TUBE, not a solid -
    the caller owns closing it.

    INTERNAL (bore) threads work through the same call, with one extra
    step.  Hand it a profile whose root sits at the LARGER radius and
    crest at the smaller (major_r -> minor_r -> minor_r -> major_r) and
    the sweep still builds correctly, but it encloses the material INSIDE
    the thread surface - that solid is a thread-cutter shape, the exact
    negative of a nut.  For a real bore the material belongs OUTSIDE, so
    call flip_normals() on the result, then close both rings with
    cap_annulus() out to an outer wall.  Do NOT try to get there by
    reversing the profile instead: the disk/skirt/seam construction
    assumes point 0 is the profile's BOTTOM root and point n-1 its TOP,
    and a reversed profile violates that silently.  Verified numerically
    for both orientations (external rod, and a bore whose volume lands
    between the outer_r..major_r and outer_r..minor_r tube volumes,
    exactly as a threaded bore must)."""
    _check_positive(pitch=pitch, height=height)
    if res < 3:
        raise ValueError("helical_solid needs res >= 3")
    if not profile.is_closed:
        raise ValueError("helical_solid requires a closed profile loop")
    pts = profile.verts
    n = len(pts)
    if n < 3:
        raise ValueError("helical_solid profile needs >= 3 verts")
    root_r = pts[0][0]
    if abs(pts[-1][0] - root_r) > 1e-9:
        raise ValueError(
            "helical_solid: the profile's first and last points must share "
            "the root radius (got %.6f and %.6f) - that shared radius is "
            "what the turn-to-turn root band connects" % (root_r, pts[-1][0]))
    if root_r <= 0.0:
        raise ValueError("helical_solid: root radius must be positive")
    span = max(p[2] for p in pts) - min(p[2] for p in pts)
    if span >= pitch:
        raise ValueError(
            "helical_solid: profile dz span (%.6f) must be < pitch (%.6f) - "
            "the leftover is the root flat between turns" % (span, pitch))

    last = max(res + 1, int(round((height - span) * res / pitch)))
    steps = last + 1

    verts = []
    rings = []
    for i in range(steps):
        ang = 2.0 * math.pi * i / res
        zb = pitch * i / res
        ring = []
        for (r, _, dz) in pts:
            verts.append((r * math.cos(ang), r * math.sin(ang), zb + dz))
            ring.append(len(verts) - 1)
        rings.append(ring)

    faces = []
    # flank / crest strips - in-profile steps only, no wraparound
    for i in range(steps - 1):
        for k in range(n - 1):
            faces.append((rings[i][k], rings[i][k + 1],
                          rings[i + 1][k + 1], rings[i + 1][k]))
    # root band - this turn's top root to the NEXT TURN's bottom root
    for i in range(steps - 1 - res):
        faces.append((rings[i + res][0], rings[i + 1 + res][0],
                      rings[i + 1][n - 1], rings[i][n - 1]))

    def _add(v):
        verts.append(v)
        return len(verts) - 1

    # bottom end: flat disk, rising skirt, split seam patch
    z_b = verts[rings[0][0]][2]
    disk_b = [_add((verts[rings[k][0]][0], verts[rings[k][0]][1], z_b))
              for k in range(res)]
    for k in range(res - 1):
        faces.append((rings[k][0], rings[k + 1][0], disk_b[k + 1], disk_b[k]))
    faces.append(tuple(reversed(rings[0])))
    faces.append((disk_b[res - 1], rings[res - 1][0], rings[res][0],
                  rings[0][n - 1], rings[0][0]))
    if cap_start:
        faces.append(tuple(disk_b))

    # top end: the same construction mirrored
    L = steps - 1
    z_t = verts[rings[L][n - 1]][2]
    disk_t = [_add((verts[rings[L - k][n - 1]][0],
                    verts[rings[L - k][n - 1]][1], z_t))
              for k in range(res)]
    for k in range(res - 1):
        faces.append((rings[L - k][n - 1], rings[L - k - 1][n - 1],
                      disk_t[k + 1], disk_t[k]))
    faces.append(tuple(rings[L]))
    faces.append((disk_t[res - 1], rings[L - res + 1][n - 1],
                  rings[L - res][n - 1], rings[L][0], rings[L][n - 1]))
    if cap_end:
        faces.append(tuple(disk_t))

    # Both rings are returned indexed by BEARING: point k sits at angle
    # 2*pi*k/res.  disk_b already is (it is built from rings 0..res-1),
    # but disk_t is built from the LAST res rings, so disk_t[j] sits at
    # bearing (L - j) mod res - reversing it yields increasing bearings
    # but starting from (L + 1) mod res, i.e. rotated by however many
    # steps the sweep happened to finish on.  Handing that out would
    # silently twist any cap_annulus()/bridge() against it: the faces
    # still close and still report positive volume, they just fan across
    # the hole instead of spanning it radially.  Re-index by bearing so
    # point k of either ring pairs with point k of an ordinary circle().
    start_ring = Loop([verts[i] for i in disk_b], True, "thread_start")
    end_ring = Loop([verts[disk_t[(L - k) % res]] for k in range(res)],
                    True, "thread_end")

    mesh = Mesh(verts, [], faces)
    # The disk/skirt/seam faces above coincide with existing sweep verts
    # at each seam (disk point 0 IS the first ring's root), so those quads
    # collapse to triangles - weld_duplicates drops the degenerate corner
    # rather than leaving a face that repeats a vertex index.
    weld_duplicates(mesh, tolerance=1e-5)
    # Built consistently but inside-out relative to Mesh.signed_volume()'s
    # outward convention (a helix's rotation-and-rise flips which
    # traversal reads as outward - same reason helical_sweep() documents
    # an inverted end-cap convention).  Confirmed by signed volume, not
    # assumed.
    flip_normals(mesh)
    return mesh, start_ring, end_ring


# ---------------------------------------------------------------------------
# 3.4 pattern generators
# ---------------------------------------------------------------------------

def array(loop, count, offset_vec):
    """count copies of loop, the i-th offset by i * offset_vec."""
    if count < 1:
        raise ValueError("array count must be >= 1")
    return [translate(loop, _mul(offset_vec, float(i)))
            for i in range(count)]


def radial_array(loop, count, center=(0, 0, 0), axis=(0, 0, 1)):
    """count copies arranged radially (evenly) around axis@center."""
    if count < 1:
        raise ValueError("radial_array count must be >= 1")
    return [rotate(loop, axis, 360.0 * i / count, center)
            for i in range(count)]


def mirror_copy(loop, axis):
    """(original, mirrored) pair."""
    return (loop.copy(), mirror(loop, axis))


def grid(loop, rows, cols, spacing_x, spacing_y):
    """rows x cols copies stepped by spacing_x / spacing_y, row-major."""
    if rows < 1 or cols < 1:
        raise ValueError("grid rows and cols must be >= 1")
    out = []
    for r in range(rows):
        for c in range(cols):
            out.append(translate(loop, (c * spacing_x, r * spacing_y, 0.0)))
    return out


# ---------------------------------------------------------------------------
# 3.5 connectivity
# ---------------------------------------------------------------------------

def connect_rings(ring1, ring2, triangulate=False):
    """Alias for bridge() on closed rings."""
    if not (ring1.is_closed and ring2.is_closed):
        raise ValueError("connect_rings expects closed loops "
                         "(use connect_strips for open ones)")
    return bridge(ring1, ring2, triangulate)


def connect_strips(strip1, strip2):
    """bridge() for open, same-count strips."""
    if strip1.is_closed or strip2.is_closed:
        raise ValueError("connect_strips expects open loops")
    return bridge(strip1, strip2)


def append_mesh(mesh1, mesh2):
    """Combine two meshes.  Indices are re-offset AND coincident
    vertices (within 1e-6 mm) are welded, so parts built from shared
    rings fuse into a single watertight surface."""
    pool = _VertPool()
    out = Mesh()
    for src in (mesh1, mesh2):
        remap = [pool.add(v) for v in src.verts]
        for f in src.faces:
            out.faces.append(tuple(remap[i] for i in f))
        for e in src.edges:
            out.edges.append(tuple(remap[i] for i in e))
    out.verts = pool.verts
    return out


# ---------------------------------------------------------------------------
# 3.6 face helpers
# ---------------------------------------------------------------------------

def quad(a, b, c, d, verts_list):
    """Bounds-checked quad face (a, b, c, d)."""
    n = len(verts_list)
    for idx in (a, b, c, d):
        if not (0 <= idx < n):
            raise ValueError("quad index %d out of bounds (%d verts)"
                             % (idx, n))
    if len({a, b, c, d}) != 4:
        raise ValueError("quad repeats a vertex index")
    return (a, b, c, d)


def fan(center, ring, verts_list):
    """Triangles radiating from vertex index `center` to the ring of
    indices `ring` (closed: wraps last -> first)."""
    n = len(verts_list)
    for idx in [center] + list(ring):
        if not (0 <= idx < n):
            raise ValueError("fan index %d out of bounds (%d verts)"
                             % (idx, n))
    m = len(ring)
    if m < 2:
        raise ValueError("fan needs a ring of >= 2 indices")
    return [(center, ring[i], ring[(i + 1) % m]) for i in range(m)]


def triangulate(loop):
    """Ear-clipping triangulation of a closed (possibly concave) planar
    loop.  Returns triangles as index triples into loop.verts."""
    if not loop.is_closed:
        raise ValueError("triangulate requires a closed loop")
    verts = loop.verts
    n = len(verts)
    if n < 3:
        raise ValueError("triangulate needs >= 3 verts")
    if n == 3:
        return [(0, 1, 2)]
    nrm = _loop_plane_normal(verts)
    u, v, _ = _basis_from_normal(nrm)
    pts = [(_dot(p, u), _dot(p, v)) for p in verts]
    # Newell normal is winding-aligned, so pts is CCW in this basis.

    def cross2(o, a, b):
        return ((a[0] - o[0]) * (b[1] - o[1])
                - (a[1] - o[1]) * (b[0] - o[0]))

    def inside(p, a, b, c):
        # closed-triangle test: a vertex on the ear's boundary (e.g. a
        # reflex corner exactly on the clipping diagonal) blocks the ear
        d1 = cross2(a, b, p)
        d2 = cross2(b, c, p)
        d3 = cross2(c, a, p)
        return d1 >= -_EPS and d2 >= -_EPS and d3 >= -_EPS

    idx = list(range(n))
    tris = []
    guard = 0
    while len(idx) > 3:
        guard += 1
        if guard > 2 * n * n:
            raise ValueError("triangulate: degenerate polygon")
        found = False
        for k in range(len(idx)):
            i0 = idx[(k - 1) % len(idx)]
            i1 = idx[k]
            i2 = idx[(k + 1) % len(idx)]
            if cross2(pts[i0], pts[i1], pts[i2]) <= _EPS:
                continue                       # reflex or collinear
            if any(inside(pts[j], pts[i0], pts[i1], pts[i2])
                   for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            found = True
            break
        if not found:
            # numeric fallback: clip the least-reflex corner
            best_k = max(range(len(idx)), key=lambda k: cross2(
                pts[idx[(k - 1) % len(idx)]], pts[idx[k]],
                pts[idx[(k + 1) % len(idx)]]))
            tris.append((idx[(best_k - 1) % len(idx)], idx[best_k],
                         idx[(best_k + 1) % len(idx)]))
            idx.pop(best_k)
    tris.append(tuple(idx))
    return tris


def fill(loop):
    """Fill a closed loop with triangles (ear clipping)."""
    return Mesh(list(loop.verts), [], triangulate(loop))


# ---------------------------------------------------------------------------
# 3.7 mechanical helpers (FDM domain profiles)
# ---------------------------------------------------------------------------

def snap_fit_connector(width, depth, clearance=0.2, orientation="male"):
    """Cantilever snap-fit barb profile in the XY plane (open along the
    base edge, ready to extrude).  width: base span (x), depth: total
    protrusion (y).  Female = same outline grown by `clearance` for a
    Bambu/Prusa slip fit (the slot outline to build receiving walls
    around)."""
    _check_positive(width=width, depth=depth)
    if clearance < 0.0:
        raise ValueError("clearance must be >= 0")
    if orientation not in ("male", "female"):
        raise ValueError("orientation must be 'male' or 'female'")
    c = clearance if orientation == "female" else 0.0
    w = width
    ws = 0.5 * width + 2.0 * c        # stem width
    ob = 0.15 * width                 # barb overhang per side
    tw = 0.25 * width + 2.0 * c      # tip width
    d = depth + c
    hl = 0.4 * depth                  # head (barb) length
    sl = d - hl                       # stem length
    x0 = (w - ws) / 2.0
    x1 = (w + ws) / 2.0
    pts = [(x0, 0.0, 0.0),
           (x0, sl, 0.0),
           (x0 - ob, sl, 0.0),
           ((w - tw) / 2.0, d, 0.0),
           ((w + tw) / 2.0, d, 0.0),
           (x1 + ob, sl, 0.0),
           (x1, sl, 0.0),
           (x1, 0.0, 0.0)]
    return Loop(pts, False, "snap_%s" % orientation)


def living_hinge(length, thickness, segments_per_wave=8):
    """Serpentine living-hinge centerline in the XY plane (open):
    extrude along Z and thicken to `thickness` walls.  thickness also
    sets the wave rhythm (one wave per ~8x thickness of length)."""
    _check_positive(length=length, thickness=thickness)
    if segments_per_wave < 2:
        raise ValueError("segments_per_wave must be >= 2")
    waves = max(3, int(round(length / (8.0 * thickness))))
    amp = 1.5 * thickness
    total = waves * segments_per_wave
    pts = []
    for k in range(total + 1):
        t = k / total
        pts.append((length * t,
                    amp * math.sin(2.0 * math.pi * waves * t), 0.0))
    return Loop(pts, False, "living_hinge")


def gear_profile(num_teeth, module, pressure_angle=20.0,
                 profile_type="involute"):
    """Closed involute spur-gear outline in the XY plane (extrude to
    make the gear).  module: tooth size in mm (pitch dia = m * teeth)."""
    if profile_type != "involute":
        raise ValueError("only profile_type='involute' is supported")
    if num_teeth < 4:
        raise ValueError("gear needs >= 4 teeth")
    _check_positive(module=module)
    z = num_teeth
    m = module
    pa = math.radians(pressure_angle)
    r_p = m * z / 2.0                 # pitch radius
    r_b = r_p * math.cos(pa)          # base radius
    r_a = r_p + m                     # addendum (tip)
    r_d = r_p - 1.25 * m              # dedendum (root)
    if r_d <= 0.0:
        raise ValueError("too few teeth for this module (root radius <= 0)")

    def inv(alpha):
        return math.tan(alpha) - alpha

    def half_width(r):
        # angular half-thickness of the tooth at radius r
        alpha = math.acos(min(1.0, r_b / r))
        return math.pi / (2.0 * z) + inv(pa) - inv(alpha)

    flank_lo = max(r_b, r_d)
    samples = 6
    radii = [flank_lo + (r_a - flank_lo) * t / (samples - 1)
             for t in range(samples)]
    psi_root = half_width(flank_lo)

    pts = []
    for i in range(z):
        th = 2.0 * math.pi * i / z
        # radial relief down to the root circle (undercut approximation)
        if r_d < flank_lo - 1e-12:
            pts.append((th - psi_root, r_d))
        # right flank, root -> tip (angle increases: stays CCW)
        for r in radii:
            pts.append((th - half_width(r), r))
        # left flank, tip -> root
        for r in reversed(radii):
            pts.append((th + half_width(r), r))
        if r_d < flank_lo - 1e-12:
            pts.append((th + psi_root, r_d))
        # root arc toward the next tooth
        th_next = 2.0 * math.pi * (i + 1) / z
        a0 = th + psi_root
        a1 = th_next - psi_root
        for t in (1.0 / 3.0, 2.0 / 3.0):
            pts.append((a0 + (a1 - a0) * t, r_d))
    verts = [(r * math.cos(a), r * math.sin(a), 0.0) for (a, r) in pts]
    # drop consecutive duplicates (zero-width tip lands)
    out = [verts[0]]
    for p in verts[1:]:
        if distance(p, out[-1]) > 1e-9:
            out.append(p)
    if distance(out[0], out[-1]) < 1e-9:
        out.pop()
    return Loop(out, True, "gear")


def ratchet_teeth(num_teeth, inner_radius, outer_radius, tooth_depth):
    """Closed ratchet-wheel outline in the XY plane (CCW): sharp drop
    faces with long engagement ramps.  Valleys sit at outer_radius -
    tooth_depth, which must stay at or outside inner_radius (the hub
    you will keep solid)."""
    if num_teeth < 3:
        raise ValueError("ratchet needs >= 3 teeth")
    _check_positive(inner_radius=inner_radius, outer_radius=outer_radius,
                    tooth_depth=tooth_depth)
    rv = outer_radius - tooth_depth
    if rv < inner_radius - 1e-9:
        raise ValueError("tooth_depth cuts below inner_radius")
    pts = []
    step = 2.0 * math.pi / num_teeth
    for i in range(num_teeth):
        th = i * step
        pts.append((th, outer_radius))                 # tip
        pts.append((th + 0.08 * step, rv))             # sharp drop face
        for f in (0.3, 0.55, 0.8):                     # gradual ramp up
            pts.append((th + f * step,
                        rv + (outer_radius - rv) * (f - 0.08) / 0.92))
    verts = [(r * math.cos(a), r * math.sin(a), 0.0) for (a, r) in pts]
    return Loop(verts, True, "ratchet")


def thread_profile(pitch, depth, angle=60.0):
    """One thread tooth (open) in the XY plane spanning x in [0, pitch]:
    root - flank - crest - flank - root.  Sweep along helix() with
    extrude_path, or lathe for annular grooves."""
    _check_positive(pitch=pitch, depth=depth)
    if not (0.0 < angle < 180.0):
        raise ValueError("thread angle must be in (0, 180)")
    flank = depth * math.tan(math.radians(angle) / 2.0)
    if 2.0 * flank > pitch + 1e-9:
        raise ValueError("thread too steep: flanks (%.3f) exceed pitch"
                         % (2.0 * flank))
    crest = pitch - 2.0 * flank
    pts = [(0.0, 0.0, 0.0)]
    x = (pitch - crest) / 2.0
    pts.append((x, depth, 0.0))
    if crest > 1e-9:
        pts.append((x + crest, depth, 0.0))
    pts.append((pitch, 0.0, 0.0))
    return Loop(pts, False, "thread")


def spring_curve(turns, inner_radius, outer_radius, pitch, segments=128):
    """Conical spring centerline (open): radius sweeps inner -> outer
    while rising pitch per turn.  Thicken with extrude_path(circle(wire_r),
    ...) for a printable spring."""
    _check_positive(turns=turns, inner_radius=inner_radius,
                    outer_radius=outer_radius, pitch=pitch)
    if segments < 8:
        raise ValueError("spring_curve needs segments >= 8")
    pts = []
    for k in range(segments + 1):
        t = k / segments
        a = 2.0 * math.pi * turns * t
        r = inner_radius + (outer_radius - inner_radius) * t
        pts.append((r * math.cos(a), r * math.sin(a),
                    pitch * turns * t))
    return Loop(pts, False, "spring")


def dovetail(width, depth, num_tails, male=True):
    """Dovetail joint edge profile (open) in the XY plane spanning x in
    [0, width]: male tails protrude +y by depth, female sockets recess
    -y.  Classic 15-degree flare."""
    _check_positive(width=width, depth=depth)
    if num_tails < 1:
        raise ValueError("dovetail needs >= 1 tail")
    flare = depth * math.tan(math.radians(15.0))
    seg = width / (2.0 * num_tails + 1.0)     # pin, tail, pin, ..., pin
    if seg / 2.0 <= flare:
        raise ValueError("tails too narrow for depth (reduce depth or "
                         "num_tails)")
    sign = 1.0 if male else -1.0
    pts = [(0.0, 0.0, 0.0)]
    for i in range(num_tails):
        x0 = seg * (2 * i + 1)                # tail base left
        x1 = x0 + seg                         # tail base right
        pts.append((x0, 0.0, 0.0))
        pts.append((x0 - flare, sign * depth, 0.0))
        pts.append((x1 + flare, sign * depth, 0.0))
        pts.append((x1, 0.0, 0.0))
    pts.append((width, 0.0, 0.0))
    return Loop(pts, False, "dovetail_%s" % ("male" if male else "female"))


def bearing_profile(bore_diameter, outer_diameter,
                    profile_type="angular_contact"):
    """Simplified bearing cross-section (closed, in the XZ plane, x =
    radius) for lathe(): an annular seat/containment ring.
    "angular_contact": chamfered top-outer corner; "deep_groove":
    ball groove notched into the outer face; "thrust": wider flat ring.
    """
    _check_positive(bore_diameter=bore_diameter,
                    outer_diameter=outer_diameter)
    if bore_diameter >= outer_diameter:
        raise ValueError("bore must be smaller than outer diameter")
    rb = bore_diameter / 2.0
    ro = outer_diameter / 2.0
    wall = ro - rb
    if profile_type == "thrust":
        w = 0.40 * outer_diameter
    elif profile_type in ("angular_contact", "deep_groove"):
        w = 0.30 * outer_diameter
    else:
        raise ValueError("profile_type must be 'angular_contact', "
                         "'deep_groove' or 'thrust'")
    # CCW as drawn in the XZ plane (x right, z up)
    if profile_type == "angular_contact":
        ch = 0.25 * wall
        pts = [(rb, 0.0), (ro, 0.0), (ro, w - ch), (ro - ch, w),
               (rb, w)]
    elif profile_type == "deep_groove":
        g = 0.15 * wall                      # groove depth
        pts = [(rb, 0.0), (ro, 0.0), (ro, 0.35 * w),
               (ro - g, 0.5 * w), (ro, 0.65 * w), (ro, w), (rb, w)]
    else:                                     # thrust
        pts = [(rb, 0.0), (ro, 0.0), (ro, w), (rb, w)]
    return Loop([(x, 0.0, z) for (x, z) in pts], True,
                "bearing_%s" % profile_type)


def wall_thickness_fillet(radius, segments=8):
    """Quarter-circle fillet arc (open) for internal corners, clamped
    to the FDM-safe minimum radius (MIN_FILLET_MM, 0.8mm)."""
    if segments < 2:
        raise ValueError("fillet needs segments >= 2")
    r = max(float(radius), MIN_FILLET_MM)
    lp = arc(r, 90.0, segments)
    lp.name = "fillet"
    return lp


# ---------------------------------------------------------------------------
# 3.8 edge & topology utilities (Mesh -> Mesh, in place)
# ---------------------------------------------------------------------------

def flip_normals(mesh):
    """Reverse every face's winding in place."""
    mesh.faces = [tuple(reversed(f)) for f in mesh.faces]
    return mesh


def weld_duplicates(mesh, tolerance=0.001):
    """Merge vertices within `tolerance`, drop faces that degenerate."""
    if tolerance <= 0.0:
        raise ValueError("tolerance must be positive")
    key_of = {}
    remap = []
    verts = []
    for v in mesh.verts:
        key = (round(v[0] / tolerance), round(v[1] / tolerance),
               round(v[2] / tolerance))
        i = key_of.get(key)
        if i is None:
            i = len(verts)
            key_of[key] = i
            verts.append(v)
        remap.append(i)
    mesh.verts = verts
    mesh.faces = [f2 for f2 in
                  (_clean_face(tuple(remap[i] for i in f))
                   for f in mesh.faces) if f2]
    mesh.edges = [tuple(remap[i] for i in e) for e in mesh.edges
                  if remap[e[0]] != remap[e[1]]]
    return mesh


def merge_vertices(mesh, vert_indices):
    """Collapse the given vertices into one at their centroid."""
    idxs = sorted(set(vert_indices))
    if len(idxs) < 2:
        return mesh
    for i in idxs:
        if not (0 <= i < len(mesh.verts)):
            raise ValueError("merge index %d out of bounds" % i)
    target = idxs[0]
    mesh.verts[target] = centroid([mesh.verts[i] for i in idxs])
    remap = {i: target for i in idxs[1:]}
    mesh.faces = [f2 for f2 in
                  (_clean_face(tuple(remap.get(i, i) for i in f))
                   for f in mesh.faces) if f2]
    mesh.edges = [tuple(remap.get(i, i) for i in e) for e in mesh.edges
                  if remap.get(e[0], e[0]) != remap.get(e[1], e[1])]
    return remove_unused_vertices(mesh)


def remove_unused_vertices(mesh):
    """Strip vertices not referenced by any face or edge."""
    used = set()
    for f in mesh.faces:
        used.update(f)
    for e in mesh.edges:
        used.update(e)
    remap = {}
    verts = []
    for i, v in enumerate(mesh.verts):
        if i in used:
            remap[i] = len(verts)
            verts.append(v)
    mesh.verts = verts
    mesh.faces = [tuple(remap[i] for i in f) for f in mesh.faces]
    mesh.edges = [tuple(remap[i] for i in e) for e in mesh.edges]
    return mesh


def _clean_face(f):
    out = []
    for idx in f:
        if not out or out[-1] != idx:
            out.append(idx)
    if len(out) > 1 and out[0] == out[-1]:
        out.pop()
    return tuple(out) if len(out) >= 3 else None


def subdivide_edges(mesh, edges_list, depth=1):
    """Split the given edges (vertex-index pairs) at their midpoints,
    updating every face that uses them.  depth=k splits each edge into
    2**k segments.  Preserves manifoldness."""
    if depth < 1:
        raise ValueError("depth must be >= 1")
    targets = [frozenset(e) for e in edges_list]
    for e in targets:
        if len(e) != 2:
            raise ValueError("edge %r is not a vertex pair" % (set(e),))
    for _ in range(depth):
        target_set = set(targets)
        mids = {}

        def midpoint_index(a, b):
            key = frozenset((a, b))
            i = mids.get(key)
            if i is None:
                i = len(mesh.verts)
                mesh.verts.append(midpoint(mesh.verts[a], mesh.verts[b]))
                mids[key] = i
            return i

        new_faces = []
        for f in mesh.faces:
            nf = []
            for k in range(len(f)):
                a, b = f[k], f[(k + 1) % len(f)]
                nf.append(a)
                if frozenset((a, b)) in target_set:
                    nf.append(midpoint_index(a, b))
            new_faces.append(tuple(nf))
        mesh.faces = new_faces
        next_targets = []
        for key, mi in mids.items():
            a, b = tuple(key)
            next_targets.append(frozenset((a, mi)))
            next_targets.append(frozenset((mi, b)))
        targets = next_targets
    return mesh


def _bevel_rings(mesh, edges_list, segments, leg_for_vertex, ring_points):
    """Shared engine for bevel_edges/fillet_edges.  Requirements: the
    edges form closed loop(s) on a winding-consistent mesh, and every
    face touching a loop vertex contains at least one loop edge (true
    for rims/borders between two surface sheets - the common case)."""
    edge_set = set(frozenset(e) for e in edges_list)
    for e in edge_set:
        if len(e) != 2:
            raise ValueError("bevel edge %r is not a vertex pair"
                             % (set(e),))

    directed = {}
    for fi, f in enumerate(mesh.faces):
        for k in range(len(f)):
            de = (f[k], f[(k + 1) % len(f)])
            if de in directed:
                raise ValueError("mesh winding inconsistent at edge %s - "
                                 "fix before beveling" % (de,))
            directed[de] = fi

    # group edges into closed loops
    adjacency = {}
    for e in edge_set:
        a, b = tuple(e)
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)
    for v, nbrs in adjacency.items():
        if len(nbrs) != 2:
            raise ValueError("bevel edges must form closed loops (vertex "
                             "%d has %d loop edges)" % (v, len(nbrs)))
    loops = []
    unvisited = set(adjacency)
    while unvisited:
        start = next(iter(unvisited))
        walk = [start]
        unvisited.discard(start)
        prev, cur = None, start
        while True:
            nxt = [n for n in adjacency[cur] if n != prev]
            nxt = nxt[0] if nxt else adjacency[cur][0]
            if nxt == start:
                break
            walk.append(nxt)
            unvisited.discard(nxt)
            prev, cur = cur, nxt
        loops.append(walk)

    for walk in loops:
        _bevel_one_loop(mesh, walk, directed, segments, leg_for_vertex,
                        ring_points)
        # rebuild the directed map: faces changed
        directed = {}
        for fi, f in enumerate(mesh.faces):
            for k in range(len(f)):
                directed[(f[k], f[(k + 1) % len(f)])] = fi
    return remove_unused_vertices(mesh)


def _bevel_one_loop(mesh, walk, directed, segments, leg_for_vertex,
                    ring_points):
    n = len(walk)
    side_a = {}                                # face -> True (side A)
    side_b = {}
    for k in range(n):
        a, b = walk[k], walk[(k + 1) % n]
        fa = directed.get((a, b))
        fb = directed.get((b, a))
        if fa is None or fb is None:
            raise ValueError("bevel edge (%d, %d) is not shared by two "
                             "faces" % (a, b))
        side_a[fa] = True
        side_b[fb] = True
    if set(side_a) & set(side_b):
        raise ValueError("a face lies on both sides of the bevel loop - "
                         "unsupported topology")

    ring_verts = set(walk)

    def away_direction(face, v):
        """Unit direction from v along `face`, away from the loop."""
        f = mesh.faces[face]
        k = f.index(v)
        dirs = []
        for nb in (f[(k - 1) % len(f)], f[(k + 1) % len(f)]):
            if nb not in ring_verts:
                dirs.append(_normalize(_sub(mesh.verts[nb],
                                            mesh.verts[v])))
        if not dirs:                            # cap face: all ring verts
            c = centroid([mesh.verts[i] for i in f])
            dirs.append(_normalize(_sub(c, mesh.verts[v])))
        acc = (0.0, 0.0, 0.0)
        for d in dirs:
            acc = _add(acc, d)
        return _normalize(acc)

    rings = {}                                  # v -> [idx0..idxS]
    for v in walk:
        faces_a = [fi for fi in side_a if v in mesh.faces[fi]]
        faces_b = [fi for fi in side_b if v in mesh.faces[fi]]
        if not faces_a or not faces_b:
            raise ValueError("loop vertex %d lacks faces on both sides"
                             % v)
        da = (0.0, 0.0, 0.0)
        for fi in faces_a:
            da = _add(da, away_direction(fi, v))
        db = (0.0, 0.0, 0.0)
        for fi in faces_b:
            db = _add(db, away_direction(fi, v))
        da = _normalize(da)
        db = _normalize(db)
        gamma = math.acos(max(-1.0, min(1.0, _dot(da, db))))
        if gamma < 1e-6:
            raise ValueError("bevel directions collapse at vertex %d "
                             "(coplanar faces)" % v)
        leg = leg_for_vertex(gamma)
        pts = ring_points(mesh.verts[v], da, db, leg, gamma, segments)
        idxs = []
        for p in pts:
            mesh.verts.append(p)
            idxs.append(len(mesh.verts) - 1)
        rings[v] = idxs

    # any face touching a loop vertex must sit on one of the two sides,
    # otherwise the rebuild would tear the mesh
    for fi, f in enumerate(mesh.faces):
        if any(v in rings for v in f):
            if fi not in side_a and fi not in side_b:
                raise ValueError("face %d touches the bevel loop but "
                                 "shares no loop edge - unsupported "
                                 "topology" % fi)

    S = segments
    for fi in side_a:
        mesh.faces[fi] = tuple(rings[v][0] if v in rings else v
                               for v in mesh.faces[fi])
    for fi in side_b:
        mesh.faces[fi] = tuple(rings[v][S] if v in rings else v
                               for v in mesh.faces[fi])
    for k in range(n):
        a, b = walk[k], walk[(k + 1) % n]
        qa, qb = rings[a], rings[b]
        for s in range(S):
            mesh.faces.append((qb[s], qa[s], qa[s + 1], qb[s + 1]))


def bevel_edges(mesh, edges_list, bevel_distance=0.5):
    """Chamfer a closed loop of edges (e.g. a cylinder's rim) by
    cutting `bevel_distance` down each adjacent surface.  Edges must
    form closed loop(s); every face touching the loop must share a loop
    edge (rims and borders qualify).  Min 0.2mm for FDM."""
    if bevel_distance < 0.2:
        raise ValueError("bevel_distance below FDM minimum (0.2mm)")

    def leg(_gamma):
        return bevel_distance

    def ring_points(v, da, db, leg_len, _gamma, _segments):
        return [_add(v, _mul(da, leg_len)), _add(v, _mul(db, leg_len))]

    return _bevel_rings(mesh, edges_list, 1, leg, ring_points)


def fillet_edges(mesh, edges_list, radius=1.0, segments=4):
    """Round a closed loop of edges with a circular arc of `radius`
    (tangent to both adjacent surfaces).  Same topology requirements as
    bevel_edges.  Min radius MIN_FILLET_MM (0.8mm) for FDM."""
    if radius < MIN_FILLET_MM:
        raise ValueError("fillet radius below FDM minimum (%.1fmm)"
                         % MIN_FILLET_MM)
    if segments < 1:
        raise ValueError("fillet segments must be >= 1")

    def leg(gamma):
        return radius / math.tan(gamma / 2.0)

    def ring_points(v, da, db, leg_len, gamma, segs):
        va = _add(v, _mul(da, leg_len))
        vb = _add(v, _mul(db, leg_len))
        bis = _normalize(_add(da, db))
        c = _add(v, _mul(bis, radius / math.sin(gamma / 2.0)))
        ra = _sub(va, c)
        rb = _sub(vb, c)
        axis = _cross(ra, rb)
        if _length(axis) < 1e-12:
            return [va, vb]
        axis = _normalize(axis)
        total = math.acos(max(-1.0, min(1.0, _dot(_normalize(ra),
                                                  _normalize(rb)))))
        return [_add(c, _rotate_about(ra, axis, total * s / segs))
                for s in range(segs + 1)]

    return _bevel_rings(mesh, edges_list, segments, leg, ring_points)


# ---------------------------------------------------------------------------
# 3.9 coordinate helpers
# ---------------------------------------------------------------------------

def polar(radius, angle_deg, z=0.0):
    """(radius, angle) -> (x, y, z)."""
    a = math.radians(angle_deg)
    return (radius * math.cos(a), radius * math.sin(a), z)


def cyl(radius, angle_deg, z):
    """Cylindrical coords -> (x, y, z)."""
    return polar(radius, angle_deg, z)


def spherical(r, theta_deg, phi_deg):
    """Spherical coords -> (x, y, z).  theta: azimuth from +X in the XY
    plane; phi: inclination from +Z."""
    t = math.radians(theta_deg)
    p = math.radians(phi_deg)
    return (r * math.sin(p) * math.cos(t),
            r * math.sin(p) * math.sin(t),
            r * math.cos(p))


def lerp(a, b, t):
    """Linear interpolation between points a and b."""
    return (a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t)


def midpoint(a, b):
    """Average of two points."""
    return lerp(a, b, 0.5)


def distance(a, b):
    """Euclidean distance."""
    return _length(_sub(a, b))


def normal(face_verts):
    """Unit normal of a planar face (Newell's method, winding-aligned)."""
    return _loop_plane_normal(list(face_verts))


def centroid(verts):
    """Average position of all verts."""
    n = len(verts)
    if n == 0:
        raise ValueError("centroid of empty vertex list")
    sx = sum(v[0] for v in verts)
    sy = sum(v[1] for v in verts)
    sz = sum(v[2] for v in verts)
    return (sx / n, sy / n, sz / n)
