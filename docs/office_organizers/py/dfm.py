# FDM design-for-manufacturing audit - pure Python, no bpy imports.
# Operates on from_pydata-style (verts, faces) so the same code runs in
# unit tests (plain Python) and inside Blender on generated meshes.

import math

OVERHANG_LIMIT_DEG = 45.0


def validate_pydata(verts, faces):
    """Structural validity: finite 3-tuples, in-bounds indices, no
    degenerate faces. Returns a list of problem strings (empty = ok)."""
    problems = []
    for vi, v in enumerate(verts):
        if len(v) != 3:
            problems.append("vert %d is not a 3-tuple" % vi)
            continue
        if not all(math.isfinite(c) for c in v):
            problems.append("vert %d has non-finite coordinate" % vi)
    nv = len(verts)
    for fi, f in enumerate(faces):
        if len(f) < 3:
            problems.append("face %d has fewer than 3 vertices" % fi)
        if len(set(f)) != len(f):
            problems.append("face %d repeats a vertex index" % fi)
        for idx in f:
            if not (0 <= idx < nv):
                problems.append("face %d index %d out of bounds" % (fi, idx))
                break
    return problems


def manifold_check(verts, faces):
    """Closed 2-manifold with consistent winding: every undirected edge is
    used by exactly two faces, once in each direction. Returns a list of
    problem strings (empty = watertight)."""
    directed = {}
    for fi, f in enumerate(faces):
        for k in range(len(f)):
            e = (f[k], f[(k + 1) % len(f)])
            if e in directed:
                problems = ["directed edge %s used twice (faces %d, %d) - "
                            "inconsistent winding or duplicate face"
                            % (e, directed[e], fi)]
                return problems
            directed[e] = fi
    problems = []
    for (a, b) in directed:
        if (b, a) not in directed:
            problems.append("boundary edge (%d, %d) - mesh is open" % (a, b))
            if len(problems) >= 10:
                problems.append("... (further boundary edges suppressed)")
                break
    return problems


def _face_normal_area(verts, face):
    """Newell's method: area-weighted normal of a (possibly n-gon) face."""
    nx = ny = nz = 0.0
    for k in range(len(face)):
        x0, y0, z0 = verts[face[k]]
        x1, y1, z1 = verts[face[(k + 1) % len(face)]]
        nx += (y0 - y1) * (z0 + z1)
        ny += (z0 - z1) * (x0 + x1)
        nz += (x0 - x1) * (y0 + y1)
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (nx, ny, nz), length * 0.5


def signed_volume(verts, faces):
    """Signed volume via divergence theorem; positive when face winding is
    CCW seen from outside. Faces are fan-triangulated from their first
    vertex (fine for the planar faces our generators emit)."""
    vol = 0.0
    for f in faces:
        x0, y0, z0 = verts[f[0]]
        for k in range(1, len(f) - 1):
            x1, y1, z1 = verts[f[k]]
            x2, y2, z2 = verts[f[k + 1]]
            vol += (x0 * (y1 * z2 - y2 * z1)
                    - y0 * (x1 * z2 - x2 * z1)
                    + z0 * (x1 * y2 - x2 * y1))
    return vol / 6.0


def overhang_faces(verts, faces, limit_deg=OVERHANG_LIMIT_DEG,
                   down=(0.0, 0.0, -1.0)):
    """Indices of faces that face downward at more than `limit_deg` from
    vertical and are not on the build plate, for a print oriented with
    `down` pointing into the bed. Those need supports when printed that
    way. Default down=(0,0,-1) is the modeled Z-up orientation - the
    same formulas as before, just expressed as a projection onto `down`
    instead of a hardcoded Z axis, so this is bit-for-bit identical to
    the old behavior for any existing caller that doesn't pass `down`."""
    if not verts:
        return []
    dlen = math.sqrt(down[0] ** 2 + down[1] ** 2 + down[2] ** 2)
    dx, dy, dz = down[0] / dlen, down[1] / dlen, down[2] / dlen
    depths = [v[0] * dx + v[1] * dy + v[2] * dz for v in verts]
    bed_depth = max(depths)
    flagged = []
    for fi, f in enumerate(faces):
        (nx, ny, nz), area = _face_normal_area(verts, f)
        if area <= 1e-9:
            continue
        length = math.sqrt(nx * nx + ny * ny + nz * nz)
        cos_down = (nx * dx + ny * dy + nz * dz) / length
        if cos_down <= 1e-6:
            continue  # faces up or vertical
        # angle between face and vertical; horizontal faces = 90 deg
        face_angle_from_vertical = math.degrees(math.asin(min(cos_down, 1.0)))
        if face_angle_from_vertical <= limit_deg:
            continue
        if all(abs(depths[i] - bed_depth) < 1e-6 for i in f):
            continue  # build-plate face
        flagged.append(fi)
    return flagged


AXIS_DOWN_CANDIDATES = [
    (0.0, 0.0, -1.0), (0.0, 0.0, 1.0),
    (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
]


def best_orientation_overhangs(verts, faces, limit_deg=OVERHANG_LIMIT_DEG,
                               candidates=None):
    """Try each axis-aligned build direction (default: the 6 world axes,
    modeled Z-up first) and return the overhang_faces() result from
    whichever needs the fewest support faces. A design isn't penalized
    for the one orientation it happens to be modeled in when rotating it
    to a different (but equally valid) print orientation avoids the
    overhang entirely - every generator here is built from axis-aligned
    boxes/walls or cylinders/wedges swept along an axis, so the 6
    axis-aligned candidates are the practical, sufficient search space
    (no arbitrary-rotation search needed)."""
    if candidates is None:
        candidates = AXIS_DOWN_CANDIDATES
    best = None
    for down in candidates:
        flagged = overhang_faces(verts, faces, limit_deg, down)
        if best is None or len(flagged) < len(best):
            best = flagged
    return best


def audit(verts, faces, limit_deg=OVERHANG_LIMIT_DEG):
    """Full audit. Returns dict with:
       ok           - bool, True when the mesh may be exported (manifold)
       errors       - hard failures (structure, manifold, inverted volume)
       warnings     - soft flags (overhangs)
       volume_mm3   - signed volume (mm^3) when computable
    """
    errors = list(validate_pydata(verts, faces))
    warnings = []
    volume = None
    if not errors:
        errors += manifold_check(verts, faces)
    if not errors:
        volume = signed_volume(verts, faces)
        if volume <= 0.0:
            errors.append("signed volume %.3f <= 0 - normals are inverted"
                          % volume)
        over = best_orientation_overhangs(verts, faces, limit_deg)
        if over:
            warnings.append("%d face(s) overhang more than %.0f deg in "
                            "every axis-aligned print orientation and "
                            "will need supports" % (len(over), limit_deg))
    return {
        "ok": not errors,
        "errors": errors,
        "warnings": warnings,
        "volume_mm3": volume,
    }
