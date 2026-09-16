"""
Pure geometry core for serpentine_spring.py - build_serpentine_spring_solid
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from serpentine_spring.py for the web configurator
(Pyodide) - see that file's own module docstring for the offset-boundary
technique this reproduces unchanged.
"""

import sys
import os
import math

GEO_HELPERS_DIR = r"C:\Users\Safu\OneDrive\Desktop\Blender\Mechanism Library\mechanisms_core"


def _script_dir():
    if GEO_HELPERS_DIR and os.path.isfile(os.path.join(GEO_HELPERS_DIR, "geo_helpers.py")):
        return GEO_HELPERS_DIR
    candidate = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(candidate, "geo_helpers.py")):
        return candidate
    raise RuntimeError(
        "Can't find geo_helpers.py - either place it next to this file, "
        "or set GEO_HELPERS_DIR near the top of this script to wherever "
        "you saved it."
    )


sys.path.insert(0, _script_dir())
import geo_helpers as gh


def _arc_pts(cx, cy, radius, start_angle, end_angle, n_segs, skip_first=False):
    pts = []
    for i in range(n_segs + 1):
        if i == 0 and skip_first:
            continue
        t = i / n_segs
        a = start_angle + t * (end_angle - start_angle)
        pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return pts


def _n_bend_segs(bend_radius):
    return max(8, round(math.pi * bend_radius / 0.2))


def build_spring_centerline(pitch, module_count, leg_length, bend_radius, n_bend, strip_thickness):
    """Centerline path: flat_end → leg → bend → … → leg → flat_end.

    Y anchors are inset by strip_thickness/2 so the outer offset lands exactly
    on spring_width after offset_centerline runs.
    """
    half_t   = strip_thickness / 2
    y_bc_bot = half_t + bend_radius
    y_bc_top = y_bc_bot + leg_length

    def x_leg(i):
        return i * pitch

    path = [(x_leg(0), y_bc_bot)]

    for i in range(module_count + 1):
        xi      = x_leg(i)
        is_even = (i % 2 == 0)
        if is_even:
            path.append((xi, y_bc_top))
            if i < module_count:
                path += _arc_pts(xi + bend_radius, y_bc_top, bend_radius,
                                  math.pi, 0.0, n_bend, skip_first=True)
        else:
            path.append((xi, y_bc_bot))
            if i < module_count:
                path += _arc_pts(xi + bend_radius, y_bc_bot, bend_radius,
                                  math.pi, 2 * math.pi, n_bend, skip_first=True)

    return path


def offset_centerline(path, half_height):
    """Offset path ±half_height along local perpendicular at each point.

    Returns (outer_verts, inner_verts), index-aligned with path.
    At sharp corners the averaged-normal offset produces a slight chamfer
    rather than a sharp miter — visible but under 1 mm at typical strip sizes.
    """
    M = len(path)
    if M < 2:
        raise ValueError("Centerline path has only %d point(s) — need at least 2." % M)

    outer, inner = [], []
    for i in range(M):
        px, py = path[i]

        if i == 0:
            dx = path[1][0] - path[0][0]
            dy = path[1][1] - path[0][1]
        elif i == M - 1:
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
        else:
            dx = path[i + 1][0] - path[i - 1][0]
            dy = path[i + 1][1] - path[i - 1][1]

        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            raise ValueError(
                "Degenerate centerline direction at path index %d (point=%s). "
                "Check for a zero-length leg or degenerate bend radius." % (i, path[i])
            )
        dx /= norm
        dy /= norm
        nx, ny = -dy, dx  # leftward perpendicular

        outer.append((px + nx * half_height, py + ny * half_height))
        inner.append((px - nx * half_height, py - ny * half_height))

    return outer, inner


def build_spring_boundary(pitch, module_count, strip_thickness, leg_length, bend_radius):
    """The spring's full flat silhouette as ONE closed 2D boundary: walk
    the outer offset loop forward, then the inner offset loop backward.
    See serpentine_spring.py's own docstring for the full rationale."""
    n_bend = _n_bend_segs(bend_radius)
    path   = build_spring_centerline(pitch, module_count, leg_length, bend_radius, n_bend, strip_thickness)
    outer, inner = offset_centerline(path, strip_thickness / 2)
    return outer + list(reversed(inner))


def build_spring_mesh(boundary_2d, strip_height_mm):
    """Closed solid: extrude the flat boundary to strip_height_mm and cap
    both ends via geo_helpers (earclip - the boundary is concave). Raises
    ValueError if the resulting mesh fails validation."""
    loop = gh.Loop([(x, y, 0.0) for x, y in boundary_2d], True, "boundary")
    top = gh.translate(loop, (0.0, 0.0, strip_height_mm))

    mesh = (gh.extrude(loop, (0.0, 0.0, strip_height_mm))
            + gh.cap(loop, fill_mode="earclip", reverse=True)
            + gh.cap(top, fill_mode="earclip"))

    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Serpentine spring mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_serpentine_spring(input_mode, module_count, spring_width, pitch,
                              spring_length, strip_thickness):
    """Pure derivation, matching serpentine_spring.py's own draw() logic -
    reports the RAW (unclamped) pitch/leg for the same inline warnings
    draw() shows, plus the FINAL (clamped) pitch/bend_radius/leg_length
    execute() actually builds with (silently floored, same "clamp don't
    cancel" pattern the rest of this project uses). Never raises."""
    if input_mode == 'PITCH_MODE':
        raw_pitch = pitch
        length = module_count * pitch + strip_thickness
    else:
        raw_pitch = (spring_length - strip_thickness) / max(module_count, 1)
        length = spring_length

    raw_bend_radius = raw_pitch / 2.0
    raw_leg = spring_width - strip_thickness - 2.0 * raw_bend_radius

    final_pitch = raw_pitch if raw_pitch >= strip_thickness else max(raw_pitch, strip_thickness + 0.01)
    final_bend_radius = final_pitch / 2.0
    final_leg = max(spring_width - strip_thickness - 2.0 * final_bend_radius, 0.01)

    return dict(
        pitch=final_pitch, bend_radius=final_bend_radius, leg_length=final_leg,
        length=length, raw_pitch=raw_pitch, raw_leg=raw_leg,
        leg_too_small=(raw_leg <= 0), pitch_too_small=(raw_pitch < strip_thickness),
        even_termination=(module_count % 2 == 0),
    )


def build_serpentine_spring_solid(input_mode='PITCH_MODE', module_count=5, spring_width=20.0,
                                   pitch=4.0, spring_length=40.0, strip_height=2.0,
                                   strip_thickness=0.8):
    """Returns (mesh, derived_dict) - a closed, single-solid serpentine
    spring. Raises ValueError for invalid geometry (degenerate centerline
    direction, or a failed mesh validation)."""
    derived = derive_serpentine_spring(input_mode, module_count, spring_width, pitch,
                                        spring_length, strip_thickness)

    boundary_2d = build_spring_boundary(
        pitch=derived["pitch"], module_count=module_count, strip_thickness=strip_thickness,
        leg_length=derived["leg_length"], bend_radius=derived["bend_radius"])
    mesh = build_spring_mesh(boundary_2d, strip_height)
    return mesh, derived
