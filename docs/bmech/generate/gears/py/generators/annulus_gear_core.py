"""
Pure geometry core for annulus_gear.py - build_annulus_gear_mesh is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim from annulus_gear.py for the web configurator (Pyodide) - see
that file's own module docstring for the matched-point-count outer-ring
technique this reproduces unchanged.

`max_pressure_angle_deg` is duplicated from gear_matching.py rather than
imported from it (that module's own `import bpy` would break a Pyodide
import) - same convention as spur_gear_core.py's own copy.
"""

import sys
import os
from math import pi, cos, sin, sqrt, radians, degrees, atan2, tan, atan

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


INVOLUTE_POINTS = 15
ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
PA_SAFETY_MARGIN_DEG = 0.2


def _tooth_profile_ok(tooth_count, pressure_angle_deg, tip_addendum_coeff):
    pa_rad = radians(pressure_angle_deg)
    ratio  = (1.0 + 2.0 * tip_addendum_coeff / tooth_count) / cos(pa_rad)
    if ratio < 1.0:
        return True
    t_pitch = tan(pa_rad)
    t_tip   = sqrt(ratio * ratio - 1.0)
    angle_pitch = t_pitch - atan(t_pitch)
    angle_tip   = t_tip - atan(t_tip)
    half_tooth_ang = pi / (2.0 * tooth_count)
    return half_tooth_ang > (angle_tip - angle_pitch)


def max_pressure_angle_deg(tooth_count, tip_addendum_coeff, _iterations=60):
    lo, hi = 0.0, radians(89.0)
    for _ in range(_iterations):
        mid = (lo + hi) / 2.0
        if _tooth_profile_ok(tooth_count, degrees(mid), tip_addendum_coeff):
            lo = mid
        else:
            hi = mid
    return degrees(lo)


def _involute_pt(base_r, t):
    return (base_r * (cos(t) + t * sin(t)),
            base_r * (sin(t) - t * cos(t)))


def _involute_t_at_r(base_r, r):
    ratio = r / base_r
    return 0.0 if ratio < 1.0 else sqrt(ratio * ratio - 1.0)


def _rot(x, y, a):
    c, s = cos(a), sin(a)
    return (x * c - y * s, x * s + y * c)


def _build_annulus_cutter_profile(module, tooth_count, pa_deg):
    """2D profile of the void inside an annulus gear bore - see
    annulus_gear.py's own docstring for the add/ded-swap rationale."""
    pa_rad  = radians(pa_deg)
    pitch_r = module * tooth_count / 2.0
    base_r  = pitch_r * cos(pa_rad)
    ded_r   = pitch_r - ADDENDUM_COEFF * module   # annulus tip_r  (inner bore)
    add_r   = pitch_r + DEDENDUM_COEFF * module   # annulus root_r (outer reach)
    half_tooth_ang = pi / (2.0 * tooth_count)
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)

    raw = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
           for i in range(INVOLUTE_POINTS)]

    px, py = _involute_pt(base_r, t_pitch)
    rot    = half_tooth_ang + atan2(py, px)
    right  = [_rot(x, -y, rot) for x, y in raw]

    if base_r > ded_r:
        ra    = atan2(right[0][1], right[0][0])
        right = [(ded_r * cos(ra), ded_r * sin(ra))] + right

    left      = [(x, -y) for x, y in right]
    tooth_pts = left + [(add_r, 0.0)] + right[::-1]

    root_pt      = _involute_pt(base_r, t_start)
    root_rot     = _rot(root_pt[0], -root_pt[1], rot)
    root_r_local = atan2(root_rot[1], root_rot[0])

    SPACE_PTS = 4
    profile   = []
    for i in range(tooth_count):
        ta = i * pitch_arc
        for x, y in tooth_pts:
            profile.append(_rot(x, y, ta))
        a0 = ta + root_r_local
        a1 = ta + pitch_arc - root_r_local
        for j in range(1, SPACE_PTS + 1):
            a = a0 + (a1 - a0) * j / (SPACE_PTS + 1)
            profile.append((ded_r * cos(a), ded_r * sin(a)))

    return profile


def build_annulus_mesh(inner_profile, outer_r, z_bot, z_top):
    """Build the complete annulus ring solid via geo_helpers - outer
    cylindrical wall, inner toothed wall, and two annular end caps. See
    annulus_gear.py's own docstring for the matched-count-but-
    independently-spaced outer ring technique and the flip-after-build
    winding rationale. Raises ValueError if the resulting mesh fails
    validation."""
    inner = gh.Loop([(x, y, z_bot) for x, y in inner_profile], True, "inner")
    outer = gh.circle(outer_r, len(inner_profile), center=(0.0, 0.0, z_bot))
    inner_top = gh.translate(inner, (0.0, 0.0, z_top - z_bot))
    outer_top = gh.translate(outer, (0.0, 0.0, z_top - z_bot))

    outer_wall = gh.bridge(outer, outer_top)
    inner_wall = gh.flip_normals(gh.bridge(inner, inner_top))
    bottom = gh.cap_annulus(outer, inner, reverse=True)
    top = gh.cap_annulus(outer_top, inner_top, reverse=False)

    mesh = outer_wall + inner_wall + bottom + top
    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Annulus gear mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_annulus_gear(module, tooth_count, pressure_angle_deg, width_mm, ring_wall_mm):
    """Pure derivation, matching annulus_gear.py's own _derived()/draw()
    logic - clamps pressure_angle_deg to this tooth count's own ceiling
    MINUS a safety margin (see annulus_gear.py's own PA_SAFETY_MARGIN_DEG
    comment for why the margin, not the theoretical limit itself, is
    used). Never raises."""
    pa_max = max_pressure_angle_deg(tooth_count, DEDENDUM_COEFF) - PA_SAFETY_MARGIN_DEG
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    pitch_r      = module * tooth_count / 2.0
    tip_r        = pitch_r - ADDENDUM_COEFF * module
    root_r_inner = pitch_r + DEDENDUM_COEFF * module
    outer_r      = root_r_inner + ring_wall_mm

    return dict(
        pressure_angle_deg=pressure_angle_deg, pa_max=pa_max, pitch_r=pitch_r,
        tip_r=tip_r, root_r_inner=root_r_inner, outer_r=outer_r,
    )


def build_annulus_gear_mesh(module=2.0, tooth_count=40, pressure_angle_deg=20.0,
                             width_mm=10.0, ring_wall_mm=5.0):
    """Returns (mesh, derived_dict) - a closed, single-solid annulus
    (internal) gear. Raises ValueError for invalid geometry (module too
    large - tip radius zero or negative)."""
    derived = derive_annulus_gear(module, tooth_count, pressure_angle_deg, width_mm, ring_wall_mm)
    if derived["tip_r"] <= 0:
        raise ValueError("Module too large - tip radius is zero or negative")

    inner_profile = _build_annulus_cutter_profile(module, tooth_count, derived["pressure_angle_deg"])
    mesh = build_annulus_mesh(inner_profile, derived["outer_r"], 0.0, width_mm)
    return mesh, derived
