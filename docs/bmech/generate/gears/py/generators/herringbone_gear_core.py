"""
Pure geometry core for herringbone_gear.py - build_herringbone_gear_mesh
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from herringbone_gear.py for the web configurator
(Pyodide) - see that file's own module docstring for the V-shaped
twist-rises-then-falls technique this reproduces unchanged.

`max_pressure_angle_deg` is duplicated from gear_matching.py rather than
imported from it (that module's own `import bpy` would break a Pyodide
import) - same convention as spur_gear_core.py's own copy.
"""

import sys
import os
from math import pi, cos, sin, tan, sqrt, radians, degrees, atan2, atan

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


# ── Involute math (verbatim from herringbone_gear.py) ──────────────────

def _involute_pt(base_r, t):
    return (base_r * (cos(t) + t * sin(t)),
            base_r * (sin(t) - t * cos(t)))


def _involute_t_at_r(base_r, r):
    ratio = r / base_r
    return 0.0 if ratio < 1.0 else sqrt(ratio * ratio - 1.0)


def _rot(x, y, a):
    c, s = cos(a), sin(a)
    return (x * c - y * s, x * s + y * c)


def _build_tooth_profile(module, tooth_count, pa_deg):
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count)

    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
               for i in range(INVOLUTE_POINTS)]

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    px, py  = _involute_pt(base_r, t_pitch)
    rot     = half_tooth_ang + atan2(py, px)
    right   = [_rot(x, -y, rot) for x, y in raw]

    if base_r > ded_r:
        ra    = atan2(right[0][1], right[0][0])
        right = [(ded_r * cos(ra), ded_r * sin(ra))] + right

    left = [(x, -y) for x, y in right]
    return left + [(add_r, 0.0)] + right[::-1]


def _build_gear_profile(module, tooth_count, pa_deg):
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count)
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch      = _involute_t_at_r(base_r, pitch_r)
    t_start      = _involute_t_at_r(base_r, max(ded_r, base_r))
    px, py       = _involute_pt(base_r, t_pitch)
    rot          = half_tooth_ang + atan2(py, px)
    root_pt      = _involute_pt(base_r, t_start)
    root_rot     = _rot(root_pt[0], -root_pt[1], rot)
    root_r_local = atan2(root_rot[1], root_rot[0])

    tooth_pts = _build_tooth_profile(module, tooth_count, pa_deg)
    SPACE_PTS = 4

    profile = []
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


def build_gear_mesh(profile_points, total_width, bore_r, ha_rad, pitch_r,
                     hand_sign, n_slices):
    """Loft the V-shaped (twist rises then falls) tooth profile through
    2*n_slices-1 Z rings, then cap the ends - see herringbone_gear.py's
    own docstring for the full rationale."""
    half_h = total_width / 2.0

    def _rotated(z, twist):
        c, s = cos(twist), sin(twist)
        return [(x * c - y * s, x * s + y * c, z) for x, y in profile_points]

    rings = []
    for k in range(n_slices):
        z     = half_h * k / (n_slices - 1)
        twist = hand_sign * z * tan(ha_rad) / pitch_r
        rings.append(gh.Loop(_rotated(z, twist), True, "hb_ring_%d" % k))
    for k in range(1, n_slices):
        z     = half_h + half_h * k / (n_slices - 1)
        twist = hand_sign * (total_width - z) * tan(ha_rad) / pitch_r
        rings.append(gh.Loop(_rotated(z, twist), True, "hb_ring_%d" % (n_slices + k - 1)))

    side     = gh.loft(rings)
    bot_loop = rings[0]
    top_loop = rings[-1]

    if bore_r > 0.0:
        hole_bot = gh.project_radial(bot_loop, bore_r, center=(0.0, 0.0, 0.0))
        hole_top = gh.project_radial(top_loop, bore_r, center=(0.0, 0.0, total_width))
        mesh = (side
                + gh.cap_annulus(top_loop, hole_top, reverse=False)
                + gh.cap_annulus(bot_loop, hole_bot, reverse=True)
                + gh.bridge(hole_top, hole_bot))
        gh.weld_duplicates(mesh, tolerance=1e-5)
    else:
        mesh = (side
                + gh.cap(top_loop, fill_mode="ngon")
                + gh.cap(bot_loop, fill_mode="ngon", reverse=True))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_herringbone_gear(module, tooth_count, pressure_angle_deg, helix_angle_deg,
                             width_mm, bore_enable, bore_diameter, bore_compensation):
    """Pure derivation, matching herringbone_gear.py's own _derived()/draw()
    logic. Never raises."""
    pa_max = max_pressure_angle_deg(tooth_count, ADDENDUM_COEFF)
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    ha_rad        = radians(helix_angle_deg)
    pitch_r       = module * tooth_count / 2.0
    ded_r         = pitch_r - DEDENDUM_COEFF * module
    add_r         = pitch_r + ADDENDUM_COEFF * module
    half_h        = width_mm / 2.0
    peak_twist    = half_h * tan(ha_rad) / pitch_r
    normal_module = module * cos(ha_rad)
    bore_r        = (bore_diameter / 2.0 + bore_compensation) if bore_enable else 0.0

    return dict(
        pressure_angle_deg=pressure_angle_deg, pa_max=pa_max, ha_rad=ha_rad,
        pitch_r=pitch_r, ded_r=ded_r, add_r=add_r, half_h=half_h,
        peak_twist=peak_twist, normal_module=normal_module, bore_r=bore_r,
    )


def build_herringbone_gear_mesh(module=2.0, tooth_count=20, pressure_angle_deg=20.0,
                                 helix_angle_deg=20.0, hand='RIGHT', width_mm=14.0,
                                 bore_enable=True, bore_diameter=5.0, bore_compensation=0.2,
                                 n_slices=12):
    """Returns (mesh, derived_dict) - a closed, single-solid herringbone
    gear, with an optional bore hole. Raises ValueError for invalid
    geometry (module too large for this tooth count)."""
    derived = derive_herringbone_gear(module, tooth_count, pressure_angle_deg, helix_angle_deg,
                                       width_mm, bore_enable, bore_diameter, bore_compensation)
    if derived["ded_r"] <= 0:
        raise ValueError("Module too large - dedendum radius is zero or negative")

    base_profile = _build_gear_profile(module, tooth_count, derived["pressure_angle_deg"])
    hand_sign    = 1.0 if hand == 'RIGHT' else -1.0
    bore_r       = derived["bore_r"]
    eff_bore_r   = bore_r if (bore_r > 0 and bore_r < derived["ded_r"]) else 0.0

    mesh = build_gear_mesh(base_profile, width_mm, eff_bore_r,
                            derived["ha_rad"], derived["pitch_r"], hand_sign, n_slices)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Herringbone gear failed to build a closed shell: %s" % "; ".join(report.errors))

    return mesh, derived
