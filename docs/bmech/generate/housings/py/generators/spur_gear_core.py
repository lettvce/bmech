"""
Pure geometry core for spur_gear.py - build_spur_gear_mesh is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim (not rewritten) from spur_gear.py so this exact logic can run
outside Blender too (the web configurator, via Pyodide) with a single
source of truth - spur_gear.py itself now just imports build_spur_gear_mesh
from here and wraps it in a bpy operator.

If this file changes, spur_gear.py's own behavior changes too - there is
no separate copy of this logic left in that file. The web configurator's
own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py) - re-sync after editing this file.

`tooth_profile_ok`/`max_pressure_angle_deg` are duplicated verbatim from
gear_matching.py rather than imported from it - that module's own
`import bpy` at the top would break a Pyodide import, and per this
project's fasteners-family convention, per-generator math stays
self-contained rather than pulled from a shared bpy-touching module.

See spur_gear.py's own module docstring for the full design rationale
(involute math, bore-via-project_radial, the weld_duplicates wrinkle) -
unchanged here, just relocated.
"""

import sys
import os
from math import cos, sin, sqrt, pi, radians, degrees, tan, atan, atan2

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


ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
MIN_TOOTH_COUNT = 5
INVOLUTE_POINTS = 15


# ── Pressure-angle ceiling (duplicated from gear_matching.py - see this
#    file's own module docstring for why) ──────────────────────────────

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


# ── Math layer (verbatim from spur_gear.py) ────────────────────────────

def compute_involute_point(base_radius, t):
    x = base_radius * (cos(t) + t * sin(t))
    y = base_radius * (sin(t) - t * cos(t))
    return (x, y)


def involute_angle_at_radius(base_radius, r):
    ratio = r / base_radius
    if ratio < 1.0:
        return 0.0
    return sqrt(ratio * ratio - 1.0)


def rotate_point(x, y, angle):
    c, s = cos(angle), sin(angle)
    return (x * c - y * s, x * s + y * c)


def build_tooth_profile(module, tooth_count, pressure_angle_deg):
    pa_rad           = radians(pressure_angle_deg)
    pitch_radius     = module * tooth_count / 2.0
    base_radius      = pitch_radius * cos(pa_rad)
    addendum_radius  = pitch_radius + ADDENDUM_COEFF * module
    dedendum_radius  = pitch_radius - DEDENDUM_COEFF * module

    half_tooth_angle = pi / (2.0 * tooth_count)

    t_start = involute_angle_at_radius(base_radius, max(dedendum_radius, base_radius))
    t_tip   = involute_angle_at_radius(base_radius, addendum_radius)

    raw_flank = []
    for i in range(INVOLUTE_POINTS):
        t = t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1)
        raw_flank.append(compute_involute_point(base_radius, t))

    t_pitch = involute_angle_at_radius(base_radius, pitch_radius)
    px, py  = compute_involute_point(base_radius, t_pitch)
    angle_at_pitch = atan2(py, px)

    rotation = half_tooth_angle + angle_at_pitch

    right_flank = [rotate_point(x, -y, rotation) for x, y in raw_flank]

    if base_radius > dedendum_radius:
        root_r_angle = atan2(right_flank[0][1], right_flank[0][0])
        right_flank = [(dedendum_radius * cos(root_r_angle), dedendum_radius * sin(root_r_angle))] + right_flank

    left_flank = [(x, -y) for x, y in right_flank]
    tip_land = [(addendum_radius, 0.0)]

    profile = []
    profile.extend(left_flank)
    profile.extend(tip_land)
    profile.extend(right_flank[::-1])

    return profile


def build_gear_profile(module, tooth_count, pressure_angle_deg):
    pa_rad            = radians(pressure_angle_deg)
    pitch_radius      = module * tooth_count / 2.0
    base_radius       = pitch_radius * cos(pa_rad)
    dedendum_radius   = pitch_radius - DEDENDUM_COEFF * module
    half_tooth_angle  = pi / (2.0 * tooth_count)
    tooth_pitch_angle = 2.0 * pi / tooth_count

    t_pitch = involute_angle_at_radius(base_radius, pitch_radius)
    t_start = involute_angle_at_radius(base_radius, max(dedendum_radius, base_radius))
    px, py  = compute_involute_point(base_radius, t_pitch)
    angle_at_pitch  = atan2(py, px)

    rotation        = half_tooth_angle + angle_at_pitch

    root_pt_local   = compute_involute_point(base_radius, t_start)
    root_pt_rotated = rotate_point(root_pt_local[0], -root_pt_local[1], rotation)
    root_r_local    = atan2(root_pt_rotated[1], root_pt_rotated[0])
    root_l_local    = -root_r_local

    tooth_pts = build_tooth_profile(module, tooth_count, pressure_angle_deg)
    SPACE_PTS = 4

    profile = []
    for i in range(tooth_count):
        tooth_angle = i * tooth_pitch_angle

        for x, y in tooth_pts:
            rx, ry = rotate_point(x, y, tooth_angle)
            profile.append((rx, ry))

        a_start = tooth_angle + root_r_local
        a_end   = tooth_angle + tooth_pitch_angle + root_l_local
        for j in range(1, SPACE_PTS + 1):
            a = a_start + (a_end - a_start) * j / (SPACE_PTS + 1)
            profile.append((dedendum_radius * cos(a), dedendum_radius * sin(a)))

    return profile


# ── Mesh layer (verbatim from spur_gear.py) ────────────────────────────

def build_gear_mesh(profile_points, width_mm, bore_r):
    top_loop = gh.Loop([(x, y, width_mm) for x, y in profile_points], True, "gear_top")
    bot_loop = gh.Loop([(x, y, 0.0) for x, y in profile_points], True, "gear_bot")
    side = gh.extrude(bot_loop, (0.0, 0.0, width_mm))

    if bore_r > 0.0:
        hole_top = gh.project_radial(top_loop, bore_r, center=(0.0, 0.0, width_mm))
        hole_bot = gh.project_radial(bot_loop, bore_r, center=(0.0, 0.0, 0.0))
        mesh = (side
                + gh.cap_annulus(top_loop, hole_top, reverse=False)
                + gh.cap_annulus(bot_loop, hole_bot, reverse=True)
                + gh.bridge(hole_top, hole_bot))
        gh.weld_duplicates(mesh, tolerance=1e-5)
    else:
        mesh = (side
                + gh.cap(top_loop, fill_mode="fan")
                + gh.cap(bot_loop, fill_mode="fan", reverse=True))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_spur_gear(module, tooth_count, pressure_angle_deg, bore_enable,
                      bore_diameter, bore_compensation):
    """Pure derivation, matching spur_gear.py's own draw()/execute() logic:
    clamps pressure_angle_deg down to this tooth count's own ceiling (same
    "silently correct, never cancel" pattern gear_matching.clamp_pressure_angle
    uses), then reports pitch/dedendum radii, undercut risk, and the
    resolved bore radius (0 if disabled or too large). Never raises - safe
    to call every redraw, even on an otherwise-invalid combination."""
    pa_max = max_pressure_angle_deg(tooth_count, ADDENDUM_COEFF)
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    pa_rad          = radians(pressure_angle_deg)
    pitch_radius    = module * tooth_count / 2.0
    dedendum_radius = pitch_radius - DEDENDUM_COEFF * module
    undercut = (pitch_radius * cos(pa_rad)) > dedendum_radius

    bore_r = 0.0
    if bore_enable:
        bore_r = bore_diameter / 2.0 + bore_compensation
        if bore_r <= 0 or bore_r >= dedendum_radius:
            bore_r = 0.0

    return dict(
        pressure_angle_deg=pressure_angle_deg, pa_max=pa_max,
        pitch_d=module * tooth_count, pitch_radius=pitch_radius,
        dedendum_radius=dedendum_radius, undercut=undercut, bore_r=bore_r,
    )


def build_spur_gear_mesh(module=1.0, tooth_count=20, pressure_angle_deg=20.0,
                          width_mm=6.0, bore_enable=True, bore_diameter=5.0,
                          bore_compensation=0.2):
    """Returns (mesh, derived_dict) - a closed, single-solid spur gear,
    with an optional bore hole. Raises ValueError for invalid geometry
    (module too large for this tooth count - dedendum radius collapses to
    zero or below)."""
    derived = derive_spur_gear(module, tooth_count, pressure_angle_deg,
                                bore_enable, bore_diameter, bore_compensation)
    if derived["dedendum_radius"] <= 0:
        raise ValueError("Module too large - dedendum radius is zero or negative")

    profile = build_gear_profile(module, tooth_count, derived["pressure_angle_deg"])
    mesh = build_gear_mesh(profile, width_mm, derived["bore_r"])

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Spur gear failed to build a closed shell: %s" % "; ".join(report.errors))

    return mesh, derived
