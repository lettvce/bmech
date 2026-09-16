"""
Pure geometry core for straight_rack.py - build_straight_rack_mesh is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from straight_rack.py for the web configurator
(Pyodide) - see that file's own module docstring for the fillet/profile
math this reproduces unchanged.
"""

import sys
import os
from math import cos, sin, tan, pi, radians

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


ADDENDUM_COEFF    = 1.0
DEDENDUM_COEFF    = 1.25
ROOT_FILLET_COEFF = 0.38


def build_rack_tooth_profile(module, pressure_angle_deg):
    """One rack tooth in local coords, centered at X=0 - see
    straight_rack.py's own docstring for the fillet derivation."""
    pa_rad       = radians(pressure_angle_deg)
    tooth_pitch  = pi * module
    half_pitch   = tooth_pitch / 2.0
    addendum     = ADDENDUM_COEFF   * module
    dedendum     = DEDENDUM_COEFF   * module
    fillet_r     = ROOT_FILLET_COEFF * module

    y_root       = -dedendum
    y_tip        = addendum
    y_fillet_top = y_root + fillet_r

    half_tooth_at_pitch = (pi * module / 2.0) / 2.0
    flank_slope = tan(pa_rad)

    x_flank_right_at_pitch =  half_tooth_at_pitch
    x_flank_left_at_pitch  = -half_tooth_at_pitch

    x_flank_right_fillet_top = x_flank_right_at_pitch + (dedendum - fillet_r) * flank_slope
    x_flank_left_fillet_top  = x_flank_left_at_pitch  - (dedendum - fillet_r) * flank_slope

    x_flank_right_tip = x_flank_right_at_pitch - addendum * flank_slope
    x_flank_left_tip  = x_flank_left_at_pitch  + addendum * flank_slope

    x_flank_right_root = x_flank_right_fillet_top + fillet_r
    x_flank_left_root  = x_flank_left_fillet_top  - fillet_r

    x_root_right = half_pitch
    x_root_left  = -half_pitch

    def fillet_arc(x_center, side, n=3):
        if fillet_r <= 0:
            return []
        pts = []
        for j in range(1, n + 1):
            phi = (pi / 2.0) * j / (n + 1)
            x = x_center + side * fillet_r * sin(phi)
            y = y_root + fillet_r * (1.0 - cos(phi))
            pts.append((x, y))
        return pts

    fillet_left  = fillet_arc(x_flank_left_root,  +1)
    fillet_right = fillet_arc(x_flank_right_root, -1)

    profile = []
    profile.append((x_root_left,  y_root))
    profile.extend(fillet_left)
    profile.append((x_flank_left_fillet_top, y_fillet_top))
    profile.append((x_flank_left_tip,  y_tip))
    profile.append((x_flank_right_tip, y_tip))
    profile.append((x_flank_right_fillet_top, y_fillet_top))
    profile.extend(fillet_right[::-1])
    profile.append((x_root_right, y_root))

    return profile


def build_rack_profile(module, pressure_angle_deg, tooth_count_rack):
    """Full rack profile: tooth_count_rack teeth tiled along X, closed
    with a rectangular base below the dedendum line - see
    straight_rack.py's own docstring for the dedup rationale."""
    tooth_pitch = pi * module
    dedendum    = DEDENDUM_COEFF * module

    all_pts = []
    for i in range(tooth_count_rack):
        tooth = build_rack_tooth_profile(module, pressure_angle_deg)
        offset_x = i * tooth_pitch
        for x, y in tooth:
            all_pts.append((x + offset_x, y))

    half_pitch = tooth_pitch / 2.0
    x_right = (tooth_count_rack - 1) * tooth_pitch + half_pitch
    x_left  = -half_pitch
    y_base  = -(dedendum + module)

    all_pts.append((x_right, y_base))
    all_pts.append((x_left,  y_base))

    deduped = []
    for pt in all_pts:
        if not deduped or abs(pt[0] - deduped[-1][0]) > 1e-9 or abs(pt[1] - deduped[-1][1]) > 1e-9:
            deduped.append(pt)
    if len(deduped) > 1 and abs(deduped[0][0] - deduped[-1][0]) < 1e-9 and abs(deduped[0][1] - deduped[-1][1]) < 1e-9:
        deduped.pop()

    return deduped


def build_rack_mesh(profile_points, width_mm):
    """Closed rack solid: extrude the 2D profile along Z and cap both
    ends via geo_helpers - see straight_rack.py's own docstring. Raises
    ValueError if the resulting mesh fails validation."""
    profile = gh.Loop([(x, y, 0.0) for x, y in profile_points], True, "rack_profile")
    top = gh.translate(profile, (0.0, 0.0, width_mm))

    mesh = (gh.extrude(profile, (0.0, 0.0, width_mm))
            + gh.cap(profile, fill_mode="earclip", reverse=True)
            + gh.cap(top, fill_mode="earclip"))

    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Rack mesh failed validation: %s" % "; ".join(report.errors))

    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_straight_rack(module, pressure_angle_deg, tooth_count_rack, width_mm):
    """Pure derivation - a rack has no self-intersection pressure-angle
    ceiling (straight flanks, not involute curves), so this mostly just
    reports overall dimensions. Never raises."""
    tooth_pitch = pi * module
    length_mm = tooth_count_rack * tooth_pitch
    return dict(tooth_pitch=tooth_pitch, length_mm=length_mm)


def build_straight_rack_mesh(module=1.0, pressure_angle_deg=20.0, tooth_count_rack=10, width_mm=6.0):
    """Returns (mesh, derived_dict) - a closed, single-solid straight
    involute rack. Raises ValueError for invalid geometry."""
    derived = derive_straight_rack(module, pressure_angle_deg, tooth_count_rack, width_mm)
    profile = build_rack_profile(module, pressure_angle_deg, tooth_count_rack)
    mesh = build_rack_mesh(profile, width_mm)
    return mesh, derived
