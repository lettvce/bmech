"""
Pure geometry core for helical_planetary_gear_set.py -
build_helical_planetary_gear_set is plain Python + geo_helpers math, zero
bpy/bmesh/mathutils dependency. Extracted verbatim from
helical_planetary_gear_set.py for the web configurator (Pyodide) - see
planetary_gear_set.py's own module docstring for the full [PHASING FIX]
derivation this reuses unchanged.

Profile math and the mesh constructions are duplicated here rather than
imported from spur_gear_core.py/helical_gear_core.py/
helical_annulus_gear_core.py, per this family's own established
convention - see this file's own module docstring and cluster_gear.py's
docs for why the planetary-set files specifically keep duplicates.

`max_pressure_angle_deg` is duplicated from gear_matching.py rather than
imported from it (that module's own `import bpy` would break a Pyodide
import) - same convention as spur_gear_core.py's own copy.

Like planetary_gear_set_core.py, build_helical_planetary_gear_set returns
each part (ring, sun, planets) with its own world-space location + Z
rotation already resolved, so a caller can rotate+translate each part's
verts and concatenate for one flat mesh (the merge-in-Python trick
threaded_jar_set_core.py uses for its own two-part assembly).
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
SPACE_PTS       = 4


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


def _build_gear_profile(module, tooth_count, pa_deg, pip_gap=0.0):
    """External involute gear cross-section profile (2D, CCW)."""
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count) - pip_gap / pitch_r
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
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


def _build_annulus_cutter_profile(module, tooth_count, pa_deg, pip_gap=0.0):
    """Annulus bore void profile (add/ded swapped; for the ring gear)."""
    pa_rad  = radians(pa_deg)
    pitch_r = module * tooth_count / 2.0
    base_r  = pitch_r * cos(pa_rad)
    ded_r   = pitch_r - ADDENDUM_COEFF * module
    add_r   = pitch_r + DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count) + pip_gap / pitch_r
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
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


def _make_helical_mesh(module, tooth_count, pa_deg, width_mm,
                        hand_sign, ha_rad, n_slices, pip_gap=0.0):
    """Helical external gear mesh (sun/planet bodies) - geo_helpers loft()
    across n_slices twisted rings + fan caps, no bore. Returns a plain
    geo_helpers Mesh (no bpy object)."""
    profile = _build_gear_profile(module, tooth_count, pa_deg, pip_gap)
    pitch_r = module * tooth_count / 2.0

    rings = []
    for k in range(n_slices):
        z     = width_mm * k / (n_slices - 1)
        twist = hand_sign * z * tan(ha_rad) / pitch_r
        c, s  = cos(twist), sin(twist)
        pts   = [(x * c - y * s, x * s + y * c, z) for x, y in profile]
        rings.append(gh.Loop(pts, True, "ring_%d" % k))

    mesh = (gh.loft(rings)
            + gh.cap(rings[0], fill_mode="fan", reverse=True)
            + gh.cap(rings[-1], fill_mode="fan"))
    return mesh


def _slice_loop(inner_profile, z, twist_rad):
    c, s = cos(twist_rad), sin(twist_rad)
    return gh.Loop([(x * c - y * s, x * s + y * c, z) for x, y in inner_profile], True, "slice")


def build_helical_annulus_solid_mesh(inner_profile, width_mm, outer_r,
                                      hand_sign, ha_rad, pitch_r, n_slices):
    """Ring gear body - twisted inner toothed wall (loft across n_slices
    rings, flip_normals after assembly), a PLAIN untwisted outer wall,
    and cap_annulus for the two end caps, with the outer ring matched to
    the inner profile by POINT COUNT, not angle - see
    gears/ring/helical_annulus_gear_core.py's own build_helical_annulus_mesh
    for the fuller derivation (duplicated here, not imported - see this
    file's own module docstring). Raises ValueError if the resulting mesh
    fails validation."""
    inner_slices = [
        _slice_loop(inner_profile, width_mm * k / (n_slices - 1),
                    hand_sign * (width_mm * k / (n_slices - 1)) * tan(ha_rad) / pitch_r)
        for k in range(n_slices)
    ]

    outer_bot = gh.circle(outer_r, len(inner_profile))
    outer_top = gh.translate(outer_bot, (0.0, 0.0, width_mm))

    outer_wall = gh.bridge(outer_bot, outer_top)
    inner_wall = gh.flip_normals(gh.loft(inner_slices))
    bottom = gh.cap_annulus(outer_bot, inner_slices[0], reverse=True)
    top = gh.cap_annulus(outer_top, inner_slices[-1], reverse=False)

    mesh = outer_wall + inner_wall + bottom + top
    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Helical planetary ring mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_helical_planetary_gear_set(module, sun_teeth, planet_teeth, planet_count,
                                       pressure_angle_deg, helix_angle_deg, width_mm,
                                       ring_wall_mm):
    """Pure derivation, matching helical_planetary_gear_set.py's own
    _derived()/draw() logic. Never raises."""
    ring_teeth = sun_teeth + 2 * planet_teeth
    ha_rad     = radians(helix_angle_deg)
    pa_max = min(
        max_pressure_angle_deg(sun_teeth, ADDENDUM_COEFF),
        max_pressure_angle_deg(planet_teeth, ADDENDUM_COEFF),
        max_pressure_angle_deg(ring_teeth, DEDENDUM_COEFF),
    )
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    r_sun         = module * sun_teeth    / 2.0
    r_planet      = module * planet_teeth / 2.0
    r_ring        = module * ring_teeth   / 2.0
    center_dist   = r_sun + r_planet
    outer_r       = r_ring + DEDENDUM_COEFF * module + ring_wall_mm
    normal_module = module * cos(ha_rad)
    assembly_ok   = (sun_teeth + ring_teeth) % planet_count == 0
    sun_twist_deg = degrees(width_mm * tan(ha_rad) / r_sun)

    return dict(
        ring_teeth=ring_teeth, ha_rad=ha_rad, pressure_angle_deg=pressure_angle_deg,
        pa_max=pa_max, r_sun=r_sun, r_planet=r_planet, r_ring=r_ring,
        center_dist=center_dist, outer_r=outer_r, normal_module=normal_module,
        assembly_ok=assembly_ok, sun_twist_deg=sun_twist_deg,
    )


def build_helical_planetary_gear_set(module=2.0, sun_teeth=12, planet_teeth=18, planet_count=3,
                                      pressure_angle_deg=20.0, helix_angle_deg=20.0, hand='RIGHT',
                                      width_mm=10.0, ring_wall_mm=5.0, pip_gap=0.2, n_slices=16):
    """Returns (parts, derived_dict) - same {"ring", "sun", "planets"}
    shape as planetary_gear_set_core.build_planetary_gear_set (see that
    function's own docstring for how a caller merges the parts into one
    flat mesh). "Hand" selects the SUN; planets and ring are derived
    automatically (external-external = opposite hands, external-internal
    = same hand - see module docstring). Raises ValueError only if the
    ring gear's own mesh fails validation, matching the Blender
    operator's own validation scope."""
    derived = derive_helical_planetary_gear_set(
        module, sun_teeth, planet_teeth, planet_count, pressure_angle_deg,
        helix_angle_deg, width_mm, ring_wall_mm)
    pa = derived["pressure_angle_deg"]
    ha_rad = derived["ha_rad"]
    ring_teeth = derived["ring_teeth"]

    sun_sign    = 1.0 if hand == 'RIGHT' else -1.0
    planet_sign = -sun_sign
    ring_sign   = planet_sign

    cutter_pts = _build_annulus_cutter_profile(module, ring_teeth, pa, pip_gap)
    ring_mesh = build_helical_annulus_solid_mesh(
        cutter_pts, width_mm, derived["outer_r"], ring_sign, ha_rad, derived["r_ring"], n_slices)
    ring_rot_z = (-pi / ring_teeth) if (planet_teeth % 2 == 0) else 0.0

    sun_mesh = _make_helical_mesh(module, sun_teeth, pa, width_mm, sun_sign, ha_rad, n_slices, pip_gap)
    planet_mesh = _make_helical_mesh(module, planet_teeth, pa, width_mm, planet_sign, ha_rad, n_slices, pip_gap)

    angle_step = 2.0 * pi / planet_count
    planet_phi0 = pi - pi / planet_teeth
    planets = []
    for i in range(planet_count):
        theta = i * angle_step
        location = (derived["center_dist"] * cos(theta), derived["center_dist"] * sin(theta), 0.0)
        rotation_z = theta * (sun_teeth / planet_teeth + 1.0) + planet_phi0
        planets.append(dict(mesh=planet_mesh, location=location, rotation_z=rotation_z))

    parts = dict(
        ring=dict(mesh=ring_mesh, location=(0.0, 0.0, 0.0), rotation_z=ring_rot_z),
        sun=dict(mesh=sun_mesh, location=(0.0, 0.0, 0.0), rotation_z=0.0),
        planets=planets,
    )
    return parts, derived
