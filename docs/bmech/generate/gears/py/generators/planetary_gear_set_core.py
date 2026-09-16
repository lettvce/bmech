"""
Pure geometry core for planetary_gear_set.py - build_planetary_gear_set
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from planetary_gear_set.py for the web configurator
(Pyodide) - see that file's own module docstring for the full phasing
derivation ([PHASING FIX] section) this reproduces unchanged.

Profile math and the annulus-ring mesh construction are duplicated here
rather than imported from spur_gear_core.py/annulus_gear_core.py, per
this family's own established convention - see planetary_gear_set.py's
own module docstring ("Kept as this file's own duplicate copies... see
cluster_gear.py's own docs for why the planetary-set files specifically
keep duplicates").

`max_pressure_angle_deg` is duplicated from gear_matching.py rather than
imported from it (that module's own `import bpy` would break a Pyodide
import) - same convention as spur_gear_core.py's own copy.

Unlike a single-body generator, this one places THREE kinds of parts (a
ring, a sun, and `planet_count` planets sharing one mesh) at their own
world-space location + Z rotation - build_planetary_gear_set returns each
part's mesh already placed in that final position/orientation (verts
pre-transformed), so a caller (web page or Blender operator) can just
concatenate them, the same "merge in Python, one flat mesh out" trick
threaded_jar_set_core.py uses for its own two-part assembly.
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
    """External involute gear profile (2D, CCW, for sun/planet bodies)."""
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


def _make_spur_mesh(module, tooth_count, pa_deg, width_mm, pip_gap=0.0):
    """External spur gear mesh (sun/planet bodies) - geo_helpers extrude +
    fan caps, no bore. Returns a plain geo_helpers Mesh (no bpy object)."""
    profile = _build_gear_profile(module, tooth_count, pa_deg, pip_gap)
    loop    = gh.Loop([(x, y, 0.0) for x, y in profile], True, "profile")
    top     = gh.translate(loop, (0.0, 0.0, width_mm))

    mesh = (gh.bridge(loop, top)
            + gh.cap(loop, fill_mode="fan", reverse=True)
            + gh.cap(top, fill_mode="fan"))
    return mesh


def build_annulus_solid_mesh(inner_profile, outer_r, z_bot, z_top):
    """Ring gear body - independently-spaced outer circle matched to
    inner_profile by POINT COUNT (not angle), cap_annulus for the two end
    caps, flip_normals on the inner (toothed) wall after a normal
    bottom-to-top loft - see gears/ring/annulus_gear_core.py's own
    build_annulus_mesh for the fuller winding derivation (duplicated
    here, not imported - see this file's own module docstring). Raises
    ValueError if the resulting mesh fails validation."""
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
        raise ValueError("Planetary ring mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_planetary_gear_set(module, sun_teeth, planet_teeth, planet_count,
                               pressure_angle_deg, ring_wall_mm):
    """Pure derivation, matching planetary_gear_set.py's own _derived()/
    draw() logic - clamps pressure_angle_deg to the tightest of all three
    members' own ceilings. Never raises."""
    ring_teeth = sun_teeth + 2 * planet_teeth
    pa_max = min(
        max_pressure_angle_deg(sun_teeth, ADDENDUM_COEFF),
        max_pressure_angle_deg(planet_teeth, ADDENDUM_COEFF),
        max_pressure_angle_deg(ring_teeth, DEDENDUM_COEFF),
    )
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    r_sun       = module * sun_teeth    / 2.0
    r_planet    = module * planet_teeth / 2.0
    r_ring      = module * ring_teeth   / 2.0
    center_dist = r_sun + r_planet
    outer_r     = r_ring + DEDENDUM_COEFF * module + ring_wall_mm
    assembly_ok = (sun_teeth + ring_teeth) % planet_count == 0

    return dict(
        ring_teeth=ring_teeth, pressure_angle_deg=pressure_angle_deg, pa_max=pa_max,
        r_sun=r_sun, r_planet=r_planet, r_ring=r_ring, center_dist=center_dist,
        outer_r=outer_r, assembly_ok=assembly_ok,
    )


def build_planetary_gear_set(module=2.0, sun_teeth=12, planet_teeth=18, planet_count=3,
                              pressure_angle_deg=20.0, width_mm=10.0, ring_wall_mm=5.0,
                              pip_gap=0.2):
    """Returns (parts, derived_dict). `parts` is
    {"ring": {mesh, location, rotation_z}, "sun": {...},
     "planets": [{mesh, location, rotation_z}, ...]} - the planet meshes
    all share ONE geo_helpers Mesh object (same geometry, different
    placement), matching the Blender operator's own "shared mesh, N
    linked copies" convention. Location/rotation_z are exactly what the
    Blender operator applies to obj.location/obj.rotation_euler.z; a
    caller that wants one flat mesh (e.g. the web configurator) rotates
    each part's own verts by rotation_z about Z and translates by
    location, then concatenates - the same merge-in-Python trick
    threaded_jar_set_core.py uses for its own two-part assembly.

    Raises ValueError only if the ring gear's own mesh fails validation
    (the only validation the original Blender operator itself performs -
    sun/planet meshes and the assembly_ok equal-spacing condition are
    reported for information only, matching planetary_gear_set.py's own
    "No popup here on purpose" comment)."""
    derived = derive_planetary_gear_set(module, sun_teeth, planet_teeth, planet_count,
                                         pressure_angle_deg, ring_wall_mm)
    pa = derived["pressure_angle_deg"]
    ring_teeth = derived["ring_teeth"]

    cutter_pts = _build_annulus_cutter_profile(module, ring_teeth, pa, pip_gap)
    ring_mesh = build_annulus_solid_mesh(cutter_pts, derived["outer_r"], 0.0, width_mm)
    ring_rot_z = (-pi / ring_teeth) if (planet_teeth % 2 == 0) else 0.0

    sun_mesh = _make_spur_mesh(module, sun_teeth, pa, width_mm, pip_gap)
    planet_mesh = _make_spur_mesh(module, planet_teeth, pa, width_mm, pip_gap)

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
