"""
Pure geometry core for internal_ratchet.py - build_internal_ratchet_parts
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from internal_ratchet.py for the web configurator
(Pyodide) - see that file's own module docstring for the full design
rationale this reproduces unchanged.

internal_ratchet.py itself never actually used the mathutils.Vector it
imported (every coordinate in that file was already plain tuple/index
math) - so unlike ratchet_pawl.py, there was no vector math to replace
here at all.

Like ratchet_pawl_core.py's build_ratchet_mechanism and the gears
family's planetary sets, build_internal_ratchet_parts returns each part
(ring, hub, pawls) with its own world-space location + Z rotation
already resolved, so a caller can rotate+translate each part's verts and
concatenate for one flat mesh - the same merge-in-Python trick used
throughout this project's web configurators. All pawls share IDENTICAL
local geometry (only their placement differs), so this core builds that
one pawl mesh ONCE and reuses it for every pawl placement - a pure
efficiency choice with zero effect on the resulting geometry, unlike the
Blender wrapper (which still builds N independent mesh datablocks, one
per pawl object, matching the original file's own object-model
behavior exactly).
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


HOLE_SEGMENTS = 32
PIVOT_PAD_MM  = 2.0


# ── Low-level mesh helpers (verbatim from internal_ratchet.py) ─────────

def _circle_pts(radius, n=HOLE_SEGMENTS):
    return [
        (radius * math.cos(2 * math.pi * i / n),
         radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def _matched_hole_segments(outer_point_count, target=HOLE_SEGMENTS):
    """Smallest whole-number multiplier that brings outer_point_count up to
    at least `target`, while staying an EXACT multiple of it."""
    per_edge = max(1, math.ceil(target / outer_point_count))
    return per_edge, outer_point_count * per_edge


def build_profile_with_hole_mesh(outer_points, width_mm, inner_points=None):
    """Closed solid: extrude the outer 2D profile to width_mm and cap both
    ends, with an optional inner boundary cut through as a hole.
    inner_points=None means solid, no hole. Raises ValueError if the
    resulting mesh fails validation."""
    outer = gh.Loop([(x, y, 0.0) for x, y in outer_points], True, "outer")
    outer_top = gh.translate(outer, (0.0, 0.0, width_mm))
    outer_wall = gh.bridge(outer, outer_top)

    if inner_points:
        inner = gh.Loop([(x, y, 0.0) for x, y in inner_points], True, "inner")
        inner_top = gh.translate(inner, (0.0, 0.0, width_mm))
        inner_wall = gh.flip_normals(gh.bridge(inner, inner_top))
        bottom = gh.cap_annulus(outer, inner, reverse=True)
        top = gh.cap_annulus(outer_top, inner_top, reverse=False)
        mesh = outer_wall + inner_wall + bottom + top
    else:
        bottom = gh.cap(outer, fill_mode="earclip", reverse=True)
        top = gh.cap(outer_top, fill_mode="earclip")
        mesh = outer_wall + bottom + top

    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Profile mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


def build_pawl_profile_points(arm_length, arm_width, tip_width, hole_radius):
    """Local-frame pawl boundary: pivot at origin, arm along +X."""
    pivot_pad   = hole_radius + PIVOT_PAD_MM
    taper_start = arm_length * 0.3
    wedge_len   = max(tip_width, 1.0)
    tip_base_x  = arm_length - wedge_len
    if tip_base_x <= taper_start:
        tip_base_x = taper_start + 0.001
    return [
        (-pivot_pad,   arm_width / 2.0),
        (taper_start,  arm_width / 2.0),
        (tip_base_x,   tip_width / 2.0),
        (arm_length,   0.0),
        (tip_base_x,  -tip_width / 2.0),
        (taper_start, -arm_width / 2.0),
        (-pivot_pad,  -arm_width / 2.0),
    ]


# ── Freewheel geometry helpers (verbatim from internal_ratchet.py) ─────

def solve_ring_radii(sizing_mode, tooth_count, module, outer_diameter_mm,
                     ring_wall_thickness_mm, tooth_depth_mm, tooth_depth_auto):
    """Returns (ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm).

    ring_inner_r = root of inner teeth (where teeth originate on the ring wall).
    tip_r        = innermost point of each tooth, pointing toward center.
    """
    if sizing_mode == 'MODULE':
        ring_inner_r = module * tooth_count / 2.0
        ring_outer_r = ring_inner_r + ring_wall_thickness_mm
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * module
    else:  # OUTER_DIAMETER
        ring_outer_r = outer_diameter_mm / 2.0
        ring_inner_r = ring_outer_r - ring_wall_thickness_mm
        eff_module   = ring_inner_r * 2.0 / max(tooth_count, 1)
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * eff_module
    tip_r = ring_inner_r - tooth_depth_mm
    return ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm


def build_inner_teeth_points(tooth_count, ring_inner_r, tip_r):
    """Closed polygon for the inner toothed surface of the ring, going CCW.
    Each tooth: root at ring_inner_r, tip at tip_r (pointing inward toward
    center). Pattern is identical to build_wheel_profile_points in
    ratchet_pawl_core.py, with the outer_radius and root_radius roles
    swapped so teeth point inward."""
    sector_angle = 2.0 * math.pi / tooth_count
    points = []
    for i in range(tooth_count):
        theta = i * sector_angle
        points.append((ring_inner_r * math.cos(theta), ring_inner_r * math.sin(theta)))
        points.append((tip_r       * math.cos(theta), tip_r        * math.sin(theta)))
    return points


def pawl_pivot_radius(hub_outer_r, hole_r):
    """Radial distance from assembly centre to pawl pivot."""
    return hub_outer_r + hole_r + PIVOT_PAD_MM


def auto_pawl_arm_length(hub_outer_r, tip_r, tip_engagement_depth_mm, hole_r):
    """Arm length so the tip reaches tip_r + tip_engagement_depth from the pivot."""
    pivot_r   = pawl_pivot_radius(hub_outer_r, hole_r)
    contact_r = tip_r + tip_engagement_depth_mm
    return max(contact_r - pivot_r, 1.0)


def validate_freewheel(ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, tooth_count,
                        hub_outer_r, bore_r, clearance_mm,
                        pawl_count, pawl_arm_length_mm, pawl_arm_width_mm,
                        pivot_hole_diameter_mm, pivot_hole_compensation_mm):
    errs = []
    if ring_inner_r <= 0:
        errs.append("Ring inner radius ≤ 0 — reduce wall thickness or increase size.")
    if tip_r <= 0:
        errs.append("Tip radius ≤ 0 — reduce tooth depth.")
    if tooth_depth_mm >= ring_inner_r:
        errs.append("Tooth depth (%.2f) ≥ ring inner radius (%.2f)." % (tooth_depth_mm, ring_inner_r))
    max_hub_r = tip_r - clearance_mm
    if hub_outer_r >= max_hub_r:
        errs.append(
            "Hub outer radius (%.2f mm) must be < tip radius − clearance (%.2f mm). "
            "Reduce hub diameter or increase ring size." % (hub_outer_r, max_hub_r)
        )
    if bore_r >= hub_outer_r:
        errs.append("Bore radius (%.2f mm) ≥ hub outer radius (%.2f mm)." % (bore_r, hub_outer_r))
    if tooth_count < 4:
        errs.append("Tooth count must be ≥ 4.")
    if pawl_count < 1:
        errs.append("Pawl count must be ≥ 1.")
    pivot_hole_r = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    if pivot_hole_r * 2 >= pawl_arm_width_mm:
        errs.append("Pivot hole diameter too large for pawl arm width.")
    if pawl_arm_length_mm <= pivot_hole_diameter_mm + pivot_hole_compensation_mm:
        errs.append("Pawl arm length must be greater than pivot hole diameter.")
    return errs


# ── Pure mesh builders (split off from *_object in internal_ratchet.py) ─

def build_ring_mesh(tooth_count, ring_outer_r, ring_inner_r, tip_r, width_mm):
    """Outer boundary is a plain circle; the inner toothed profile is the
    hole. See internal_ratchet.py's own build_ring_object docstring for
    why the outer circle is built at the SAME point count as the inner
    tooth profile (matches annulus_gear.py's own pattern)."""
    inner_pts = build_inner_teeth_points(tooth_count, ring_inner_r, tip_r)
    outer_pts = [(x, y) for x, y, z in gh.circle(ring_outer_r, len(inner_pts)).verts]
    return build_profile_with_hole_mesh(outer_pts, width_mm, inner_pts)


def build_hub_mesh(hub_outer_r, bore_r, width_mm):
    outer_pts = _circle_pts(hub_outer_r, HOLE_SEGMENTS)
    inner_pts = _circle_pts(bore_r,      HOLE_SEGMENTS)
    return build_profile_with_hole_mesh(outer_pts, width_mm, inner_pts)


def build_pawl_mesh(arm_length, arm_width, tip_width, pivot_hole_diameter_mm,
                     pivot_hole_compensation_mm, width_mm):
    """Local-frame pawl mesh (pivot at origin, arm along +X) - the caller
    positions it (see build_pawl_placement)."""
    hole_r = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    pts    = build_pawl_profile_points(arm_length, arm_width, tip_width, hole_r)

    outer_pts = pts
    inner_pts = None
    if hole_r > 0.0:
        per_edge, n = _matched_hole_segments(len(pts))
        if per_edge > 1:
            resampled = gh.resample_loop(
                gh.Loop([(x, y, 0.0) for x, y in pts], True, "pawl_outline"), per_edge)
            outer_pts = [(x, y) for x, y, z in resampled.verts]
        inner_pts = [(x, y) for x, y, z in gh.circle(hole_r, len(outer_pts)).verts]

    return build_profile_with_hole_mesh(outer_pts, width_mm, inner_pts)


def build_pawl_placement(hub_outer_r, pivot_hole_diameter_mm, pivot_hole_compensation_mm,
                          angle, base_location):
    """Returns (location, rotation_z) for a pawl at this angle - the
    world-space placement half of what build_pawl_object used to do in
    one step, split out so the mesh (identical for every pawl) can be
    built once and reused."""
    hole_r = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    pivot_r = pawl_pivot_radius(hub_outer_r, hole_r)
    pivot_x = base_location[0] + pivot_r * math.cos(angle)
    pivot_y = base_location[1] + pivot_r * math.sin(angle)
    pivot_z = base_location[2]
    return (pivot_x, pivot_y, pivot_z), angle


# ── Derived values + top-level build ────────────────────────────────────

def derive_internal_ratchet(sizing_mode, tooth_count, module, outer_diameter_mm,
                             ring_wall_thickness_mm, tooth_depth_mm, tooth_depth_auto,
                             hub_outer_diameter_mm, bore_diameter_mm, bore_compensation_mm,
                             pivot_hole_diameter_mm, pivot_hole_compensation_mm,
                             pawl_arm_length_auto, pawl_arm_length_mm, tip_engagement_depth_mm):
    """Pure derivation, matching internal_ratchet.py's own _compute()
    logic. Never raises."""
    ring_outer_r, ring_inner_r, tip_r, resolved_tooth_depth_mm = solve_ring_radii(
        sizing_mode, tooth_count, module, outer_diameter_mm,
        ring_wall_thickness_mm, tooth_depth_mm, tooth_depth_auto)
    hub_outer_r = hub_outer_diameter_mm / 2.0
    bore_r      = (bore_diameter_mm + bore_compensation_mm) / 2.0
    hole_r      = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    arm_length  = (auto_pawl_arm_length(hub_outer_r, tip_r, tip_engagement_depth_mm, hole_r)
                   if pawl_arm_length_auto else pawl_arm_length_mm)
    return dict(
        ring_outer_r=ring_outer_r, ring_inner_r=ring_inner_r, tip_r=tip_r,
        tooth_depth_mm=resolved_tooth_depth_mm, hub_outer_r=hub_outer_r,
        bore_r=bore_r, arm_length=arm_length,
    )


def build_internal_ratchet_parts(sizing_mode='MODULE', tooth_count=24, module=2.0, outer_diameter_mm=50.0,
                                  ring_wall_thickness_mm=4.0, tooth_depth_mm=1.2, tooth_depth_auto=True,
                                  hub_outer_diameter_mm=20.0, bore_diameter_mm=5.0, bore_compensation_mm=0.0,
                                  clearance_mm=0.3, pawl_count=3, pawl_arm_length_auto=True,
                                  pawl_arm_length_mm=8.0, pawl_arm_width_mm=4.0, pawl_tip_width_mm=1.5,
                                  tip_engagement_depth_mm=0.6, pivot_hole_diameter_mm=2.0,
                                  pivot_hole_compensation_mm=0.0, width_mm=6.0):
    """Returns (parts, derived_dict). `parts` is
    {"ring": {mesh, location, rotation_z}, "hub": {...}, "pawls": [{...}, ...]}
    - same shape as ratchet_pawl_core.build_ratchet_mechanism and the
    gears family's planetary sets. All pawls share one mesh (identical
    local geometry, only placement differs) - see this file's own module
    docstring. Raises ValueError joining every failed check from
    validate_freewheel (matching internal_ratchet.py's own execute(),
    which cancels on ANY of them, not just the first)."""
    derived = derive_internal_ratchet(
        sizing_mode, tooth_count, module, outer_diameter_mm, ring_wall_thickness_mm,
        tooth_depth_mm, tooth_depth_auto, hub_outer_diameter_mm, bore_diameter_mm,
        bore_compensation_mm, pivot_hole_diameter_mm, pivot_hole_compensation_mm,
        pawl_arm_length_auto, pawl_arm_length_mm, tip_engagement_depth_mm)

    errs = validate_freewheel(
        derived["ring_outer_r"], derived["ring_inner_r"], derived["tip_r"], derived["tooth_depth_mm"],
        tooth_count, derived["hub_outer_r"], derived["bore_r"], clearance_mm,
        pawl_count, derived["arm_length"], pawl_arm_width_mm,
        pivot_hole_diameter_mm, pivot_hole_compensation_mm)
    if errs:
        raise ValueError(" ".join(errs))

    base_location = (0.0, 0.0, 0.0)
    ring_mesh = build_ring_mesh(tooth_count, derived["ring_outer_r"], derived["ring_inner_r"],
                                 derived["tip_r"], width_mm)
    hub_mesh = build_hub_mesh(derived["hub_outer_r"], derived["bore_r"], width_mm)
    pawl_mesh = build_pawl_mesh(derived["arm_length"], pawl_arm_width_mm, pawl_tip_width_mm,
                                 pivot_hole_diameter_mm, pivot_hole_compensation_mm, width_mm)

    pawls = []
    for i in range(pawl_count):
        angle = i * 2.0 * math.pi / pawl_count + math.pi / tooth_count
        location, rotation_z = build_pawl_placement(
            derived["hub_outer_r"], pivot_hole_diameter_mm, pivot_hole_compensation_mm, angle, base_location)
        pawls.append(dict(mesh=pawl_mesh, location=location, rotation_z=rotation_z))

    parts = dict(
        ring=dict(mesh=ring_mesh, location=base_location, rotation_z=0.0),
        hub=dict(mesh=hub_mesh, location=base_location, rotation_z=0.0),
        pawls=pawls,
    )
    return parts, derived
