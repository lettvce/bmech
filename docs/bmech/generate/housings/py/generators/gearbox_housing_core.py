"""
Pure geometry core for gearbox_housing.py - build_gearbox_housing_mesh is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from gearbox_housing.py for the web configurator
(Pyodide) - see that file's own module docstring for the full design
rationale (the 8-piece construction, why the half-bridge technique beats
N-hole ear-clip on an elongated stadium, and the small-center-distance
failure mode an earlier design hit) this reproduces unchanged.

tip_diameter()/mesh_center_distance() expose the same math
bmech_sync_target() uses to derive gear_od_mm/center_distance_mm from a
picked gear pair in Blender - the web configurator's own "pick two
gears" UI calls these directly instead of reading Match-Target-stamped
custom properties off a live object (there are no Blender objects here).
"""

import sys
import os

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

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import stadium_utils_core as su


ADDENDUM_COEFF = 1.0   # standard external-gear addendum, matches every gears/*.py file


def tip_diameter(module, tooth_count):
    """Same math _tip_diameter() derives from a Match-Target-stamped
    object's own bmech_module/bmech_tooth_count - here taking those two
    numbers directly instead of reading them off a live object."""
    return module * (tooth_count + 2.0 * ADDENDUM_COEFF)


def mesh_center_distance(module_a, teeth_a, module_b, teeth_b):
    """Standard external-gear mesh center-distance formula - only
    meaningful when both gears share the same module (a hard meshing
    requirement); the caller checks that separately, same as
    gearbox_housing.py's own bmech_sync_target()."""
    return module_a * (teeth_a + teeth_b) / 2.0


# ── Derived values + top-level build ────────────────────────────────────

def derive_gearbox_housing(center_distance_mm, gear_od_mm, wall_clearance_mm,
                            wall_thickness_mm, shaft_hole_dia_mm, boss_thickness_mm):
    """Pure derivation, matching gearbox_housing.py's own draw() logic -
    the live r_inner/r_outer/r_boss/r_bore readouts and the
    shaft-overlap warning. Never raises."""
    r_inner = gear_od_mm / 2.0 + wall_clearance_mm
    r_outer = r_inner + wall_thickness_mm
    r_boss = shaft_hole_dia_mm / 2.0 + boss_thickness_mm
    r_bore = shaft_hole_dia_mm / 2.0
    return dict(
        r_inner=r_inner, r_outer=r_outer, r_boss=r_boss, r_bore=r_bore,
        shafts_overlap=(center_distance_mm < shaft_hole_dia_mm),
    )


def build_gearbox_housing_mesh(center_distance_mm=30.0, gear_od_mm=24.0, wall_clearance_mm=0.3,
                                wall_thickness_mm=2.0, wall_height_mm=10.0, base_thickness_mm=2.0,
                                shaft_hole_dia_mm=5.0, boss_thickness_mm=2.0, boss_height_mm=4.0,
                                segments=32, internal_compensation_mm=0.075,
                                external_compensation_mm=0.05):
    """Returns (mesh, derived_dict) - a closed, single-solid two-shaft
    gearbox housing (stadium base + wall, bosses, through-bores).
    `derived` includes the same NOMINAL (uncompensated) r_inner/r_outer/
    wall_top_z/segments/shaft_hole_dia_mm/half_span values
    gearbox_housing.py's own execute() stamps as custom properties for
    gearbox_lid.py's Match Target to read - build_gearbox_lid_mesh takes
    those same four as direct kwargs instead. Raises ValueError for
    invalid geometry (center distance smaller than the shaft holes, or a
    failed mesh validation)."""
    half_span = center_distance_mm / 2.0
    derived_raw = derive_gearbox_housing(center_distance_mm, gear_od_mm, wall_clearance_mm,
                                          wall_thickness_mm, shaft_hole_dia_mm, boss_thickness_mm)
    r_inner, r_outer = derived_raw["r_inner"], derived_raw["r_outer"]
    r_boss, r_bore = derived_raw["r_boss"], derived_raw["r_bore"]

    if center_distance_mm < shaft_hole_dia_mm:
        raise ValueError("Center Distance is smaller than the shaft holes - they'd overlap")

    # FDM compensation - kept in SEPARATE "used for geometry" variables,
    # so the returned derived dict stays NOMINAL - see gearbox_housing.py's
    # own execute() comment for why (gearbox_lid.py applies its OWN
    # compensation against this same as-designed target rather than
    # compounding this housing's correction into its own math).
    r_inner_c = r_inner + internal_compensation_mm
    r_bore_c = r_bore + internal_compensation_mm
    r_outer_c = r_outer + external_compensation_mm
    r_boss_c = r_boss + external_compensation_mm

    z_base_top = base_thickness_mm
    z_wall_top = z_base_top + wall_height_mm
    has_boss = boss_height_mm > 1e-6
    z_boss_top = z_base_top + boss_height_mm if has_boss else z_base_top
    shaft_xs = (-half_span, half_span)
    arc_segments = segments
    ring_segments = arc_segments + 3

    outer_z0 = su.stadium_loop(half_span, r_outer_c, arc_segments, 0.0, "outer_z0")
    outer_ztop = su.stadium_loop(half_span, r_outer_c, arc_segments, z_wall_top, "outer_ztop")
    inner_zbase = su.stadium_loop(half_span, r_inner_c, arc_segments, z_base_top, "inner_zbase")
    inner_ztop = su.stadium_loop(half_span, r_inner_c, arc_segments, z_wall_top, "inner_ztop")

    # 1. Outer wall - full height, outward-facing.
    mesh = gh.loft([outer_z0, outer_ztop])

    # 2. Bottom cap - r_outer footprint minus the two through-bores,
    #    facing -Z. Two half-bridges, not ear-clip triangulation.
    outer_right_z0, outer_left_z0 = su.stadium_half_loops(half_span, r_outer_c, arc_segments, 0.0)
    for cx, half_loop, is_right in ((-half_span, outer_left_z0, False), (half_span, outer_right_z0, True)):
        bore_ring = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, 0.0))
        if is_right:
            bore_ring = su.rotate_loop_start(bore_ring, ring_segments // 2)
        mesh = mesh + gh.bridge(bore_ring, half_loop)

    # 3. Bore inner walls - through the whole stack at each shaft.
    for cx in shaft_xs:
        bore_bottom = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, 0.0))
        bore_top = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, z_boss_top))
        mesh = mesh + gh.flip_normals(gh.loft([bore_bottom, bore_top]))

    # 4. Wall's own inner (cavity) surface.
    mesh = mesh + gh.flip_normals(gh.loft([inner_zbase, inner_ztop]))

    # 5. Wall top rim - flat annulus, open in the middle, facing +Z.
    mesh = mesh + gh.cap_annulus(outer_ztop, inner_ztop)

    # 6. Floor - r_inner footprint minus the boss (or bore) holes, +Z.
    inner_right_zbase, inner_left_zbase = su.stadium_half_loops(half_span, r_inner_c, arc_segments, z_base_top)
    hole_radius = r_boss_c if has_boss else r_bore_c
    for cx, half_loop, is_right in ((-half_span, inner_left_zbase, False), (half_span, inner_right_zbase, True)):
        hole_ring = gh.circle(hole_radius, ring_segments, center=(cx, 0.0, z_base_top))
        if is_right:
            hole_ring = su.rotate_loop_start(hole_ring, ring_segments // 2)
        mesh = mesh + gh.bridge(hole_ring.reversed(), half_loop.reversed())

    # 7 & 8. Boss outer wall + boss top annulus (per shaft) - skipped
    #    entirely when boss_height_mm is ~0.
    if has_boss:
        for cx in shaft_xs:
            boss_bottom = gh.circle(r_boss_c, ring_segments, center=(cx, 0.0, z_base_top))
            boss_top = gh.circle(r_boss_c, ring_segments, center=(cx, 0.0, z_boss_top))
            mesh = mesh + gh.loft([boss_bottom, boss_top])

            boss_ring_at_top = gh.circle(r_boss_c, ring_segments, center=(cx, 0.0, z_boss_top))
            bore_ring_at_top = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, z_boss_top))
            mesh = mesh + gh.cap_annulus(boss_ring_at_top, bore_ring_at_top)

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Gearbox housing failed to build a closed shell: %s" % "; ".join(report.errors))

    derived = dict(
        r_inner=r_inner, r_outer=r_outer, wall_top_z=z_wall_top, segments=arc_segments,
        shaft_hole_dia_mm=shaft_hole_dia_mm, half_span=half_span, r_boss=r_boss, r_bore=r_bore,
    )
    return mesh, derived
