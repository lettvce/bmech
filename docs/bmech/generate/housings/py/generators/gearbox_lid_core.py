"""
Pure geometry core for gearbox_lid.py - build_gearbox_lid_mesh is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim from gearbox_lid.py for the web configurator (Pyodide) - see
that file's own module docstring for the full design rationale (the
friction-fit skirt, the six-piece construction, the shared half-bridge
technique with gearbox_housing.py) this reproduces unchanged.

Unlike the Blender operator (which reads center_distance_mm/r_outer_mm/
shaft_hole_dia_mm/segments off a picked housing object's own stamped
custom properties via Match Target), build_gearbox_lid_mesh takes those
same four values as direct kwargs - there's no live object to pick from
here, so a caller (the web page, or gearbox_housing_core.py's own
derived dict) passes them straight through instead.
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


# ── Derived values + top-level build ────────────────────────────────────

def derive_gearbox_lid(center_distance_mm, r_outer_mm, shaft_hole_dia_mm, skirt_interference_mm):
    """Pure derivation, matching gearbox_lid.py's own draw() logic - the
    two inline error warnings. Never raises."""
    return dict(
        skirt_interference_too_large=(skirt_interference_mm >= r_outer_mm),
        shafts_overlap=(center_distance_mm < shaft_hole_dia_mm),
    )


def build_gearbox_lid_mesh(center_distance_mm=30.0, r_outer_mm=14.3, shaft_hole_dia_mm=5.0,
                            lid_thickness_mm=3.0, skirt_interference_mm=0.15, skirt_thickness_mm=2.0,
                            skirt_depth_mm=5.0, segments=32, internal_compensation_mm=0.075,
                            external_compensation_mm=0.05):
    """Returns (mesh, derived_dict) - a closed, single-solid friction-fit
    gearbox lid. Raises ValueError for invalid geometry (skirt
    interference too large for the housing radius, center distance
    smaller than the shaft holes, or a failed mesh validation)."""
    half_span = center_distance_mm / 2.0
    r_outer_housing = r_outer_mm

    r_skirt_inner = r_outer_housing - skirt_interference_mm
    if r_skirt_inner <= 0.0:
        raise ValueError("Skirt Interference is too large for this Housing OD Radius")
    r_skirt_outer = r_skirt_inner + skirt_thickness_mm
    r_bore = shaft_hole_dia_mm / 2.0

    if center_distance_mm < shaft_hole_dia_mm:
        raise ValueError("Center Distance is smaller than the shaft holes - they'd overlap")

    # FDM compensation - applied to SEPARATE "used for geometry" variables,
    # after the nominal skirt_interference_mm/skirt_thickness_mm math
    # above, so those two dials keep meaning exactly what they say
    # against the as-designed target.
    r_skirt_inner_c = r_skirt_inner + internal_compensation_mm
    r_skirt_outer_c = r_skirt_outer + external_compensation_mm
    r_bore_c = r_bore + internal_compensation_mm

    lid_thickness = lid_thickness_mm
    skirt_depth = skirt_depth_mm
    arc_segments = segments
    shaft_xs = (-half_span, half_span)
    ring_segments = arc_segments + 3

    outer_zbot = su.stadium_loop(half_span, r_skirt_outer_c, arc_segments, -skirt_depth, "outer_zbot")
    outer_ztop = su.stadium_loop(half_span, r_skirt_outer_c, arc_segments, lid_thickness, "outer_ztop")
    skirt_inner_zbot = su.stadium_loop(half_span, r_skirt_inner_c, arc_segments, -skirt_depth, "skirt_inner_zbot")
    skirt_inner_z0 = su.stadium_loop(half_span, r_skirt_inner_c, arc_segments, 0.0, "skirt_inner_z0")

    # 1. Outer wall - constant r_skirt_outer radius, full height.
    mesh = gh.loft([outer_zbot, outer_ztop])

    # 2. Cap top - r_skirt_outer footprint minus the two shaft bores, +Z.
    outer_right_ztop, outer_left_ztop = su.stadium_half_loops(half_span, r_skirt_outer_c, arc_segments, lid_thickness)
    for cx, half_loop, is_right in ((-half_span, outer_left_ztop, False), (half_span, outer_right_ztop, True)):
        bore_ring = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, lid_thickness))
        if is_right:
            bore_ring = su.rotate_loop_start(bore_ring, ring_segments // 2)
        mesh = mesh + gh.bridge(bore_ring.reversed(), half_loop.reversed())

    # 3. Skirt bottom rim - the skirt wall's own lowest edge, facing -Z.
    mesh = mesh + gh.cap_annulus(outer_zbot, skirt_inner_zbot, reverse=True)

    # 4. Skirt inner wall - faces INWARD (this surface grips the
    #    housing's own exterior wall).
    mesh = mesh + gh.flip_normals(gh.loft([skirt_inner_zbot, skirt_inner_z0]))

    # 5. Cap underside - r_skirt_inner footprint minus the two shaft
    #    bores, facing -Z.
    skirt_inner_right_z0, skirt_inner_left_z0 = su.stadium_half_loops(half_span, r_skirt_inner_c, arc_segments, 0.0)
    for cx, half_loop, is_right in ((-half_span, skirt_inner_left_z0, False), (half_span, skirt_inner_right_z0, True)):
        bore_ring = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, 0.0))
        if is_right:
            bore_ring = su.rotate_loop_start(bore_ring, ring_segments // 2)
        mesh = mesh + gh.bridge(bore_ring, half_loop)

    # 6. Bore inner walls (per shaft) - through the cap's own thickness only.
    for cx in shaft_xs:
        bore_bottom = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, 0.0))
        bore_top = gh.circle(r_bore_c, ring_segments, center=(cx, 0.0, lid_thickness))
        mesh = mesh + gh.flip_normals(gh.loft([bore_bottom, bore_top]))

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Gearbox lid failed to build a closed shell: %s" % "; ".join(report.errors))

    derived = dict(r_skirt_inner=r_skirt_inner, r_skirt_outer=r_skirt_outer, half_span=half_span)
    return mesh, derived
