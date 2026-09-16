"""
Pure geometry core for cluster_gear.py - build_cluster_gear_mesh is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim from cluster_gear.py for the web configurator (Pyodide) - see
that file's own module docstring for the shoulder/project_onto_loop
technique this reproduces unchanged.

Imports ADDENDUM_COEFF/DEDENDUM_COEFF/build_gear_profile/
max_pressure_angle_deg from spur_gear_core.py directly (not from
spur_gear.py, which imports bpy) - same sibling-core-importing pattern
threaded_jar_set_core.py uses in the fasteners family.
"""

import sys
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from spur_gear_core import ADDENDUM_COEFF, DEDENDUM_COEFF, build_gear_profile, max_pressure_angle_deg
import geo_helpers as gh


def build_cluster_mesh(bottom_profile, top_profile, width_bottom, width_top, bore_r):
    """bottom_profile/top_profile: lists of (x, y) from
    spur_gear_core.build_gear_profile(). Returns a geo_helpers Mesh - see
    module docstring for why every ring in this construction shares
    bottom_profile's own point count and bearings."""
    bottom_loop = gh.Loop([(x, y, 0.0) for x, y in bottom_profile], True, "bottom")
    top_native  = gh.Loop([(x, y, 0.0) for x, y in top_profile], True, "top_native")
    top_loop    = gh.project_onto_loop(bottom_loop, top_native, center=(0.0, 0.0, 0.0))

    z_shoulder = width_bottom
    z_top      = width_bottom + width_top

    bottom_z0 = bottom_loop
    bottom_z1 = gh.translate(bottom_loop, (0.0, 0.0, z_shoulder))
    top_z0    = gh.translate(top_loop, (0.0, 0.0, z_shoulder))
    top_z1    = gh.translate(top_loop, (0.0, 0.0, z_top))

    mesh = (gh.bridge(bottom_z0, bottom_z1)
            + gh.cap_annulus(bottom_z1, top_z0, reverse=False)
            + gh.bridge(top_z0, top_z1))

    if bore_r > 0.0:
        hole_bot = gh.project_radial(bottom_z0, bore_r, center=(0.0, 0.0, 0.0))
        hole_top = gh.project_radial(top_z1, bore_r, center=(0.0, 0.0, z_top))
        mesh = (mesh
                + gh.cap_annulus(bottom_z0, hole_bot, reverse=True)
                + gh.cap_annulus(top_z1, hole_top, reverse=False)
                + gh.bridge(hole_top, hole_bot))
    else:
        mesh = (mesh
                + gh.cap(bottom_z0, fill_mode="fan", reverse=True)
                + gh.cap(top_z1, fill_mode="fan"))

    gh.weld_duplicates(mesh, tolerance=1e-5)
    return mesh


def derive_cluster_gear(module, bottom_teeth, top_teeth, pressure_angle_deg,
                         bore_enable, axle_hole_mm, axle_compensation_mm):
    """Pure derivation, matching cluster_gear.py's own _compute()/draw()
    logic - clamps pressure_angle_deg to the tighter of both tooth
    counts' own ceilings. Never raises."""
    pa_max = min(max_pressure_angle_deg(t, ADDENDUM_COEFF) for t in (bottom_teeth, top_teeth))
    pressure_angle_deg = min(pressure_angle_deg, pa_max)

    bore_r    = (axle_hole_mm / 2.0 + axle_compensation_mm) if bore_enable else 0.0
    bottom_od = module * (bottom_teeth + 2 * ADDENDUM_COEFF)
    top_od    = module * (top_teeth    + 2 * ADDENDUM_COEFF)
    bottom_ded_r = module * (bottom_teeth / 2.0 - DEDENDUM_COEFF)
    min_ded_r = min(module * (t / 2.0 - DEDENDUM_COEFF) for t in (bottom_teeth, top_teeth))

    return dict(
        pressure_angle_deg=pressure_angle_deg, pa_max=pa_max, bore_r=bore_r,
        bottom_od=bottom_od, top_od=top_od, bottom_ded_r=bottom_ded_r, min_ded_r=min_ded_r,
    )


def build_cluster_gear_mesh(module=1.0, bottom_teeth=36, top_teeth=12, pressure_angle_deg=20.0,
                             width_bottom=6.0, width_top=6.0, bore_enable=True,
                             axle_hole_mm=5.0, axle_compensation_mm=0.2):
    """Returns (mesh, derived_dict) - a closed, single-solid cluster gear
    (two involute gears on one shared axle). Raises ValueError for
    invalid geometry (axle hole too large for the smaller gear, or the
    top gear's OD too large to fit inside the bottom gear's dedendum)."""
    derived = derive_cluster_gear(module, bottom_teeth, top_teeth, pressure_angle_deg,
                                   bore_enable, axle_hole_mm, axle_compensation_mm)
    bore_r, top_od, bottom_ded_r, min_ded_r = (
        derived["bore_r"], derived["top_od"], derived["bottom_ded_r"], derived["min_ded_r"])

    if bore_r > 0 and bore_r >= min_ded_r:
        raise ValueError("Axle hole too large for the smallest gear")
    if top_od >= bottom_ded_r * 2.0:
        raise ValueError("Top gear OD too large - must fit inside bottom gear's dedendum")

    bottom_prof = build_gear_profile(module, bottom_teeth, derived["pressure_angle_deg"])
    top_prof    = build_gear_profile(module, top_teeth, derived["pressure_angle_deg"])

    mesh = build_cluster_mesh(bottom_prof, top_prof, width_bottom, width_top, bore_r)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Cluster gear failed to build a closed shell: %s" % "; ".join(report.errors))

    derived["total_h"] = width_bottom + width_top
    return mesh, derived
