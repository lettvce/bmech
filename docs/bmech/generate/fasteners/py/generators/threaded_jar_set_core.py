"""
Pure geometry core for threaded_jar_set.py - build_threaded_jar_set_meshes
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted for the web configurator (Pyodide).

Unlike the bpy operator (which duplicates threaded_container.py's/
threaded_lid.py's geometry-building code verbatim, per that file's own
docstring, specifically to avoid a nested bpy.ops call breaking its
single-undo-step redo panel), this core module has no undo steps to
protect - it's a plain function call, not a Blender operator - so it
reuses threaded_container_core.build_threaded_container_mesh and
threaded_lid_core.build_threaded_lid_mesh directly instead of
re-duplicating their thread-sweep/lathe/weld logic a third time. Only the
genuinely new cross-part logic - resolving ONE shared truncation that
satisfies both parts' own minimums simultaneously - is implemented here,
translated verbatim from threaded_jar_set.py's own _clamp().
"""

import sys
import os

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from threaded_container_core import _thread_params, _min_truncation_for_max_depth, build_threaded_container_mesh
from threaded_lid_core import build_threaded_lid_mesh


def derive_threaded_jar_set(thread_diameter_mm, pitch_mm, flank_angle_deg, truncation,
                             fit_offset_mm, resolution,
                             container_wall_thickness_mm, container_height_mm,
                             container_thread_length_mm, container_compensation_mm,
                             lid_wall_thickness_mm, lid_thread_length_mm,
                             lid_guide_length_mm, lid_compensation_mm):
    """Applies the same silent-clamp-or-raise logic as threaded_jar_set.py's
    own _clamp(): container-side clamp first (may raise truncation's own
    floor), then lid-side clamp reusing/further-raising that SAME shared
    truncation, so whichever part needs the higher minimum wins and both
    stay valid simultaneously. Raises ValueError for the three genuinely
    irreconcilable cases _clamp() itself can't paper over."""
    # ── Container side - see threaded_container_core.py's own
    #    derive_threaded_container() for the identical single-part logic.
    outer_r = thread_diameter_mm / 2.0 + container_compensation_mm - fit_offset_mm / 2.0
    max_wall = min(outer_r, container_height_mm) - 0.1
    if max_wall <= 0:
        raise ValueError("Container OD/height too small for any positive wall thickness")
    if container_wall_thickness_mm > max_wall:
        container_wall_thickness_mm = max_wall

    max_depth = container_wall_thickness_mm * 0.7
    min_trunc = _min_truncation_for_max_depth(pitch_mm, flank_angle_deg, max_depth)
    if min_trunc > 0.3:
        raise ValueError("Pitch too coarse for the container's wall thickness even at max truncation")
    if truncation < min_trunc:
        truncation = min_trunc

    floor_z = container_wall_thickness_mm
    n = 12 * resolution
    _, c_cf, c_fdz, _ = _thread_params(outer_r, pitch_mm, flank_angle_deg, truncation)
    c_span = 2.0 * c_fdz + c_cf
    c_min_thread = pitch_mm * (n + 1) / n + c_span
    max_thread_length = container_height_mm - floor_z - 0.5
    if max_thread_length < c_min_thread:
        raise ValueError("Container too short for a full thread turn at this pitch")
    if container_thread_length_mm > max_thread_length:
        container_thread_length_mm = max_thread_length
    if container_thread_length_mm < c_min_thread:
        container_thread_length_mm = c_min_thread

    # ── Lid side - see threaded_lid_core.py's own derive_threaded_lid()
    #    for the identical single-part logic. truncation may get pushed
    #    UP further here than the container-side pass alone required.
    major_r = thread_diameter_mm / 2.0 + lid_compensation_mm + fit_offset_mm / 2.0
    margin = max(1.0, major_r * 0.05)
    max_depth2 = major_r - margin
    min_trunc2 = _min_truncation_for_max_depth(pitch_mm, flank_angle_deg, max_depth2)
    if min_trunc2 > 0.3:
        raise ValueError("Pitch too coarse for this thread diameter even at max truncation")
    if truncation < min_trunc2:
        truncation = min_trunc2

    clearance_margin = max(1.0, 0.25 * pitch_mm)
    if lid_guide_length_mm < clearance_margin:
        lid_guide_length_mm = clearance_margin

    _, l_cf, l_fdz, _ = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    min_thread_length = (2.0 * l_fdz + l_cf) + 0.5 * pitch_mm
    if lid_thread_length_mm < min_thread_length:
        lid_thread_length_mm = min_thread_length

    return dict(
        truncation=truncation,
        container_wall_thickness_mm=container_wall_thickness_mm,
        container_thread_length_mm=container_thread_length_mm,
        lid_guide_length_mm=lid_guide_length_mm,
        lid_thread_length_mm=lid_thread_length_mm,
    )


def build_threaded_jar_set_meshes(
        thread_diameter_mm=60.0, pitch_mm=4.0, flank_angle_deg=90.0, truncation=0.0,
        fit_offset_mm=0.0, resolution=5,
        container_wall_thickness_mm=3.0, container_height_mm=60.0,
        container_thread_length_mm=10.0, container_compensation_mm=0.0,
        lid_wall_thickness_mm=3.0, lid_thread_length_mm=8.0, lid_guide_length_mm=4.0,
        lid_compensation_mm=0.0):
    """Returns (container_mesh, container_derived, lid_mesh, lid_derived) -
    a matching container+lid pair guaranteed to mate: same thread_diameter/
    pitch/flank_angle and ONE shared truncation resolved to whichever
    part's own minimum is larger (see derive_threaded_jar_set). Raises
    ValueError for irreconcilable geometry on either side."""
    clamped = derive_threaded_jar_set(
        thread_diameter_mm, pitch_mm, flank_angle_deg, truncation, fit_offset_mm, resolution,
        container_wall_thickness_mm, container_height_mm, container_thread_length_mm,
        container_compensation_mm, lid_wall_thickness_mm, lid_thread_length_mm,
        lid_guide_length_mm, lid_compensation_mm)

    container_mesh, container_derived = build_threaded_container_mesh(
        thread_diameter_mm=thread_diameter_mm,
        wall_thickness_mm=clamped["container_wall_thickness_mm"],
        height_mm=container_height_mm,
        thread_length_mm=clamped["container_thread_length_mm"],
        pitch_mm=pitch_mm, flank_angle_deg=flank_angle_deg, truncation=clamped["truncation"],
        resolution=resolution, outer_compensation_mm=container_compensation_mm,
        fit_offset_mm=fit_offset_mm)

    lid_mesh, lid_derived = build_threaded_lid_mesh(
        thread_diameter_mm=thread_diameter_mm, wall_thickness_mm=lid_wall_thickness_mm,
        thread_length_mm=clamped["lid_thread_length_mm"],
        guide_length_mm=clamped["lid_guide_length_mm"],
        pitch_mm=pitch_mm, flank_angle_deg=flank_angle_deg, truncation=clamped["truncation"],
        resolution=resolution, inner_compensation_mm=lid_compensation_mm,
        fit_offset_mm=fit_offset_mm)

    return container_mesh, container_derived, lid_mesh, lid_derived
