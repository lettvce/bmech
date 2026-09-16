"""
Pure geometry core for threaded_lid.py - build_threaded_lid_mesh is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim from threaded_lid.py for the web configurator (Pyodide) - see
that file's own module docstring for the single-solid "thread welded as a
section of the bore" construction (thread built first via helical_solid
with normals flipped into a bore, welded to a shell+ceiling lathe surface)
this reproduces unchanged.
"""

import sys
import os
from math import tan, radians

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


def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = max(truncation * pitch, 1e-4)
    rf    = max(2.0 * truncation * pitch, 1e-4)
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _min_truncation_for_max_depth(pitch, flank_deg, max_depth):
    """Smallest `truncation` that keeps thread depth <= max_depth. See
    threaded_container_core.py's copy of this function for the full
    derivation - identical formula, duplicated per this family's
    convention. Returns the RAW (unclamped) result."""
    ha = max(radians(flank_deg / 2.0), radians(0.5))
    return (1.0 - 2.0 * max_depth * tan(ha) / pitch) / 3.0


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward - the ADDITIVE ridge shape for an internal
    thread, wrapping its two equal-radius points at major_r (the ridge's
    attachment radius)."""
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
    ]


def derive_threaded_lid_raw(thread_diameter_mm, wall_thickness_mm, thread_length_mm,
                             guide_length_mm, pitch_mm, flank_angle_deg, truncation,
                             inner_compensation_mm, fit_offset_mm):
    """Pure geometry from the CURRENT (possibly not-yet-clamped) property
    values - never raises, matching threaded_lid.py's own _derived(), which
    draw() calls every redraw just to display numbers and info labels."""
    major_r = thread_diameter_mm / 2.0 + inner_compensation_mm + fit_offset_mm / 2.0
    outer_r = major_r + wall_thickness_mm
    height  = wall_thickness_mm + thread_length_mm + guide_length_mm
    minor_r, cf, fdz, depth = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    return dict(major_r=major_r, outer_r=outer_r, height=height,
                minor_r=minor_r, cf=cf, fdz=fdz, depth=depth)


def derive_threaded_lid(thread_diameter_mm, wall_thickness_mm, thread_length_mm,
                         guide_length_mm, pitch_mm, flank_angle_deg, truncation,
                         inner_compensation_mm, fit_offset_mm):
    """Applies the same silent-clamp-or-raise logic as threaded_lid.py's
    own _clamp(), then returns the fully-derived geometry for the
    (possibly clamped) final values. Raises ValueError for the one
    genuinely irreconcilable case _clamp() itself can't paper over."""
    major_r = thread_diameter_mm / 2.0 + inner_compensation_mm + fit_offset_mm / 2.0

    # 1. truncation must be high enough that the thread ridge's own
    # minor_r stays positive with a real margin.
    margin = max(1.0, major_r * 0.05)
    max_depth = major_r - margin
    min_trunc = _min_truncation_for_max_depth(pitch_mm, flank_angle_deg, max_depth)
    if min_trunc > 0.3:
        raise ValueError("Pitch too coarse for this thread diameter even at max truncation")
    if truncation < min_trunc:
        truncation = min_trunc

    # 2. guide_length_mm must leave a real, comfortably-thick unthreaded
    # gap between the thread's own end and the real mouth.
    clearance_margin = max(1.0, 0.25 * pitch_mm)
    if guide_length_mm < clearance_margin:
        guide_length_mm = clearance_margin

    # 3. thread_length_mm must exceed the thread profile's own axial span
    # by a real margin.
    minor_r, cf, fdz, depth = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    min_thread_length = (2.0 * fdz + cf) + 0.5 * pitch_mm
    if thread_length_mm < min_thread_length:
        thread_length_mm = min_thread_length

    outer_r = major_r + wall_thickness_mm
    height  = wall_thickness_mm + thread_length_mm + guide_length_mm

    return dict(major_r=major_r, outer_r=outer_r, height=height, minor_r=minor_r,
                cf=cf, fdz=fdz, depth=depth, truncation=truncation,
                guide_length_mm=guide_length_mm, thread_length_mm=thread_length_mm)


def build_threaded_lid_mesh(
        thread_diameter_mm=60.0, wall_thickness_mm=3.0, thread_length_mm=8.0,
        guide_length_mm=4.0, pitch_mm=4.0, flank_angle_deg=90.0, truncation=0.25,
        resolution=5, inner_compensation_mm=0.0, fit_offset_mm=0.0):
    """Returns (mesh, derived_dict) - a closed, single-solid screw-top lid
    whose thread is welded directly into the bore wall (see module
    docstring). Raises ValueError for invalid/irreconcilable geometry."""
    derived = derive_threaded_lid(
        thread_diameter_mm, wall_thickness_mm, thread_length_mm, guide_length_mm,
        pitch_mm, flank_angle_deg, truncation, inner_compensation_mm, fit_offset_mm)
    major_r, outer_r = derived["major_r"], derived["outer_r"]
    minor_r, cf, fdz = derived["minor_r"], derived["cf"], derived["fdz"]
    thread_length_mm = derived["thread_length_mm"]
    guide_length_mm = derived["guide_length_mm"]
    n = 12 * resolution

    # ── Thread, built FIRST so its quantized height is known before
    #    anything else is positioned. Uncapped at both ends so the flat
    #    major_r rings stay open as the weld interface, then flipped so
    #    the material reads as being OUTSIDE the thread surface (a bore)
    #    rather than inside it (a thread cutter) - see module docstring.
    prof = _internal_profile(major_r, minor_r, cf, fdz)
    profile_loop = gh.Loop([(r, 0.0, dz) for r, dz in prof], True, "thread")
    thread_mesh, thread_start, thread_end = gh.helical_solid(
        profile_loop, pitch_mm, thread_length_mm, n,
        cap_start=False, cap_end=False)
    gh.flip_normals(thread_mesh)
    thread_h = thread_end.verts[0][2] - thread_start.verts[0][2]

    # The thread starts right at the ceiling's underside and runs DOWN
    # the bore (this file's z grows downward from the ceiling), with the
    # unthreaded guide zone below it.
    thread_z0 = wall_thickness_mm
    thread_z1 = thread_z0 + thread_h
    height = thread_z1 + guide_length_mm
    thread_mesh = gh.translate_mesh(thread_mesh, (0.0, 0.0, thread_z0))

    # ── Shell: ceiling pole -> out -> down the outside -> in across the
    #    mouth rim -> back UP the plain guide bore, ending on the
    #    identical ring the thread's lower end sits on.
    shell = gh.Loop([
        (0.0, 0.0, 0.0),
        (outer_r, 0.0, 0.0),
        (outer_r, 0.0, height),
        (major_r, 0.0, height),
        (major_r, 0.0, thread_z1),
    ], False, "lid_shell")

    # ── Ceiling underside: a flat disc closing the thread's upper ring
    #    in to the axis.
    ceiling = gh.Loop([
        (major_r, 0.0, thread_z0),
        (0.0, 0.0, thread_z0),
    ], False, "lid_ceiling")

    mesh = (thread_mesh
            + gh.lathe(shell, axis="z", steps=n)
            + gh.lathe(ceiling, axis="z", steps=n))

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Threaded lid failed to build a closed shell: %s"
                          % "; ".join(report.errors))

    derived["height"] = height
    return mesh, derived
