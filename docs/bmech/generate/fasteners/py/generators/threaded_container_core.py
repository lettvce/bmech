"""
Pure geometry core for threaded_container.py - build_threaded_container_mesh
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from threaded_container.py for the web configurator
(Pyodide) - see that file's own module docstring for the single-solid
"thread welded as a section of the wall" construction this reproduces
unchanged (thread built first, uncapped at both ends, its two flat
minor_r rings are the weld interface to the plain-wall lathe() surfaces
above and below it).
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
    """Smallest `truncation` that keeps thread depth <= max_depth for a
    given pitch/flank_angle. See threaded_container.py's own docstring
    for the closed-form derivation. Returns the RAW (unclamped) result -
    the caller checks whether it exceeds 0.3 (no valid truncation can
    achieve the requested max_depth) before clamping it for actual use."""
    ha = max(radians(flank_deg / 2.0), radians(0.5))
    return (1.0 - 2.0 * max_depth * tan(ha) / pitch) / 3.0


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points outward (thread ridge on the outside of the wall)."""
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
    ]


def derive_threaded_container_raw(thread_diameter_mm, wall_thickness_mm, pitch_mm,
                                   flank_angle_deg, truncation, outer_compensation_mm,
                                   fit_offset_mm):
    """Pure geometry from the CURRENT (possibly not-yet-clamped) property
    values - never raises, matching threaded_container.py's own _derived(),
    which draw() calls every redraw (including on invalid/pre-clamp values)
    just to display numbers and info labels."""
    outer_r = thread_diameter_mm / 2.0 + outer_compensation_mm - fit_offset_mm / 2.0
    inner_r = outer_r - wall_thickness_mm
    floor_z = wall_thickness_mm
    minor_r, cf, fdz, depth = _thread_params(outer_r, pitch_mm, flank_angle_deg, truncation)
    return dict(outer_r=outer_r, inner_r=inner_r, floor_z=floor_z, minor_r=minor_r,
                cf=cf, fdz=fdz, depth=depth)


def derive_threaded_container(thread_diameter_mm, wall_thickness_mm, height_mm,
                               thread_length_mm, pitch_mm, flank_angle_deg, truncation,
                               resolution, outer_compensation_mm, fit_offset_mm):
    """Applies the same silent-clamp-or-raise logic as threaded_container.py's
    own _clamp(), then returns the fully-derived geometry for the (possibly
    clamped) final values. Raises ValueError for the genuinely irreconcilable
    cases _clamp() itself can't paper over."""
    outer_r = thread_diameter_mm / 2.0 + outer_compensation_mm - fit_offset_mm / 2.0

    # 1. wall_thickness_mm must leave a real interior cavity.
    max_wall = min(outer_r, height_mm) - 0.1
    if max_wall <= 0:
        raise ValueError("OD/height too small for any positive wall thickness")
    if wall_thickness_mm > max_wall:
        wall_thickness_mm = max_wall

    # 2. truncation must keep thread depth safely under the wall thickness.
    max_depth = wall_thickness_mm * 0.7
    min_trunc = _min_truncation_for_max_depth(pitch_mm, flank_angle_deg, max_depth)
    if min_trunc > 0.3:
        raise ValueError("Pitch too coarse for this wall thickness even at max truncation")
    if truncation < min_trunc:
        truncation = min_trunc

    inner_r = outer_r - wall_thickness_mm
    floor_z = wall_thickness_mm
    minor_r, cf, fdz, depth = _thread_params(outer_r, pitch_mm, flank_angle_deg, truncation)

    # 3. thread_length_mm has to fit between the floor and the rim, with
    # 0.5mm to spare, and be long enough for at least one turn-to-turn
    # root band.
    n = 12 * resolution
    span = 2.0 * fdz + cf
    min_thread_length = pitch_mm * (n + 1) / n + span
    max_thread_length = height_mm - floor_z - 0.5
    if max_thread_length < min_thread_length:
        raise ValueError("Container too short for a full thread turn at this pitch")
    if thread_length_mm > max_thread_length:
        thread_length_mm = max_thread_length
    if thread_length_mm < min_thread_length:
        thread_length_mm = min_thread_length

    return dict(outer_r=outer_r, inner_r=inner_r, floor_z=floor_z, minor_r=minor_r,
                cf=cf, fdz=fdz, depth=depth, wall_thickness_mm=wall_thickness_mm,
                truncation=truncation, thread_length_mm=thread_length_mm, n=n)


def build_threaded_container_mesh(
        thread_diameter_mm=60.0, wall_thickness_mm=3.0, height_mm=60.0,
        thread_length_mm=10.0, pitch_mm=4.0, flank_angle_deg=90.0, truncation=0.0,
        resolution=5, outer_compensation_mm=0.0, fit_offset_mm=0.0):
    """Returns (mesh, derived_dict) - a closed, single-solid screw-top
    container whose thread is welded directly into the wall (no separate
    ribbon, no core cylinder - see module docstring). Raises ValueError for
    invalid/irreconcilable geometry."""
    derived = derive_threaded_container(
        thread_diameter_mm, wall_thickness_mm, height_mm, thread_length_mm,
        pitch_mm, flank_angle_deg, truncation, resolution,
        outer_compensation_mm, fit_offset_mm)
    outer_r, inner_r, floor_z = derived["outer_r"], derived["inner_r"], derived["floor_z"]
    minor_r, cf, fdz = derived["minor_r"], derived["cf"], derived["fdz"]
    thread_length_mm = derived["thread_length_mm"]
    n = derived["n"]

    # ── Thread, built FIRST so its quantized height is known before
    #    anything is positioned. Uncapped at both ends: the two flat
    #    minor_r rings left open are the weld interface to the wall
    #    below and the rim above (see module docstring).
    prof = _external_profile(outer_r, minor_r, cf, fdz)
    profile_loop = gh.Loop([(r, 0.0, dz) for r, dz in prof], True, "thread")
    thread_mesh, thread_start, thread_end = gh.helical_solid(
        profile_loop, pitch_mm, thread_length_mm, n,
        cap_start=False, cap_end=False)
    thread_h = thread_end.verts[0][2] - thread_start.verts[0][2]

    # Placed so the thread's TOP lands exactly on height_mm, keeping the
    # rim where the user asked even though the thread's own length rounds
    # to whole ring steps.
    thread_z0 = height_mm - thread_h
    if thread_z0 <= floor_z:
        raise ValueError("Container too short for this thread length")
    thread_mesh = gh.translate_mesh(thread_mesh, (0.0, 0.0, thread_z0))

    # ── Lower body: floor pole -> out -> up the outer wall -> STEP IN to
    #    the thread's own root radius, ending on the identical ring the
    #    thread starts from, which fuses the two by coincidence.
    lower = gh.Loop([
        (0.0, 0.0, 0.0),
        (outer_r, 0.0, 0.0),
        (outer_r, 0.0, thread_z0),
        (minor_r, 0.0, thread_z0),
    ], False, "container_lower")

    # ── Upper body: from the thread's end ring in across the rim, then
    #    down the cavity wall to the inner floor pole.
    upper = gh.Loop([
        (minor_r, 0.0, height_mm),
        (inner_r, 0.0, height_mm),
        (inner_r, 0.0, floor_z),
        (0.0, 0.0, floor_z),
    ], False, "container_upper")

    mesh = (thread_mesh
            + gh.lathe(lower, axis="z", steps=n)
            + gh.lathe(upper, axis="z", steps=n))

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Threaded container failed to build a closed shell: %s"
                          % "; ".join(report.errors))

    return mesh, derived
