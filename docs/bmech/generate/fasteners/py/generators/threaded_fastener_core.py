"""
Pure geometry core for threaded_fastener.py - build_threaded_fastener_mesh
is plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim from threaded_fastener.py for the web configurator
(Pyodide) - see that file's own module docstring for the four thread_type
x operation modes this raw helix is meant to be boolean'd into afterward.
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
    """Return (minor_r, crest_flat, flank_dz, thread_depth)."""
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = max(truncation * pitch, 1e-4)
    rf    = max(2.0 * truncation * pitch, 1e-4)
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points outward (bolt ridge on outside of shaft)."""
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
    ]


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward (nut ridge on inside of bore)."""
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
    ]


def derive_threaded_fastener(diameter_mm, pitch_mm, flank_angle_deg, truncation,
                              thread_type, operation, outer_compensation_mm,
                              inner_compensation_mm, fit_offset_mm):
    major_r = diameter_mm / 2.0
    if thread_type == 'EXTERNAL':
        major_r -= fit_offset_mm / 2.0
    else:
        major_r += fit_offset_mm / 2.0
    minor_r, cf, fdz, depth = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    if thread_type == 'EXTERNAL' and operation == 'ADDITIVE':
        major_r += outer_compensation_mm
        minor_r  = major_r - depth
    elif operation == 'SUBTRACTIVE' or (thread_type == 'INTERNAL' and operation == 'ADDITIVE'):
        major_r += inner_compensation_mm
        minor_r  = major_r - depth
    return dict(major_r=major_r, minor_r=minor_r, cf=cf, fdz=fdz, depth=depth)


def build_threaded_fastener_mesh(
        diameter_mm=8.0, pitch_mm=1.25, flank_angle_deg=60.0, truncation=0.125,
        height_mm=12.0, resolution=3, thread_type='EXTERNAL', operation='ADDITIVE',
        outer_compensation_mm=0.0, inner_compensation_mm=0.0, fit_offset_mm=0.0):
    """Returns (mesh, derived_dict) - a raw helical thread solid, closed at
    both ends, for the caller to boolean into a bolt/nut/tapped-hole
    afterward (this generator never did that itself, even before the
    boolean-free rewrite). Raises ValueError if truncation leaves no room
    for flanks at this pitch."""
    derived = derive_threaded_fastener(
        diameter_mm, pitch_mm, flank_angle_deg, truncation, thread_type, operation,
        outer_compensation_mm, inner_compensation_mm, fit_offset_mm)
    major_r, minor_r, cf, fdz = derived["major_r"], derived["minor_r"], derived["cf"], derived["fdz"]
    if fdz <= 0:
        raise ValueError("Truncation too high - no room for flanks at this pitch")

    using_external_profile = (thread_type == 'EXTERNAL') == (operation == 'ADDITIVE')
    if using_external_profile:
        prof = _external_profile(major_r, minor_r, cf, fdz)
    else:
        prof = _internal_profile(major_r, minor_r, cf, fdz)

    n = 12 * resolution
    profile_loop = gh.Loop([(r, 0.0, dz) for r, dz in prof], True, "thread")
    if not using_external_profile:
        profile_loop = profile_loop.reversed()
    mesh = gh.helical_sweep(profile_loop, pitch_mm, height_mm, n)
    start_ring = gh.Loop(mesh.verts[:4], True, "start")
    end_ring = gh.Loop(mesh.verts[-4:], True, "end")
    mesh = (mesh
            + gh.cap(start_ring, fill_mode="ngon", reverse=False)
            + gh.cap(end_ring, fill_mode="ngon", reverse=True))

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Threaded fastener failed to build a closed shell: %s" % "; ".join(report.errors))
    return mesh, derived
