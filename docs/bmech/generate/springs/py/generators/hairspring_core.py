"""
Pure geometry core for hairspring.py - build_hairspring_solid is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted
verbatim from hairspring.py for the web configurator (Pyodide) - see that
file's own module docstring for why sweeping via geo_helpers.extrude_path
is exact (not approximate) for this planar centerline.
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


def generate_centerline_points(r_inner_mm, gap_mm, turns, resolution):
    """Return list of (x, y, z) tuples for the spiral centerline.

    r(θ) = r_inner + (gap / 2π) × θ,  θ ∈ [0, 2π·turns]
    """
    N = max(int(round(resolution * turns)) + 1, 2)
    two_pi_turns = 2.0 * math.pi * turns
    gap_over_2pi = gap_mm / (2.0 * math.pi)
    points = []
    for i in range(N):
        t     = i / (N - 1)
        theta = t * two_pi_turns
        r     = r_inner_mm + gap_over_2pi * theta
        points.append((r * math.cos(theta), r * math.sin(theta), 0.0))
    return points


def build_hairspring_mesh(points, strip_thickness_mm, strip_width_mm):
    """Sweep a rectangular cross-section along the spiral centerline via
    geo_helpers.extrude_path - see hairspring.py's own docstring for why
    this is exact (not approximate) for a planar centerline. Raises
    ValueError if the resulting mesh fails validation."""
    profile = gh.rectangle(strip_thickness_mm, strip_width_mm)
    path = gh.Loop([tuple(p) for p in points], False, "centerline")
    swept = gh.extrude_path(profile, path)

    n = len(profile.verts)
    m = len(path.verts)
    start_ring = gh.Loop(swept.verts[0:n], True, "start")
    end_ring = gh.Loop(swept.verts[(m - 1) * n: m * n], True, "end")
    mesh = swept + gh.cap(start_ring, fill_mode="ngon", reverse=True) + gh.cap(end_ring, fill_mode="ngon")

    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Hairspring mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


# ── Derived values + top-level build ────────────────────────────────────

def derive_hairspring(input_mode, r_inner, turns, gap, r_outer):
    """Pure derivation, matching hairspring.py's own draw() logic for
    both input modes. Never raises - reports an outer_r/gap_mm pair plus
    validity flags for MODE_B's own reachable error states (informational
    only - build_hairspring_solid still clamps the derived gap to the
    0.1mm floor, same as the Blender operator's own draw()-only warning)."""
    if input_mode == 'MODE_A':
        return dict(gap_mm=gap, outer_r=r_inner + gap * turns,
                    mode_b_valid=True, mode_b_gap_too_small=False)

    mode_b_valid = r_outer > r_inner
    derived_gap = ((r_outer - r_inner) / turns) if (turns > 0 and mode_b_valid) else 0.0
    gap_mm = max(derived_gap, 0.1)
    return dict(gap_mm=gap_mm, outer_r=r_outer, mode_b_valid=mode_b_valid,
                mode_b_gap_too_small=(mode_b_valid and derived_gap < 0.1))


def build_hairspring_solid(input_mode='MODE_A', r_inner=10.0, turns=5.0, gap=2.0,
                            r_outer=20.0, strip_width=1.0, strip_thickness=0.4,
                            resolution=128):
    """Returns (mesh, derived_dict) - a closed, single-solid ribbon
    hairspring. Raises ValueError for invalid geometry (MODE_B with
    outer radius not exceeding inner radius)."""
    derived = derive_hairspring(input_mode, r_inner, turns, gap, r_outer)
    if input_mode == 'MODE_B' and not derived["mode_b_valid"]:
        raise ValueError("Outer radius must exceed inner radius")

    points = generate_centerline_points(r_inner, derived["gap_mm"], turns, resolution)
    mesh = build_hairspring_mesh(points, strip_thickness, strip_width)
    return mesh, derived
