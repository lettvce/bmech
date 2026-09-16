"""
Pure geometry core for hex_nut.py - build_hex_nut_mesh is plain Python +
geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted verbatim
from hex_nut.py for the web configurator (Pyodide) - see that file's own
module docstring for the single-solid internal-thread technique (the
bore IS the thread surface, via one helical_solid() call with normals
flipped) this reproduces unchanged.
"""

import sys
import os
from math import cos, sin, tan, pi, radians

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


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """One tooth of an internal thread: root at major_r (the bore wall),
    crest reaching inward to minor_r."""
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
    ]


def _hex_loop(across_flats, n, z):
    """Regular hexagon sampled at the n uniform bearings the thread's own
    rings use, so cap_annulus() pairs them point-for-point without a
    twist."""
    a = across_flats / 2.0
    pts = []
    for k in range(n):
        th = 2.0 * pi * k / n
        fold = ((th + pi / 6.0) % (pi / 3.0)) - pi / 6.0
        r = a / cos(fold)
        pts.append((r * cos(th), r * sin(th), z))
    return gh.Loop(pts, True, "hex")


def derive_hex_nut(z_height_mm, across_flats_mm, thread_diameter_mm, pitch_mm,
                    flank_angle_deg, truncation, resolution, inner_compensation_mm,
                    fit_offset_mm):
    major_r = thread_diameter_mm / 2.0 + inner_compensation_mm + fit_offset_mm / 2.0
    minor_r, cf, fdz, depth = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    wall = across_flats_mm / 2.0 - major_r
    n = 12 * resolution
    span = 2.0 * fdz + cf
    # Same floor helical_solid() enforces internally: below one pitch plus
    # a tooth's axial span there is no room for a single turn-to-turn root
    # band.
    min_height = pitch_mm * (n + 1) / n + span
    clamped_height = max(z_height_mm, min_height)
    return dict(major_r=major_r, minor_r=minor_r, cf=cf, fdz=fdz, depth=depth,
                wall=wall, n=n, min_height=min_height, clamped_height=clamped_height)


def build_hex_nut_mesh(
        z_height_mm=6.5, across_flats_mm=13.0, thread_diameter_mm=8.0,
        pitch_mm=1.25, flank_angle_deg=60.0, truncation=0.125, resolution=3,
        inner_compensation_mm=0.0, fit_offset_mm=0.0):
    """Returns (mesh, derived_dict) - a closed, single-solid hex nut whose
    bore doubles as the thread surface. Height quantizes to whole ring
    steps (derived["z0"]/["z1"] report what was actually built). Raises
    ValueError for invalid geometry (truncation too high, or the hex too
    thin for the thread)."""
    derived = derive_hex_nut(z_height_mm, across_flats_mm, thread_diameter_mm,
                              pitch_mm, flank_angle_deg, truncation, resolution,
                              inner_compensation_mm, fit_offset_mm)
    major_r, minor_r, cf, fdz = derived["major_r"], derived["minor_r"], derived["cf"], derived["fdz"]
    wall, n = derived["wall"], derived["n"]

    if fdz <= 0 or wall <= 0:
        raise ValueError("Invalid geometry - check truncation and across-flats vs thread diameter")

    height = derived["clamped_height"]

    # ── Bore: the thread surface IS the bore, one helical_solid call.
    #    Built uncapped so both flat major_r rings stay open as the weld
    #    interface to the hex faces, then flipped so material reads as
    #    OUTSIDE it (a bore) rather than inside it (a thread cutter).
    prof = _internal_profile(major_r, minor_r, cf, fdz)
    profile_loop = gh.Loop([(r, 0.0, dz) for r, dz in prof], True, "thread")
    bore, bore_bot, bore_top = gh.helical_solid(
        profile_loop, pitch_mm, height, n, cap_start=False, cap_end=False)
    gh.flip_normals(bore)

    # ── Hex body, spanning exactly the bore's own quantized extent.
    z0 = bore_bot.verts[0][2]
    z1 = bore_top.verts[0][2]
    hex_bot = _hex_loop(across_flats_mm, n, z0)
    hex_top = _hex_loop(across_flats_mm, n, z1)

    mesh = (bore
            + gh.bridge(hex_bot, hex_top)
            + gh.cap_annulus(hex_bot, bore_bot, reverse=True)
            + gh.cap_annulus(hex_top, bore_top, reverse=False))

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Hex nut failed to build a closed shell: %s" % "; ".join(report.errors))

    derived["z0"] = z0
    derived["z1"] = z1
    derived["actual_height"] = z1 - z0
    return mesh, derived
