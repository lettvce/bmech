"""
Pure geometry core for hex_bolt.py - build_hex_bolt_mesh is plain Python +
geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted verbatim
(not rewritten) from hex_bolt.py so this exact logic can run outside
Blender too (the web configurator, via Pyodide) with a single source of
truth - hex_bolt.py itself now just imports build_hex_bolt_mesh from here
and wraps it in a bpy operator.

If this file changes, hex_bolt.py's own behavior changes too - there is
no separate copy of this logic left in that file. The web configurator's
own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py) - re-sync after editing this file.

See hex_bolt.py's own module docstring for the full design rationale
(helical_solid() thread, welded hex head, tip handling) - unchanged here,
just relocated.
"""

import sys
import os
from math import tan, radians, cos, sin, pi

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


# ── Thread geometry (duplicated verbatim from threaded_fastener.py, per
#    this family's own convention - see docs/bmech/fasteners/README.md) ──

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = max(truncation * pitch, 1e-4)
    rf    = max(2.0 * truncation * pitch, 1e-4)
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points outward (bolt ridge on outside of shaft) - CCW in
    the (r, dz) plane. A raw sweep of this is ALREADY a complete,
    self-contained threaded solid - no separate blank needed."""
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
    ]


def _hex_loop(across_flats, n, z):
    """Regular hexagon (flat-to-flat = across_flats) sampled at the n
    uniform bearings 2*pi*k/n - the same angles the shaft's own rings
    use, so the head's top face closes onto the shank's base ring as a
    plain cap_annulus() with no twist."""
    a = across_flats / 2.0
    pts = []
    for k in range(n):
        th = 2.0 * pi * k / n
        fold = ((th + pi / 6.0) % (pi / 3.0)) - pi / 6.0
        r = a / cos(fold)
        pts.append((r * cos(th), r * sin(th), z))
    return gh.Loop(pts, True, "hex")


def derive_hex_bolt(hex_across_flats_mm, thread_diameter_mm, pitch_mm,
                     flank_angle_deg, truncation, outer_compensation_mm,
                     fit_offset_mm, hex_length_mm, shank_enable,
                     shank_length_mm, thread_length_mm, tip_enable,
                     tip_length_mm):
    """Derived values a caller (Blender operator draw() or a web UI) wants
    to show before/without building the mesh - major/minor radius, thread
    depth, head wall thickness, total length. No geometry built here."""
    major_r = thread_diameter_mm / 2.0 + outer_compensation_mm - fit_offset_mm / 2.0
    minor_r, cf, fdz, depth = _thread_params(major_r, pitch_mm, flank_angle_deg, truncation)
    head_wall = hex_across_flats_mm / 2.0 - thread_diameter_mm / 2.0
    total_length = (hex_length_mm
                    + (shank_length_mm if shank_enable else 0.0)
                    + thread_length_mm
                    + (tip_length_mm if tip_enable else 0.0))
    return dict(major_r=major_r, minor_r=minor_r, cf=cf, fdz=fdz, depth=depth,
                head_wall=head_wall, total_length=total_length)


def build_hex_bolt_mesh(
        hex_length_mm=5.5, hex_across_flats_mm=13.0,
        shank_enable=True, shank_length_mm=10.0, shank_diameter_mm=8.0,
        thread_length_mm=20.0, thread_diameter_mm=8.0, pitch_mm=1.25,
        flank_angle_deg=60.0, truncation=0.125, resolution=3,
        outer_compensation_mm=0.0, fit_offset_mm=0.0,
        tip_enable=True, tip_length_mm=3.0, tip_diameter_mm=0.0):
    """Returns (mesh, derived_dict) - a geo_helpers Mesh, closed/watertight,
    plus the same derived values derive_hex_bolt() computes (so a caller
    doesn't have to call both). Raises ValueError for invalid geometry
    (truncation too high for the pitch, head too small for the thread)."""
    derived = derive_hex_bolt(
        hex_across_flats_mm, thread_diameter_mm, pitch_mm, flank_angle_deg,
        truncation, outer_compensation_mm, fit_offset_mm, hex_length_mm,
        shank_enable, shank_length_mm, thread_length_mm, tip_enable, tip_length_mm)
    major_r, minor_r, cf, fdz = derived["major_r"], derived["minor_r"], derived["cf"], derived["fdz"]
    head_wall = derived["head_wall"]

    if fdz <= 0 or head_wall <= 0:
        raise ValueError("Invalid geometry - check truncation and head size vs thread diameter")

    n = 12 * resolution

    # ── Thread - a genuinely SOLID threaded section, built first because
    #    its own length quantizes to whole ring steps and everything
    #    above it keys off where it actually ends. Both ends UNCAPPED -
    #    their flat minor_r rings are the weld interface to the shank
    #    below and the tip above.
    prof = _external_profile(major_r, minor_r, cf, fdz)
    profile_loop = gh.Loop([(r, 0.0, dz) for r, dz in prof], True, "thread")
    thread_mesh, thread_start, thread_end = gh.helical_solid(
        profile_loop, pitch_mm, thread_length_mm, n, cap_start=False, cap_end=False)

    # ── Main wall: [shank] -> step -> (thread welds in here) -> [tip pin]
    z = hex_length_mm
    r0 = shank_diameter_mm / 2.0 if shank_enable else minor_r

    apothem = hex_across_flats_mm / 2.0
    if r0 > apothem - 0.05:
        r0 = apothem - 0.05
    bottom_ring = gh.circle(r0, n, center=(0.0, 0.0, z))

    # ── Head - WELDED to the shaft (same n uniform bearings)
    hex_bot = _hex_loop(hex_across_flats_mm, n, 0.0)
    hex_top = _hex_loop(hex_across_flats_mm, n, z)
    head = (gh.cap(hex_bot, fill_mode="ngon", reverse=True)
            + gh.bridge(hex_bot, hex_top)
            + gh.cap_annulus(hex_top, bottom_ring, reverse=False))

    wall = gh.Mesh()
    prev_ring = bottom_ring

    if shank_enable:
        z += shank_length_mm
        shank_top_ring = gh.circle(r0, n, center=(0.0, 0.0, z))
        wall = wall + gh.bridge(prev_ring, shank_top_ring)
        prev_ring = shank_top_ring
    thread_z0 = z

    if abs(r0 - minor_r) > 1e-6:
        step_ring = gh.circle(minor_r, n, center=(0.0, 0.0, thread_z0))
        if minor_r > r0:
            wall = wall + gh.cap_annulus(step_ring, prev_ring, reverse=True)
        else:
            wall = wall + gh.cap_annulus(prev_ring, step_ring, reverse=False)
        prev_ring = step_ring

    thread_mesh = gh.translate_mesh(thread_mesh, (0.0, 0.0, thread_z0))
    thread_end_z = thread_z0 + thread_end.verts[0][2]
    prev_ring = gh.translate(thread_end, (0.0, 0.0, thread_z0))
    z = thread_end_z

    if tip_enable:
        tip_r = tip_diameter_mm / 2.0
        max_tip_r = max(minor_r - 1e-6, 0.0)
        if tip_r > max_tip_r:
            tip_r = max_tip_r
        if tip_r <= 1e-6:
            wall = wall + gh.cap(prev_ring, fill_mode="fan")
        else:
            tip_start_ring = gh.circle(tip_r, n, center=(0.0, 0.0, z))
            if tip_r > minor_r:
                wall = wall + gh.cap_annulus(tip_start_ring, prev_ring, reverse=True)
            else:
                wall = wall + gh.cap_annulus(prev_ring, tip_start_ring, reverse=False)
            z += tip_length_mm
            tip_end_ring = gh.circle(tip_r, n, center=(0.0, 0.0, z))
            wall = wall + gh.bridge(tip_start_ring, tip_end_ring)
            wall = wall + gh.cap(tip_end_ring, fill_mode="fan")
    else:
        wall = wall + gh.cap(prev_ring, fill_mode="fan")

    mesh = head + wall + thread_mesh

    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Hex bolt failed to build a closed shell: %s" % "; ".join(report.errors))

    derived["total_length_actual"] = z
    return mesh, derived
