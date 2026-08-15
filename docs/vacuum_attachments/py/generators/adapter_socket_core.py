"""
Pure geometry core for adapter_socket.py - build_adapter_socket_mesh is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim (not rewritten) from adapter_socket.py so this exact
logic can run outside Blender too (the web configurator, via Pyodide)
with a single source of truth - adapter_socket.py itself now just
imports build_adapter_socket_mesh from here and wraps it in a bpy
object/operator.

If this file changes, adapter_socket.py's own behavior changes too -
there is no separate copy of this logic left in that file. The web
configurator's own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

Parametric friction-fit receiving SOCKET for vacuum attachments - the
female counterpart to adapter_taper.py's male plug.

Some vacuums put the tapered male spigot on the VACUUM SIDE (a stub
protruding from the hose/body, narrow at its own free tip and widening
toward its base where it's mounted) rather than on the attachment side.
For that configuration the attachment needs to be the RECEIVING socket,
not another male plug - adapter_taper.py's own shape (solid outer taper)
is backwards for this case.

The mirror-image rule: our socket's BORE should be widest at its own
entrance (so it can slip over the spigot's narrow tip) and narrow as you
go deeper (away from that opening), matching the spigot's own base
getting wider the further it is from its own free tip. At full
insertion, the deepest point of our bore contacts the spigot's tip; the
shallowest point (our own entrance) contacts the spigot's base - so
"ID gets smaller further from the opening" is the correct direction
here, the opposite of adapter_taper.py's OUTER taper direction.

Orientation: the neck's own connecting ring sits at the local origin
facing +Z (matching every mouth generator's own neck-stub convention);
the whole part extends downward from there, socket entrance facing -Z.

Shape, neck-end to socket entrance (four rings, three sections):
  1. Straight neck stub - identical to adapter_taper.py's (25.4mm ID /
     1.2mm wall by default), same shared attachment-interface convention
     every mouth generator's own neck stub already uses.
  2. Transition cone - the BORE expands from the neck stub's ID out to
     the socket's own narrow (deep) engagement radius. Outer wall tracks
     the bore at a constant wall_mm offset throughout this whole part -
     there's no separate "outer taper direction" to reason about here,
     it just follows the bore.
  3. Engagement taper - the actual friction surface, a SHALLOW cone
     (taper_angle_deg, default 3 degrees, matching the <5 degree figure
     used elsewhere in this project) that continues expanding from the
     narrow/deep point to the wide entrance point, straddling the target
     spigot's own OD at the midpoint of that range.

Bore stays open at both ends, same annular-cap convention as every other
generator in this project - the region inside each inner ring is never
meshed, that absence IS the open bore.

target_spigot_od_mm/taper_angle_deg/engagement_len_mm are provisional,
not measured off a real vacuum - recalibrate once a real spigot gets
calipered. The 32/35mm presets are carried over from this project's own
earlier household-hose-ID research; they haven't been independently
confirmed as spigot-OD figures specifically, they're just the most
reasonable starting point available without new measurements.

The bore itself is built BORE_DIAMETER_COMPENSATION_MM larger than
target_spigot_od_mm, not that value directly - FDM printers consistently
print internal bores a bit undersized (extrusion overlap eats into the
cavity), so a bore modeled exactly at the target diameter tends to print
tight. See that constant's own comment for the figure and reasoning.
This is one-sided - adapter_taper.py's own OUTER plug surface gets no
equivalent compensation, since an outer surface trends the other way
(slightly oversized), not the same problem.
"""

import sys
import os
import math

# Pre-set to this project's own folder, where geo_helpers.py already
# lives - makes pasting this into a blank Text Editor tab in an unsaved
# .blend work with no manual step. Only change this if the project
# folder ever moves. When vendored into the web configurator, geo_helpers.py
# sits alongside this file, so the plain sys.path fallback below finds it
# there too without needing GEO_HELPERS_DIR set at all.
GEO_HELPERS_DIR = r"C:\Users\Safu\OneDrive\Desktop\cuum"


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


# FDM printers consistently print internal bores/holes a bit UNDERSIZED
# relative to the modeled dimension (extrusion overlap along a hole's own
# wall eats into the cavity) - the opposite direction from an outer
# surface like adapter_taper.py's own plug, which tends to print slightly
# OVERSIZED instead, so this compensation is deliberately one-sided and
# only applied here, not there. 0.15mm on diameter is a reasonable single
# fixed compensation across the expected range of target spigot sizes
# this project deals with (not itself printer/slicer-specific tuning -
# recalibrate if a particular printer's own hole tolerance differs
# noticeably once real prints are measured).
BORE_DIAMETER_COMPENSATION_MM = 0.15


def build_adapter_socket_mesh(
    neck_id_mm, wall_mm,
    neck_stub_len_mm, transition_len_mm,
    target_spigot_od_mm, taper_angle_deg, engagement_len_mm,
    segments=128, cap_neck=True,
):
    """Returns a geo_helpers Mesh - a hollow tube whose bore widens from
    the neck end out to the socket's own entrance, open bore at both
    ends, closed/watertight everywhere else (when cap_neck is True, the
    default - the entrance end always stays capped, it's the assembly's
    own terminal end regardless). The bore is built
    BORE_DIAMETER_COMPENSATION_MM larger than target_spigot_od_mm, not
    that value directly - see that constant's own comment. Raises
    ValueError if the target spigot is too small to reach past the
    neck's own bore, or if the resulting engagement-tip radius is
    non-positive.

    cap_neck=False skips the neck-end annular cap - for assembling onto
    a neck connector's own matching (also uncapped) end via geo_helpers'
    automatic vertex welding, not a boolean union. See
    build_rectangular_mouth_mesh's own cap_neck docstring for why
    leaving both touching ends capped and welding anyway isn't the same
    as a genuinely fused single shell."""
    r_neck_outer = neck_id_mm / 2.0 + wall_mm
    r_neck_inner = neck_id_mm / 2.0
    # + BORE_DIAMETER_COMPENSATION_MM, not target_spigot_od_mm directly -
    # see that constant's own comment for why the bore is deliberately
    # built a hair larger than the nominal target.
    r_target = (target_spigot_od_mm + BORE_DIAMETER_COMPENSATION_MM) / 2.0
    radial_range = engagement_len_mm * math.tan(math.radians(taper_angle_deg))
    r_base = r_target + radial_range / 2.0   # wide end - socket entrance, mates spigot's base
    r_tip = r_target - radial_range / 2.0    # narrow end - deepest point, mates spigot's tip

    if r_tip <= r_neck_inner:
        raise ValueError(
            "target spigot OD (%.1fmm) too small: engagement tip radius %.2fmm must "
            "exceed the neck's own inner radius %.2fmm - increase target_spigot_od_mm "
            "or reduce neck_id_mm" % (target_spigot_od_mm, r_tip, r_neck_inner))
    if r_tip <= 0:
        raise ValueError(
            "engagement tip radius %.2fmm is non-positive - reduce taper_angle_deg or "
            "engagement_len_mm, or increase target_spigot_od_mm" % r_tip)

    z0 = 0.0
    z1 = neck_stub_len_mm
    z2 = z1 + transition_len_mm     # narrow end of the socket taper (deepest)
    z3 = z2 + engagement_len_mm     # wide end of the socket taper (entrance)
    z_levels = [z0, z1, z2, z3]
    inner_r = [r_neck_inner, r_neck_inner, r_tip, r_base]
    outer_r = [r + wall_mm for r in inner_r]

    # Rings are built at their natural (neck-at-z=0, entrance-at-z=+total)
    # positions first, then rotated 180 degrees about the X axis - a PROPER
    # rotation (determinant +1), which repositions the neck ring exactly onto
    # the origin facing +Z and the entrance ring to z=-total facing -Z, while
    # leaving every ring's own winding untouched. See adapter_taper_core.py's
    # copy of this same comment for why a bare Z-negation (a reflection,
    # det -1) would silently invert every face's outward-normal direction
    # instead, and how that was confirmed directly (checking each cap face's
    # own normal, not just validate()'s closed/positive-volume check).
    outer_rings = [gh.rotate(gh.circle(outer_r[i], segments, center=(0.0, 0.0, z_levels[i])), "x", 180.0)
                   for i in range(4)]
    inner_rings = [gh.rotate(gh.circle(inner_r[i], segments, center=(0.0, 0.0, z_levels[i])), "x", 180.0)
                   for i in range(4)]

    outer_shell = gh.loft(outer_rings)
    inner_shell = gh.flip_normals(gh.loft(inner_rings))
    top_cap = gh.cap_annulus(outer_rings[-1], inner_rings[-1], reverse=False)

    mesh = outer_shell + inner_shell + top_cap
    if cap_neck:
        mesh = mesh + gh.cap_annulus(outer_rings[0], inner_rings[0], reverse=True)

    # The volume-sign self-check only means something on a fully closed
    # mesh - skip it when the neck cap was intentionally left off
    # (winding doesn't change just because a cap face is missing; this
    # was already verified correct on the fully-capped construction).
    if cap_neck and mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=cap_neck)
    if not report.is_valid:
        raise ValueError("adapter socket mesh failed validation: %s" % "; ".join(report.errors))
    # neck_outer_loop/neck_inner_loop: see adapter_taper_core.py's copy of
    # this same comment - the exact (local-space, already-rotated,
    # un-capped) boundary loops at the neck end, for explicit bridging
    # in assemble_core.py.
    return mesh, dict(r_neck_outer=r_neck_outer, r_tip=r_tip, r_base=r_base,
                       total_len=z3, transition_end_z=z2,
                       neck_outer_loop=outer_rings[0], neck_inner_loop=inner_rings[0])
