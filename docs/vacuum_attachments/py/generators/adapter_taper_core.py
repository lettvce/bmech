"""
Pure geometry core for adapter_taper.py - build_adapter_taper_mesh is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim (not rewritten) from adapter_taper.py so this exact
logic can run outside Blender too (the web configurator, via Pyodide)
with a single source of truth - adapter_taper.py itself now just imports
build_adapter_taper_mesh from here and wraps it in a bpy object/operator.

If this file changes, adapter_taper.py's own behavior changes too - there
is no separate copy of this logic left in that file. The web
configurator's own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

Parametric friction-fit taper adapter for vacuum attachments.

Bridges the shared attachment-interface convention (a plain circular
stub — fixed local origin, XY plane, facing +Z, matching the neck stub
every mouth generator already exposes) to a tapered male connector sized
to plug into a real vacuum's hose/port. Own standalone part, meant to sit
at the far end of a neck (straight or bent) — chain is either
mouth -> neck -> adapter, or mouth -> neck -> bend -> neck -> adapter
once bends exist.

Orientation: the neck's own connecting ring sits at the local origin
facing +Z (matching every mouth generator's own neck-stub convention);
the whole part extends downward from there, tip facing -Z.

Shape, neck-end to tip (four rings, three sections):
  1. Straight neck stub — same ID/wall as every mouth's own neck
     (25.4mm ID / 1.2mm wall by default), a short constant-section run
     for a clean bridge/glue surface to the neck piece.
  2. Transition cone — expands from the neck stub's OD out to a
     shoulder wider than the target hose ID. This is a moderate,
     practical angle (whatever gets from neck OD to the shoulder over
     transition_len_mm), not the shallow friction-taper angle — it's
     just getting the part's diameter into the right neighborhood.
  3. Engagement taper — the actual friction/wedge surface, a SHALLOW
     cone (taper_angle_deg, default 3°, matching the <5° figure real
     vacuum-connector patents use) that CONTRACTS from the shoulder
     down to the tip, straddling the target hose ID at its midpoint.
     Pushing the adapter further into the target hose engages more of
     this taper, wedging tighter — self-limiting at the shoulder.

The bore stays open (no cap) at both ends — this is a hollow tube
carrying airflow through its whole length, same convention as every
mouth generator: an ANNULAR cap (outer rim to inner rim) at each end
gives the mesh real wall thickness at the rim, while the region inside
each inner ring stays unmeshed — that absence IS the open bore, not a
literal hole cut into a solid disc.

Engagement taper geometry is provisional, not researched hardware specs
— target_id_mm/taper_angle_deg/engagement_len_mm are meant to be
recalibrated once a real fitting gets measured with calipers. 32mm and
35mm are the two household hose IDs with real repeated evidence behind
them; everything past that (exact taper angle, engagement length) is a
reasonable generic starting point, not a match to any specific vacuum.
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


def build_adapter_taper_mesh(
    neck_id_mm, wall_mm,
    neck_stub_len_mm, transition_len_mm,
    target_id_mm, taper_angle_deg, engagement_len_mm,
    segments=128, cap_neck=True,
):
    """Returns a geo_helpers Mesh — a hollow tapered tube, open bore at
    both ends, closed/watertight everywhere else (when cap_neck is
    True, the default - the tip end always stays capped, it's the
    assembly's own terminal end regardless). Raises ValueError for a
    target diameter too small to reach past the neck's own OD, or a
    taper aggressive enough to close off the bore at the tip.

    cap_neck=False skips the neck-end annular cap - for assembling onto
    a neck connector's own matching (also uncapped) end via geo_helpers'
    automatic vertex welding, not a boolean union. See
    build_rectangular_mouth_mesh's own cap_neck docstring for why
    leaving both touching ends capped and welding anyway isn't the same
    as a genuinely fused single shell."""
    r_neck_outer = neck_id_mm / 2.0 + wall_mm
    r_neck_inner = neck_id_mm / 2.0
    r_target = target_id_mm / 2.0
    radial_range = engagement_len_mm * math.tan(math.radians(taper_angle_deg))
    r_shoulder = r_target + radial_range / 2.0
    r_tip = r_target - radial_range / 2.0

    if r_shoulder <= r_neck_outer:
        raise ValueError(
            "target diameter (%.1fmm) too small: shoulder radius %.2fmm must exceed "
            "the neck's own outer radius %.2fmm - increase target_id_mm or reduce "
            "neck_id_mm/wall_mm" % (target_id_mm, r_shoulder, r_neck_outer))
    if r_tip - wall_mm <= 0:
        raise ValueError(
            "tip radius %.2fmm minus wall %.2fmm leaves no bore - reduce "
            "taper_angle_deg or engagement_len_mm, or increase target_id_mm"
            % (r_tip, wall_mm))

    z0 = 0.0
    z1 = neck_stub_len_mm
    z2 = z1 + transition_len_mm
    z3 = z2 + engagement_len_mm
    z_levels = [z0, z1, z2, z3]
    outer_r = [r_neck_outer, r_neck_outer, r_shoulder, r_tip]
    inner_r = [r_neck_inner, r_neck_inner, r_shoulder - wall_mm, r_tip - wall_mm]

    # Rings are built at their natural (neck-at-z=0, tip-at-z=+total) positions
    # first, then rotated 180 degrees about the X axis - a PROPER rotation
    # (determinant +1), which repositions the neck ring exactly onto the
    # origin facing +Z and the tip ring to z=-total facing -Z, while leaving
    # every ring's own winding untouched. A bare Z-negation would achieve the
    # same repositioning but is a REFLECTION (determinant -1) that inverts
    # chirality, silently breaking every face's outward-normal direction -
    # confirmed by building both ways and checking each cap face's own normal
    # directly (not just validate()'s closed/positive-volume check, which a
    # globally-inverted-but-still-closed mesh can still pass): the rotated
    # version's neck-cap normal.z came out to exactly +1.0 and the tip-cap
    # normal.z to exactly -1.0, matching the requested orientation exactly.
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
        raise ValueError("adapter taper mesh failed validation: %s" % "; ".join(report.errors))
    # neck_outer_loop/neck_inner_loop: the exact (local-space, already-
    # rotated, un-capped) boundary loops at the neck end - for a caller
    # bridging this onto a neck connector's own matching end via an
    # explicit gh.bridge() (see assemble_core.py), rather than relying
    # on two independently-computed rings happening to share coincident
    # vertex positions.
    return mesh, dict(r_neck_outer=r_neck_outer, r_shoulder=r_shoulder, r_tip=r_tip,
                       total_len=z3, transition_end_z=z2,
                       neck_outer_loop=outer_rings[0], neck_inner_loop=inner_rings[0])
