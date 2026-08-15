"""
Pure geometry core for neck_connector.py - every function here is plain
Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim (not rewritten) from neck_connector.py so this exact
logic can run outside Blender too (the web configurator, via Pyodide)
with a single source of truth - neck_connector.py itself now just
imports build_centerline/build_neck_connector_mesh from here and wraps
the mesh in a bpy object/operator.

If this file changes, neck_connector.py's own behavior changes too -
there is no separate copy of this logic left in that file. The web
configurator's own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

Parametric neck/bend connector for vacuum attachments - bridges a
mouth's own neck stub to an adapter's neck stub, straight or bent.

Chain this sits in: mouth -> neck_connector -> adapter_taper/
adapter_socket (bend_angle_deg=0, straight run), or
mouth -> neck_connector(bent) -> adapter for a redirected attachment.
One generator covers both - a straight run is just the bend_angle_deg=0
degenerate case, not a separate code path.

Orientation: same shared attachment-interface convention as every other
piece in this chain - the neck-facing ring sits at the local origin,
facing +Z (verified directly on the actual mesh, not assumed: the start
cap's own face normal comes out to exactly +Z, both straight and bent).
The far end (where an adapter's own +Z-facing neck plugs in) faces
whatever direction the centerline ends up pointing after the bend - pure
-Z for a straight run, tilted for a real bend. That's expected, not a
convention violation: the whole point of a bend is to redirect the
tube's own axis, and the piece plugged in there gets rotated to match
when it's placed, same as any real 3D-printed bend fitting.

Geometry: a plain constant-cross-section tube (neck_id_mm/wall_mm,
matching every other piece's standard interface - no tapering here,
unlike the adapters, since both ends already share the same diameter)
swept along a centerline via geo_helpers.extrude_path:
  1. Straight lead-in, heading -Z from the origin.
  2. An optional circular arc (bend_angle_deg, bend_radius_mm), fixed to
     curve within the XZ plane toward +X. Skipped entirely when
     bend_angle_deg is 0 - not sampled down to a degenerate point, the
     arc contributes nothing to the centerline at all in that case.
  3. Straight lead-out, continuing in whatever direction the arc left
     off in (pure -Z again if there was no bend).
Since the swept profile is a plain circle (rotationally symmetric about
its own sweep axis), there's no twist-matching concern the way there was
for hairspring.py's rectangular ribbon profile - any residual twist in
extrude_path's parallel-transport frame is invisible on a round tube.

bend_radius_mm must exceed the tube's own outer radius or the inside
surface of the bend self-intersects - checked and rejected, not clamped
(same "clear numbers, not a silent substitution" policy as the mouth's
own tilt validation).
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


def build_centerline(lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm):
    """Open path: straight down from the origin (-Z), an optional
    circular arc curving toward +X, then a straight run continuing in
    the post-bend direction. bend_angle_deg=0 skips the arc entirely."""
    pts = [(0.0, 0.0, 0.0), (0.0, 0.0, -lead_in_len_mm)]
    tangent_end = (0.0, 0.0, -1.0)
    p_end = (0.0, 0.0, -lead_in_len_mm)

    if abs(bend_angle_deg) > 1e-9:
        theta_max = math.radians(bend_angle_deg)
        center = (bend_radius_mm, 0.0, -lead_in_len_mm)
        n_arc = max(4, round(abs(bend_angle_deg) / 5.0))
        arc_pts = []
        for i in range(1, n_arc + 1):
            t = theta_max * i / n_arc
            x = center[0] - bend_radius_mm * math.cos(t)
            z = center[2] - bend_radius_mm * math.sin(t)
            arc_pts.append((x, 0.0, z))
        pts.extend(arc_pts)
        p_end = arc_pts[-1]
        tangent_end = (math.sin(theta_max), 0.0, -math.cos(theta_max))

    p_out = (p_end[0] + lead_out_len_mm * tangent_end[0],
             p_end[1] + lead_out_len_mm * tangent_end[1],
             p_end[2] + lead_out_len_mm * tangent_end[2])
    pts.append(p_out)
    return pts


def build_neck_connector_mesh(
    neck_id_mm, wall_mm,
    lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm,
    segments=128, cap_start=True, cap_end=True,
):
    """Returns a geo_helpers Mesh - a hollow tube, open bore at both
    ends, closed/watertight everywhere else (when cap_start and cap_end
    are both True, the default). Raises ValueError if bend_radius_mm is
    too tight relative to the tube's own outer radius.

    cap_start/cap_end=False skip the corresponding annular end cap - for
    assembling onto a mouth's or adapter's own matching (also uncapped)
    end via geo_helpers' automatic vertex welding, not a boolean union.
    See build_rectangular_mouth_mesh's own cap_neck docstring for why
    leaving BOTH touching ends capped and welding anyway isn't the same
    as a genuinely fused single shell."""
    r_outer = neck_id_mm / 2.0 + wall_mm
    r_inner = neck_id_mm / 2.0
    if bend_angle_deg != 0.0 and bend_radius_mm <= r_outer:
        raise ValueError(
            "bend_radius_mm (%.2f) must exceed the tube's own outer radius (%.2f) "
            "or the inside of the bend self-intersects" % (bend_radius_mm, r_outer))

    path_pts = build_centerline(lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm)
    path = gh.Loop(path_pts, False, "centerline")

    outer_profile = gh.circle(r_outer, segments)
    inner_profile = gh.circle(r_inner, segments)

    outer_shell = gh.extrude_path(outer_profile, path)
    inner_shell = gh.flip_normals(gh.extrude_path(inner_profile, path))

    n = segments
    m = len(path.verts)
    outer_start = gh.Loop(outer_shell.verts[0:n], True, "outer_start")
    outer_end = gh.Loop(outer_shell.verts[(m - 1) * n: m * n], True, "outer_end")
    inner_start = gh.Loop(inner_shell.verts[0:n], True, "inner_start")
    inner_end = gh.Loop(inner_shell.verts[(m - 1) * n: m * n], True, "inner_end")

    mesh = outer_shell + inner_shell
    if cap_start:
        mesh = mesh + gh.cap_annulus(outer_start, inner_start, reverse=True)
    if cap_end:
        mesh = mesh + gh.cap_annulus(outer_end, inner_end, reverse=False)

    # The volume-sign self-check only means something on a fully closed
    # mesh - skip it when a cap was intentionally left off (winding
    # doesn't change just because a cap face is missing; this was
    # already verified correct on the fully-capped construction).
    if cap_start and cap_end and mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=(cap_start and cap_end))
    if not report.is_valid:
        raise ValueError("neck connector mesh failed validation: %s" % "; ".join(report.errors))
    # start/end outer/inner loops: the exact (local-space, un-capped)
    # boundary loops this piece's own ends were built from - for a
    # caller assembling this against a mouth's or adapter's own boundary
    # loop via an explicit gh.bridge() (see assemble_core.py), rather
    # than relying on two independently-computed rings happening to
    # share coincident vertex positions (they generally won't, even at
    # identical radius - see rectangular_mouth_core.py's own
    # neck_outer_loop comment for why).
    return mesh, dict(start_outer_loop=outer_start, start_inner_loop=inner_start,
                       end_outer_loop=outer_end, end_inner_loop=inner_end)
