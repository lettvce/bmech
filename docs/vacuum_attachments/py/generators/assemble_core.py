"""
Pure geometry core for assemble_attachment.py - build_assembly_mesh
builds a mouth + neck connector + adapter as one fused, watertight mesh
with zero bpy/bmesh/mathutils dependency, so this exact logic can run
outside Blender too (the web configurator, via Pyodide) with a single
source of truth. assemble_attachment.py itself now just calls
build_assembly_mesh and wraps the result in a bpy object/operator.

If this file changes, assemble_attachment.py's own behavior changes too
- there is no separate copy of this logic left in that file. The web
configurator's own copy of this file (and its own copies of the 5
generator *_core.py files it calls into) is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

The one genuinely new thing here (not just an extraction):
rotation_from_to below is a pure-Python substitute for
mathutils.Vector.rotation_difference() (Rodrigues' rotation formula),
used to orient the adapter to match the connector's own far-end tangent.
It started this session as a hand-rolled replica for headless testing
outside Blender - verified equivalent to mathutils' actual output for
both straight and bent connector cases before being trusted - and now
IS the one real implementation both the Blender operator and the web
page share, not a test double any more.

Placement logic:
  - Mouth: the "root" of the assembly, built at the local origin, no
    rotation. Its own neck ring sits at its local origin facing -Z (the
    established mouth-generator convention: material extends +Z from
    the neck up to the mouth opening, so the neck ring itself faces the
    opposite way, -Z, away from that material).
  - Neck connector: its own neck ring sits at ITS local origin facing
    +Z (verified directly on the real mesh when this piece was built) -
    the exact opposite of the mouth's neck. Built at the SAME local
    position as the mouth, no rotation, so the two -Z/+Z-facing rings
    coincide and face each other automatically.
  - Adapter: placed at the connector's own far-end position, rotated so
    the adapter's local +Z (its own neck-facing direction) points
    opposite the connector's far-end tangent - mirroring the same
    complementary-facing pairing used for the mouth/connector join,
    just computed instead of being free (the connector's far end can
    point anywhere, straight or bent). The far-end tangent is read
    directly from the last two points of the connector's own centerline
    (neck_connector_core.build_centerline) rather than recomputed by
    hand here, so this can't silently drift out of sync with the
    connector's own bend math.

Fusing into one object - why a plain weld isn't enough, and what this
does instead:
  Every piece is built with cap_neck=False (or cap_start/cap_end=False)
  at its own internal joints, leaving an open boundary ring there
  instead of a sealed end - see build_rectangular_mouth_mesh's own
  cap_neck docstring for why leaving both touching ends capped and
  welding the objects together anyway would NOT produce a genuinely
  fused single shell (two coincident, coplanar faces sitting on top of
  each other at the seam, not one).

  The mouth<->connector joint welds automatically via geo_helpers'
  Mesh.__add__ (coincident vertices, no explicit bridge needed) - the
  mouth's own neck ring is bit-identical to gh.circle() at the same
  radius/segments (see rectangular_mouth_core.py's/elliptical_mouth_core.py's
  own comments on why), matching the connector's own start ring exactly.

  The connector<->adapter joint DOES need an explicit bridge, even
  though its two rings are built from the literal same gh.circle() call:
  relying on automatic welding there is fragile in practice, since two
  INDEPENDENTLY-computed transforms (extrude_path's own frame vs. this
  module's own rotation) landing on the exact same side of geo_helpers'
  weld-tolerance rounding boundary isn't something to rely on (confirmed
  the hard way in real Blender - see JOINT_GAP_MM's own comment). The
  robust fix: push the adapter JOINT_GAP_MM further out along the
  connector's own far-end tangent before bridging - far below FDM print
  resolution, but comfortably outside the weld tolerance, so the two
  rings can never accidentally coincide and the bridge is always a
  genuine (if razor-thin) band rather than a degenerate one.
"""

import sys
import os
import math

# Pre-set to this project's own folder, where every sibling *_core module
# below already lives - makes pasting a Blender-facing wrapper into a
# blank Text Editor tab in an unsaved .blend work with no manual step.
# Only change this if the project folder ever moves. When vendored into
# the web configurator, every sibling file sits alongside this one, so
# the plain sys.path fallback below finds them there too without needing
# GEO_HELPERS_DIR set at all.
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
import rectangular_mouth_core
import elliptical_mouth_core
import neck_connector_core
import adapter_taper_core
import adapter_socket_core


# Deliberate clearance between the connector's own far end and the
# adapter's neck ring before they're explicitly bridged - see this
# module's own docstring for why relying on the two rings landing
# exactly (or almost exactly) coincident is fragile in practice, even
# though they come from the same gh.circle() call. Far below FDM print
# resolution (~0.1-0.4mm), comfortably above geo_helpers' own 1e-6mm
# weld tolerance.
JOINT_GAP_MM = 0.01


# ---------------------------------------------------------------------------
# Pure-Python rotation math - substitute for mathutils.Vector/Quaternion,
# see this module's own docstring for why this exists and how it was
# verified.
# ---------------------------------------------------------------------------

def _normalize(v):
    length = math.sqrt(sum(c * c for c in v))
    return tuple(c / length for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _rotate_about_axis(v, axis, angle):
    """Rodrigues' rotation formula: rotate vector v by angle (radians)
    about unit axis."""
    axis = _normalize(axis)
    c, s = math.cos(angle), math.sin(angle)
    t1 = tuple(x * c for x in v)
    cr = _cross(axis, v)
    t2 = tuple(x * s for x in cr)
    kdv = _dot(axis, v)
    t3 = tuple(x * kdv * (1.0 - c) for x in axis)
    return tuple(a + b + c for a, b, c in zip(t1, t2, t3))


def rotation_from_to(v_from, v_to):
    """Returns a function that applies the shortest rotation taking
    v_from onto v_to to any vector - pure-Python substitute for
    mathutils.Vector.rotation_difference(). Handles the degenerate
    parallel/antiparallel case (zero cross product) by returning an
    identity or negation instead of dividing by zero."""
    v_from = _normalize(v_from)
    v_to = _normalize(v_to)
    axis = _cross(v_from, v_to)
    axis_len = math.sqrt(sum(c * c for c in axis))
    if axis_len < 1e-9:
        sign = 1.0 if _dot(v_from, v_to) > 0 else -1.0
        return lambda v: tuple(c * sign for c in v)
    axis_n = _normalize(axis)
    angle = math.acos(max(-1.0, min(1.0, _dot(v_from, v_to))))
    return lambda v: _rotate_about_axis(v, axis_n, angle)


def _transform_mesh(gh_mesh, rotate_fn, translate):
    """Rotate then translate every vertex of a geo_helpers Mesh in place -
    used to move the adapter into the connector's own far-end frame
    before the pieces are combined."""
    gh_mesh.verts = [tuple(a + b for a, b in zip(rotate_fn(v), translate)) for v in gh_mesh.verts]
    return gh_mesh


def _transform_loop(loop, rotate_fn, translate):
    """Same transform as _transform_mesh, applied to a boundary Loop's
    own points instead of a full Mesh - for the connector/adapter
    joint's explicit bridge, which needs the adapter's own neck ring in
    the same placed frame as the rest of the adapter mesh."""
    return gh.Loop([tuple(a + b for a, b in zip(rotate_fn(v), translate)) for v in loop.verts],
                    loop.is_closed, loop.name)


def build_assembly_mesh(
    neck_id_mm, wall_mm, segments,
    mouth_kind,
    mouth_width_mm, mouth_depth_mm, mouth_corner_radius_mm,
    mouth_neck_height_mm, mouth_loft_height_mm, mouth_height_mm, mouth_tilt_deg,
    lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm,
    adapter_kind,
    adapter_neck_stub_len_mm, adapter_transition_len_mm, target_od_mm,
    taper_angle_deg, engagement_len_mm,
):
    """Returns (mesh, report) - a single fused geo_helpers Mesh (mouth +
    neck connector + adapter) and its own mesh.validate(require_closed=True)
    report, report.volume already populated. mouth_kind is 'RECTANGULAR'
    or 'ELLIPTICAL'; adapter_kind is 'TAPER' or 'SOCKET'.

    Raises ValueError, prefixed with which stage failed ("Mouth: ...",
    "Neck connector: ...", "Adapter: ...", or the bare fuse-failure
    message if the final combined mesh somehow doesn't validate - not
    expected in normal use, kept as a safety net) - callers should catch
    ValueError and show str(e) to the user, same pattern used throughout
    this project."""
    try:
        if mouth_kind == 'RECTANGULAR':
            mouth_mesh, _mouth_loops = rectangular_mouth_core.build_rectangular_mouth_mesh(
                neck_id_mm, wall_mm,
                mouth_width_mm, mouth_depth_mm, mouth_corner_radius_mm,
                mouth_neck_height_mm, mouth_loft_height_mm, mouth_height_mm,
                mouth_tilt_deg=mouth_tilt_deg, segments=segments, cap_neck=False,
            )
        else:
            mouth_mesh, _mouth_loops = elliptical_mouth_core.build_elliptical_mouth_mesh(
                neck_id_mm, wall_mm,
                mouth_width_mm, mouth_depth_mm,
                mouth_neck_height_mm, mouth_loft_height_mm, mouth_height_mm,
                mouth_tilt_deg=mouth_tilt_deg, segments=segments, cap_neck=False,
            )
    except ValueError as e:
        raise ValueError("Mouth: %s" % e) from e

    try:
        connector_mesh, conn_loops = neck_connector_core.build_neck_connector_mesh(
            neck_id_mm, wall_mm,
            lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm,
            segments=segments, cap_start=False, cap_end=False,
        )
    except ValueError as e:
        raise ValueError("Neck connector: %s" % e) from e

    # Adapter placed at the connector's own far-end local position (plus
    # a deliberate JOINT_GAP_MM clearance), rotated so its local +Z
    # (neck-facing direction) points opposite the connector's far-end
    # tangent, mirroring the mouth/connector pairing. See this module's
    # own docstring.
    path_pts = neck_connector_core.build_centerline(lead_in_len_mm, bend_angle_deg, bend_radius_mm, lead_out_len_mm)
    far_end_local = path_pts[-1]
    prev_local = path_pts[-2]
    tangent_end = _normalize(tuple(a - b for a, b in zip(far_end_local, prev_local)))
    target_dir = tuple(-c for c in tangent_end)
    rotate_fn = rotation_from_to((0.0, 0.0, 1.0), target_dir)
    gapped_translate = tuple(f + JOINT_GAP_MM * d for f, d in zip(far_end_local, target_dir))

    build_fn = adapter_taper_core.build_adapter_taper_mesh if adapter_kind == 'TAPER' else adapter_socket_core.build_adapter_socket_mesh
    try:
        adapter_mesh, adapter_info = build_fn(
            neck_id_mm, wall_mm,
            adapter_neck_stub_len_mm, adapter_transition_len_mm,
            target_od_mm, taper_angle_deg, engagement_len_mm,
            segments=segments, cap_neck=False,
        )
    except ValueError as e:
        raise ValueError("Adapter: %s" % e) from e

    adapter_neck_outer = _transform_loop(adapter_info["neck_outer_loop"], rotate_fn, gapped_translate)
    adapter_neck_inner = _transform_loop(adapter_info["neck_inner_loop"], rotate_fn, gapped_translate)
    _transform_mesh(adapter_mesh, rotate_fn, gapped_translate)

    # Fuse into one shell: the mouth/connector joint welds automatically
    # (see this module's own docstring), no bridge needed there. The
    # connector/adapter joint still needs an explicit bridge.
    joint_outer = gh.bridge(conn_loops["end_outer_loop"], adapter_neck_outer)
    joint_inner = gh.bridge(conn_loops["end_inner_loop"].reversed(), adapter_neck_inner.reversed())

    combined = mouth_mesh + connector_mesh + adapter_mesh + joint_outer + joint_inner
    report = combined.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Assembly failed to fuse into one closed shell: %s"
                          % "; ".join(report.errors))
    return combined, report
