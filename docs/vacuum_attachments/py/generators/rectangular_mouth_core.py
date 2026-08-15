"""
Pure geometry core for rectangular_mouth.py - every function here is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim (not rewritten) from rectangular_mouth.py so this
exact logic can run outside Blender too (the web configurator, via
Pyodide) with a single source of truth - rectangular_mouth.py itself
now just imports build_rectangular_mouth_mesh from here and wraps it
in a bpy object/operator.

If this file changes, rectangular_mouth.py's own behavior changes too -
there is no separate copy of this logic left in that file. The web
configurator's own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

Parametric rounded-rectangle mouth generator: a hollow, sealed-both-ends
loft from a circular neck stub to a rounded-rectangular mouth rim - circle,
neck (straight run), loft (circle -> rounded rect), mouth (straight run).

Every cross-section is sampled at the SAME set of angles, derived once
from the mouth's own shape so its points are evenly spaced by arc length
(not angle, which would pack points densely along its long straight edges
and sparsely around its tight corners) - see perimeter_uniform_thetas'
own docstring for why sharing one theta set across differently-shaped
cross-sections (the neck's plain circle vs. the mouth's rounded rect) is
what keeps the loft between them from twisting.

Mesh assembly (loft, annular end caps, welded composition, structural
validation) is built on geo_helpers.py's own DSL, vendored alongside this
file - see build_rectangular_mouth_mesh's own comments for the winding/
reverse reasoning. Only the actual rounded-rect/circle point math
(rounded_rect_point_at_angle, perimeter_uniform_thetas) is hand-rolled -
geo_helpers has no rounded-rect-with-corner-radius primitive of its own.
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


def _rounded_rect_boundary_pieces(a, b, r):
    """The 8 pieces of a rounded-rect boundary (4 straight edges, 4 corner
    arcs), walked CCW starting at the right edge - each a (kind, length,
    data) tuple. Zero-length pieces are dropped (r==0 removes all 4 arcs
    down to actual corner points; a==r or b==r removes the edges on that
    axis - a==b==r removes all 4 edges, leaving exactly 4 quarter-arcs,
    i.e. a full circle)."""
    corners = [
        (a - r, -(b - r), -math.pi / 2, 0.0),           # bottom-right
        (a - r, b - r, 0.0, math.pi / 2),                # top-right
        (-(a - r), b - r, math.pi / 2, math.pi),         # top-left
        (-(a - r), -(b - r), math.pi, 3.0 * math.pi / 2),  # bottom-left
    ]
    edges = [
        ((a, -(b - r)), (a, b - r)),          # right
        ((a - r, b), (-(a - r), b)),           # top
        ((-a, b - r), (-a, -(b - r))),         # left
        ((-(a - r), -b), (a - r, -b)),         # bottom
    ]
    seq = [
        ('edge', edges[0]), ('arc', corners[1]),
        ('edge', edges[1]), ('arc', corners[2]),
        ('edge', edges[2]), ('arc', corners[3]),
        ('edge', edges[3]), ('arc', corners[0]),
    ]
    pieces = []
    for kind, data in seq:
        if kind == 'edge':
            (x0, y0), (x1, y1) = data
            length = math.hypot(x1 - x0, y1 - y0)
        else:
            _cx, _cy, a0, a1 = data
            length = r * (a1 - a0)
        if length > 1e-9:
            pieces.append((kind, length, data))
    return pieces


def rounded_rect_point_at_angle(a, b, r, theta):
    """Point on a rounded-rect boundary (half-width a, half-depth b, corner
    radius r) along the ray from origin at angle theta. Reduces exactly to
    a circle of radius r when a == b == r (every ray then passes through
    the single corner-arc center at the origin) - this is the evaluator
    every cross-section is sampled with, always at the SAME thetas (see
    perimeter_uniform_thetas), which is what keeps index i meaning "the
    same radial direction" on every profile regardless of that profile's
    own shape - the actual requirement for a loft between differently-
    shaped profiles to not twist."""
    r = min(r, a, b)
    c, s = math.cos(theta), math.sin(theta)
    if abs(c) < 1e-12:
        return (0.0, math.copysign(b, s))
    if abs(s) < 1e-12:
        return (math.copysign(a, c), 0.0)

    ac, as_ = abs(c), abs(s)
    sign_x = math.copysign(1.0, c)
    sign_y = math.copysign(1.0, s)
    cx, cy = a - r, b - r

    y_at_a = (as_ / ac) * a
    if y_at_a <= cy + 1e-9:
        return (sign_x * a, sign_y * y_at_a)

    x_at_b = (ac / as_) * b
    if x_at_b <= cx + 1e-9:
        return (sign_x * x_at_b, sign_y * b)

    B = -2.0 * (ac * cx + as_ * cy)
    C = cx * cx + cy * cy - r * r
    disc = max(B * B - 4.0 * C, 0.0)
    t = (-B + math.sqrt(disc)) / 2.0
    return (sign_x * t * ac, sign_y * t * as_)


def _arc_len_at_angle_zero(a, b, r, pieces):
    """Cumulative arc length, walking pieces in their own order, from
    pieces[0]'s own start up to the point where the ray at angle 0
    (i.e. rounded_rect_point_at_angle(a, b, r, 0.0), always on the
    +X side) crosses the boundary. Used to re-anchor
    perimeter_uniform_thetas so index 0 sits at angle 0 - see that
    function's own docstring for why. Handles every piece kind/order
    _rounded_rect_boundary_pieces can produce, including degenerate
    shapes (r == a and/or r == b) where the crossing point falls on a
    corner arc instead of the right edge."""
    x0, y0 = rounded_rect_point_at_angle(a, b, r, 0.0)
    acc = 0.0
    for kind, length, data in pieces:
        if kind == 'edge':
            (ex0, ey0), (ex1, ey1) = data
            if abs(ex0 - ex1) < 1e-9:  # vertical edge
                on_it = abs(x0 - ex0) < 1e-6 and min(ey0, ey1) - 1e-6 <= y0 <= max(ey0, ey1) + 1e-6
            else:  # horizontal edge
                on_it = abs(y0 - ey0) < 1e-6 and min(ex0, ex1) - 1e-6 <= x0 <= max(ex0, ex1) + 1e-6
            if on_it:
                return acc + math.hypot(x0 - ex0, y0 - ey0)
        else:
            cx, cy, a0, a1 = data
            if abs(math.hypot(x0 - cx, y0 - cy) - r) < 1e-6:
                ang = math.atan2(y0 - cy, x0 - cx)
                while ang < a0 - 1e-9:
                    ang += 2.0 * math.pi
                while ang > a1 + 1e-9:
                    ang -= 2.0 * math.pi
                if a0 - 1e-6 <= ang <= a1 + 1e-6:
                    return acc + r * (ang - a0)
        acc += length
    raise ValueError("perimeter_uniform_thetas: angle-0 point not found on any boundary piece")


def perimeter_uniform_thetas(a, b, r, segments):
    """Angles (from center) of `segments` points evenly spaced by ARC
    LENGTH around a rounded-rect (or circle, if a==b==r) boundary - not
    evenly spaced by angle, which packs points densely along long straight
    edges and sparsely around tight corner arcs. Index 0 is re-anchored to
    sit at angle 0 (the +X direction) rather than wherever
    _rounded_rect_boundary_pieces' own walk happens to start (the bottom
    of the right edge) - the two coincide only when b == r.

    Returns ANGLES, not points, specifically so a caller can reuse this
    same theta set to sample every OTHER cross-section in a loft (via
    rounded_rect_point_at_angle) too. That is the actual fix for lofts
    twisting between differently-shaped profiles: derive thetas once from
    whichever profile has real corners to space evenly (the mouth), then
    evaluate every profile - including the neck's plain circle - at those
    same thetas, so corresponding indices always sit at the same radial
    direction on every ring, even though the circle's own resulting
    spacing is then whatever the mouth's proportions happen to produce
    (visually harmless for a circle, which has no corners to reveal
    uneven spacing) rather than perfectly angle-uniform itself.

    The angle-0 re-anchoring matters beyond cosmetics: without it, index 0
    here sits wherever the boundary walk happens to start (e.g. ~65
    degrees off from angle 0 for this project's own default 27.8x69.5mm
    mouth) while gh.circle() and every other plain-circle ring in this
    project's chain always starts index 0 at angle 0 - that mismatch is
    what made an otherwise-plausible shared-thetas loft (neck's circle to
    this shape) come out severely twisted (~76 degrees of worst-case
    per-index angular jump). Re-anchoring alone brings that down to ~18
    degrees for the same default mouth (confirmed against a hand-cleaned
    reference assembly, which uses exactly a plain single-band loft here,
    no extra subdivision) - the REMAINING mismatch is real (the mouth's
    own arc-length-even spacing is still a genuinely different
    distribution from the neck's angle-even one) but small enough, and
    smoothly graded around the ring rather than concentrated, to read as
    a normal loft rather than a twist."""
    r = min(r, a, b)
    pieces = _rounded_rect_boundary_pieces(a, b, r)
    total = sum(length for _kind, length, _data in pieces)
    if total < 1e-9:
        return [2.0 * math.pi * i / segments for i in range(segments)]
    offset = _arc_len_at_angle_zero(a, b, r, pieces)

    thetas = []
    for i in range(segments):
        target = (offset + total * i / segments) % total
        acc = 0.0
        for idx, (kind, length, data) in enumerate(pieces):
            if target <= acc + length or idx == len(pieces) - 1:
                t = min(max((target - acc) / length, 0.0), 1.0) if length > 1e-9 else 0.0
                if kind == 'edge':
                    (x0, y0), (x1, y1) = data
                    x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
                else:
                    cx, cy, a0, a1 = data
                    ang = a0 + (a1 - a0) * t
                    x, y = cx + r * math.cos(ang), cy + r * math.sin(ang)
                thetas.append(math.atan2(y, x))
                break
            acc += length
    return thetas


def ring_at_thetas(a, b, r, thetas):
    return [rounded_rect_point_at_angle(a, b, r, theta) for theta in thetas]


def _tilt_z_fn(tilt_deg):
    """None (flat, the default) or a function adding an X-proportional Z
    offset - tilts a ring's own plane around the Y axis without touching
    its (x, y) shape at all. Symmetric about x=0, so the ring's nominal Z
    stays its average height, not its minimum.

    Tilts around Y (not X) specifically so this lands in the SAME plane
    neck_connector_core.py's own bend curves in - that one always curves
    toward +X (see build_centerline's own docstring) - instead of tilting
    on an axis orthogonal to the bend and looking unrelated to it
    (confirmed to be an actual bug when this used Y/around-X: the two
    controls simply couldn't relate to each other on perpendicular axes,
    whatever their signs).

    Sign is negated (-x, not x) on request: a positive Mouth Tilt and a
    positive Bend Angle now lean OPPOSITE ways when both are dialed in on
    the full assembly - previously (same axis, no negation) they leaned
    the same way. Only the sign flipped here; which axis this tilts
    around is unchanged, and _validate_mouth_tilt's own margin check is
    symmetric (uses abs(mouth_tilt_deg)), so it's unaffected either
    way."""
    if tilt_deg == 0.0:
        return None
    slope = math.tan(math.radians(tilt_deg))
    return lambda x, y: slope * -x


# Minimum clearance the tilted opening ring must keep above the mouth's
# own flat base - same 0.4-1mm floor-margin convention this project uses
# everywhere else (merch_core.py's own hole/bevel/wall clamps), just
# applied here as a reject condition instead of a clamp (see
# _validate_mouth_tilt's own docstring for why this one rejects rather
# than silently substitutes a value).
MOUTH_TILT_MARGIN_MM = 0.5


def _validate_mouth_tilt(mouth_height_mm, mouth_width_mm, mouth_tilt_deg):
    """Raises a clear ValueError if mouth_tilt_deg would tilt the opening
    ring at or below the mouth's own flat base on one side, rather than
    silently clamping to whatever tilt IS safe. Clamping is the right
    call for a value like Border Thickness or Bevel Length, where the
    clamped result is still basically what was asked for - but Mouth
    Tilt is a deliberate, specific choice, and silently downgrading a
    requested 45 degrees to some much smaller safe value would produce a
    result the caller didn't ask for and might not notice. Same "reject
    with an actionable message" pattern label_maker's own text-overflow
    check already uses, for the same reason.

    Checked against mouth_width_mm (the X extent), not mouth_depth_mm -
    the tilt itself is around the Y axis now (see _tilt_z_fn's own
    docstring for why), so the opening's lowest point is at the +/-X
    extreme, not +/-Y."""
    if mouth_tilt_deg == 0.0:
        return
    slope = math.tan(math.radians(abs(mouth_tilt_deg)))
    lowest_drop = slope * (mouth_width_mm / 2.0)
    clearance = mouth_height_mm - lowest_drop
    if clearance < MOUTH_TILT_MARGIN_MM:
        max_tilt = math.degrees(math.atan(
            2.0 * (mouth_height_mm - MOUTH_TILT_MARGIN_MM) / mouth_width_mm
        )) if mouth_height_mm > MOUTH_TILT_MARGIN_MM else 0.0
        raise ValueError(
            f"Mouth Tilt of {mouth_tilt_deg:.1f} deg drops the opening "
            f"{lowest_drop:.2f}mm below Mouth Height ({mouth_height_mm:.2f}mm) on one side, "
            f"leaving only {clearance:.2f}mm of clearance above the mouth's own base - under "
            f"the {MOUTH_TILT_MARGIN_MM:.1f}mm minimum. Increase Mouth Height, reduce Mouth "
            f"Width, or keep Mouth Tilt under {max_tilt:.1f} deg."
        )


def build_rectangular_mouth_mesh(
    id_mm, wall_mm,
    mouth_width_mm, mouth_depth_mm, mouth_corner_radius_mm,
    neck_height_mm, loft_height_mm, mouth_height_mm,
    mouth_tilt_deg=0.0,
    segments=128, cap_neck=True,
):
    """Returns a geo_helpers Mesh - pure geometry, no bpy. Sealed at both
    ends by default (a full solid, printable/previewable on its own -
    see this project's own established convention). z=0 is the neck's
    own free end; z=total_height is the mouth opening.

    cap_neck=False skips the neck-end annular cap - for assembling onto
    a neck connector's own matching (uncapped) end via geo_helpers'
    automatic vertex welding, not a boolean union. Leaving BOTH pieces'
    touching ends capped and just gluing the objects together would
    still weld the boundary vertices (same radius/position) but leave
    two coincident, coplanar annular faces sitting exactly on top of
    each other at the seam - closed in the loose "no boundary edges"
    sense, but not the clean single continuous shell an actual merged
    part should be. Skipping the cap on exactly one side of a join is
    what avoids that, instead of a boolean cleanup pass afterward.

    mouth_tilt_deg tilts ONLY the opening ring (the top of mouth_height_mm's
    own straight run) around the Y axis - the mouth's bottom ring (where
    the loft from the neck ends) stays flat. The straight side walls
    already connecting a flat ring to a tilted one come out taller on one
    side and shorter on the other with no other change needed - exactly
    what a tube sliced at an angle looks like. 0 = flat (square-cut).
    Tilts around Y (not X) so it leans in the SAME plane
    neck_connector_core.py's own bend curves in - see _tilt_z_fn's own
    docstring for why. Raises ValueError (does not clamp) if the tilt
    would drop the opening at or below the mouth's own flat base - see
    _validate_mouth_tilt's own docstring for why this one rejects.

    neck_outer_r is DERIVED, not independently specified - id_mm is what a
    caller actually knows (it has to match a real shaft), and wall_mm is
    the other real design constraint, so neck_outer_r = id_mm/2 + wall_mm
    follows from those two rather than being its own free parameter that
    could silently drift out of sync with them."""
    _validate_mouth_tilt(mouth_height_mm, mouth_width_mm, mouth_tilt_deg)
    neck_inner_r = id_mm / 2.0
    neck_outer_r = neck_inner_r + wall_mm
    mouth_a, mouth_b, mouth_r = mouth_width_mm / 2.0, mouth_depth_mm / 2.0, mouth_corner_radius_mm
    mouth_inner_a = max(mouth_a - wall_mm, 0.05)
    mouth_inner_b = max(mouth_b - wall_mm, 0.05)
    mouth_inner_r = max(mouth_r - wall_mm, 0.05)

    z_neck_bottom = 0.0
    z_neck_top = neck_height_mm
    z_mouth_bottom = neck_height_mm + loft_height_mm
    z_mouth_top = neck_height_mm + loft_height_mm + mouth_height_mm
    tilt = _tilt_z_fn(mouth_tilt_deg)

    # Both neck levels (z=0, the actual external joint, and
    # z=neck_height_mm) use plain even-angle thetas - the same formula
    # gh.circle() itself uses, and also the ideal spacing for a plain
    # circle regardless (no arc-length reason to prefer anything else
    # the way there is for the mouth's own rounded rect below). Sharing
    # the identical array between these two rings makes the straight
    # run a single, genuinely untwisted cylindrical band.
    #
    # The mouth's own rim (mouth-bottom, mouth-top) shares its own
    # perimeter_uniform_thetas array instead - required to keep the
    # mouth's own corners round (checked numerically: even-angle
    # sampling puts only ~3 points on each 90 degree corner arc for this
    # project's own default mouth, visibly faceted).
    #
    # perimeter_uniform_thetas re-anchors its own index 0 to angle 0
    # (see that function's own docstring) specifically so THIS loft -
    # neck-top's even-angle ring straight into mouth-bottom's
    # perimeter-uniform ring, one plain band, no subdivision - doesn't
    # twist: without that re-anchoring the two theta arrays disagreed by
    # up to ~76 degrees at some indices (mouth-derived index 0 sat
    # wherever the boundary walk happened to start, not at angle 0);
    # with it, the worst disagreement for this project's own default
    # mouth is ~18 degrees, smoothly graded around the ring rather than
    # concentrated - confirmed against a hand-cleaned reference assembly,
    # which uses exactly this plain single-band structure with no extra
    # loft subdivision at all.
    neck_thetas = [2.0 * math.pi * k / segments for k in range(segments)]
    outer_thetas = perimeter_uniform_thetas(mouth_a, mouth_b, mouth_r, segments)
    inner_thetas = perimeter_uniform_thetas(mouth_inner_a, mouth_inner_b, mouth_inner_r, segments)

    def rr_loop(a, b, r, z, thetas, tilt_fn, loop_name):
        pts = ring_at_thetas(a, b, r, thetas)
        if tilt_fn is None:
            return gh.Loop([(x, y, z) for x, y in pts], True, loop_name)
        return gh.Loop([(x, y, z + tilt_fn(x, y)) for x, y in pts], True, loop_name)

    levels = [
        (z_neck_bottom, neck_outer_r, neck_outer_r, neck_outer_r, None),
        (z_neck_top, neck_outer_r, neck_outer_r, neck_outer_r, None),
        (z_mouth_bottom, mouth_a, mouth_b, mouth_r, None),
        (z_mouth_top, mouth_a, mouth_b, mouth_r, tilt),
    ]
    outer_loops = [
        rr_loop(a, b, r, z, neck_thetas if i < 2 else outer_thetas, tilt_fn, "outer_%d" % i)
        for i, (z, a, b, r, tilt_fn) in enumerate(levels)
    ]
    inner_loops = [
        rr_loop(max(a - wall_mm, 0.05), max(b - wall_mm, 0.05), max(r - wall_mm, 0.05), z,
                neck_thetas if i < 2 else inner_thetas, tilt_fn, "inner_%d" % i)
        for i, (z, a, b, r, tilt_fn) in enumerate(levels)
    ]

    # Both loop families wind CCW-from-+Z (same convention
    # rounded_rect_point_at_angle/perimeter_uniform_thetas trace in), and
    # every level sits at increasing Z, so gh.loft() on the outer loops
    # alone already gives outward-facing side walls (geo_helpers' own
    # convention: "outward when both rings are CCW and top sits along
    # bottom's own normal" - true here since higher Z IS "along +Z" from
    # a lower CCW-from-+Z ring). The inner loops loft the same way but
    # need flip_normals: "outward from the inner tube's own axis" there
    # actually means "into the solid wall", the wrong side for a proper
    # manifold, where every face must point away from material.
    outer_shell = gh.loft(outer_loops)
    inner_shell = gh.flip_normals(gh.loft(inner_loops))
    # Bottom (z=0) cap needs to face -Z (out through the neck's own free
    # end); a CCW-from-+Z loop's cap naturally faces +Z, so reverse=True.
    # Top (mouth opening) cap already wants +Z, its own natural facing.
    mesh = outer_shell + inner_shell
    if cap_neck:
        mesh = mesh + gh.cap_annulus(outer_loops[0], inner_loops[0], reverse=True)
    mesh = mesh + gh.cap_annulus(outer_loops[-1], inner_loops[-1])

    report = mesh.validate(require_closed=cap_neck)
    if not report.is_valid:
        raise ValueError("build_rectangular_mouth_mesh produced an invalid mesh: %s" % report.errors)
    # neck_outer_loop/neck_inner_loop: the exact (local-space, un-capped)
    # boundary loops at z=0 - for a caller assembling this onto a neck
    # connector to bridge against directly (see assemble_core.py),
    # instead of hoping two independently-computed rings happen to share
    # coincident vertex positions. This ring's own points come from
    # perimeter_uniform_thetas applied to the MOUTH's shape (see that
    # function's docstring) - even though it's mathematically a plain
    # circle here (a==b==r), its point distribution is NOT evenly spaced
    # by angle, so it will never coincide with a plain gh.circle() call
    # at the same radius. That mismatch is fine for an explicit bridge
    # (which only needs matching vertex COUNT, not matching positions)
    # and must never be "fixed" by changing this ring's own thetas - see
    # the module-level docstring for why sharing thetas across the whole
    # family is required to keep the mouth's own internal loft twist-free.
    return mesh, dict(neck_outer_loop=outer_loops[0], neck_inner_loop=inner_loops[0])
