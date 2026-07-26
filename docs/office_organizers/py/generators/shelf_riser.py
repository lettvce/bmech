# Shelf Riser generator - parametric version of modeled_orgs/shelf_riser.blend.
#
# A single open-front U-channel wall: one closed outline tracing the
# outer boundary and the inner boundary together (connected at the two
# open front ends, at y=0), with the two back corners filleted -
# concentric outer/inner arcs sharing one center each, so wall
# thickness stays constant all the way around the curve. Extruded
# straight up from z=0 to z=height_mm with flat top/bottom caps.
#
# The result: a riser that elevates whatever sits on its flat top by
# height_mm, with a hollow, front-accessible cubby underneath, bounded
# by the back wall and two side legs. Pure geo_helpers, no booleans -
# unlike wall_cubby, this is a single simple outline (not a multi-hole
# face, and nothing stacks face-to-face), so it never hits either of
# the gaps that pushed wall_cubby toward bmesh_helpers.
#
# X = width, Y = depth (0 at the open front, increasing toward the back
# wall), Z = height (0 at the bottom, sitting on the build plate).
#
# depth_mm/height_mm are swapped from the more literal axis-name
# mapping, per explicit request: depth_mm drives the vertical
# extrusion (Z, the riser's rise) and height_mm drives the U-channel's
# front-to-back footprint (Y).

import math

from . import geo_helpers as gh

MIN_WIDTH_MM = 30.0
MIN_DEPTH_MM = 20.0
MIN_HEIGHT_MM = 20.0
MIN_WALL_THICKNESS_MM = 1.0
MIN_FILLET_MARGIN_MM = 0.5

# Corner rounding is a cosmetic/practical detail, not something a user
# needs to tune per-part - the INNER radius scales proportionally with
# wall_thickness_mm (ratio frozen, not user-tunable), clamped down
# automatically if the part's own interior is too small to fit it. A
# thicker wall gets a proportionally bigger, still-nicely-rounded
# interior corner instead of either shrinking toward a point (an
# earlier version froze the OUTER radius, which had that problem) or
# staying visually undersized next to a much thicker wall (a version
# in between froze the INNER radius instead, fixing the shrinking but
# leaving a thick wall looking under-rounded on the inside). The RATIO
# is chosen so wall_thickness_mm's own default (2.4mm) reproduces the
# same 3.0mm inner radius those earlier versions used, for continuity.
# The outer radius is always inner + wall_thickness_mm (concentric
# arcs sharing one center).
INNER_FILLET_RATIO = 1.25
FILLET_SEGS = 12


def shelf_riser_params(width_mm, depth_mm, height_mm, wall_thickness_mm):
    """Validate parameters and return derived geometry values.

    depth_mm drives the vertical rise, height_mm drives the front-to-
    back footprint - see module docstring."""
    if width_mm < MIN_WIDTH_MM:
        raise ValueError("width_mm must be at least %.1fmm" % MIN_WIDTH_MM)
    if height_mm < MIN_DEPTH_MM:
        raise ValueError("height_mm must be at least %.1fmm" % MIN_DEPTH_MM)
    if depth_mm < MIN_HEIGHT_MM:
        raise ValueError("depth_mm must be at least %.1fmm" % MIN_HEIGHT_MM)
    if wall_thickness_mm < MIN_WALL_THICKNESS_MM:
        raise ValueError("wall_thickness_mm must be at least %.1fmm"
                         % MIN_WALL_THICKNESS_MM)

    x_out = width_mm / 2.0
    x_in = x_out - wall_thickness_mm
    if x_in <= 0:
        raise ValueError("width_mm too small for this wall_thickness_mm")

    y_back_out = height_mm
    y_back_in = y_back_out - wall_thickness_mm
    if y_back_in <= 0:
        raise ValueError("height_mm too small for this wall_thickness_mm")

    # inner fillet radius: scales with wall_thickness_mm, clamped down
    # if the interior itself is too small to fit it. The clamp reserves
    # MIN_FILLET_MARGIN_MM on each axis ITSELF (rather than clamping to
    # the exact boundary and checking margin afterward, which would
    # always measure exactly zero margin whenever that branch is what
    # bound the clamp) - without it, the left/right (or front/back)
    # fillet centers could collapse onto the centerline (cx_r/fc_y
    # below hitting 0) and self-overlap instead of forming a valid
    # corner. If there isn't even room for that margin, this is a
    # genuinely too-small-a-part-for-this-wall-thickness combination.
    if x_in <= MIN_FILLET_MARGIN_MM or y_back_in <= MIN_FILLET_MARGIN_MM:
        raise ValueError(
            "wall_thickness_mm too large relative to width_mm/height_mm - "
            "no room left for the corner fillets")
    inner_r = min(wall_thickness_mm * INNER_FILLET_RATIO,
                 x_in - MIN_FILLET_MARGIN_MM, y_back_in - MIN_FILLET_MARGIN_MM)
    fillet_r = inner_r + wall_thickness_mm

    fc_y = y_back_in - inner_r          # fillet center Y (either radius)
    cx_r = x_in - inner_r               # right-corner fillet center X

    return {
        "x_out": x_out, "x_in": x_in,
        "y_back_out": y_back_out, "y_back_in": y_back_in,
        "fillet_r": fillet_r, "inner_r": inner_r,
        "fc_y": fc_y, "cx_r": cx_r,
        "overall_height_mm": depth_mm,
    }


def _arc_interior(cx, cy, r, a0, a1, segs):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * k / segs)),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * k / segs)))
            for k in range(1, segs)]


def _outline_loop(p, z):
    """CCW single closed outline tracing inner-left-wall -> inner-left
    corner -> inner-back -> inner-right corner -> inner-right-wall ->
    (open front end) -> outer-right-wall -> outer-right corner ->
    outer-back -> outer-left corner -> outer-left-wall -> (open front
    end, closing back to the start)."""
    x_out, x_in = p["x_out"], p["x_in"]
    y_back_out, y_back_in = p["y_back_out"], p["y_back_in"]
    fillet_r, inner_r = p["fillet_r"], p["inner_r"]
    fc_y, cx_r = p["fc_y"], p["cx_r"]
    cx_l = -cx_r

    pts = [(-x_in, 0.0), (-x_in, fc_y)]
    pts += _arc_interior(cx_l, fc_y, inner_r, 180, 90, FILLET_SEGS)
    pts.append((cx_l, y_back_in))
    pts.append((cx_r, y_back_in))
    pts += _arc_interior(cx_r, fc_y, inner_r, 90, 0, FILLET_SEGS)
    pts.append((x_in, fc_y))
    pts.append((x_in, 0.0))
    pts.append((x_out, 0.0))
    pts.append((x_out, fc_y))
    pts += _arc_interior(cx_r, fc_y, fillet_r, 0, 90, FILLET_SEGS)
    pts.append((cx_r, y_back_out))
    pts.append((cx_l, y_back_out))
    pts += _arc_interior(cx_l, fc_y, fillet_r, 90, 180, FILLET_SEGS)
    pts.append((-x_out, fc_y))
    pts.append((-x_out, 0.0))
    return gh.Loop([(x, y, z) for (x, y) in pts], True, "u_outline")


def _shoelace_area(pts_xy):
    a = 0.0
    n = len(pts_xy)
    for i in range(n):
        x0, y0 = pts_xy[i]
        x1, y1 = pts_xy[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return abs(a) / 2.0


def generate_shelf_riser(width_mm=104.8, depth_mm=60.0, height_mm=42.4,
                         wall_thickness_mm=2.4):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = shelf_riser_params(width_mm, depth_mm, height_mm, wall_thickness_mm)
    bottom = _outline_loop(p, 0.0)
    top = _outline_loop(p, depth_mm)
    solid = (gh.bridge(bottom, top)
            + gh.cap(bottom, fill_mode="ngon", reverse=True)
            + gh.cap(top, fill_mode="ngon"))
    verts, edges, faces = solid.to_pydata()
    return verts, [], faces


def expected_volume_mm3(width_mm, depth_mm, height_mm, wall_thickness_mm):
    """Exact analytic volume: outline cross-sectional area (shoelace,
    using the SAME outline construction the generator itself uses) x
    the vertical rise (depth_mm) - pure Python, cross-checked against
    Mesh.signed_volume() in dev/run_tests.py."""
    p = shelf_riser_params(width_mm, depth_mm, height_mm, wall_thickness_mm)
    bottom = _outline_loop(p, 0.0)
    area = _shoelace_area([(x, y) for x, y, z in bottom.verts])
    return area * depth_mm
