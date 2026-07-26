# Tray Cubbies generator - pure from_pydata geometry, no bpy imports.
#
# Fixed 3-column layout: a middle column with ONE tall row, flanked by
# two IDENTICAL side columns (same width, same row count, same row
# depth - mirrored left and right, made of identical stacked boxes).
# The middle row's depth is DERIVED to match the side columns' own
# total depth, not an independent input, so there's no parameter
# combination that leaves the tray non-rectangular - unlike an earlier,
# more general version of this generator (arbitrary column count, each
# with its own independent row list) which needed its own "columns
# must reach the same depth" validation error just to keep that
# freedom self-consistent, plus a nested-CollectionProperty Blender UI
# (a column list, each with its own row list) to expose it. That
# generality solved a problem this project didn't actually have - every
# real use case was this one symmetric shape - so it's gone; this
# version's UI is as plain as any other generator's sliders.
#
# The mesh-construction approach (per-column top-land/pocket loop,
# outer wall-strip top faces, an n-gon divider top walking both
# neighbors' row ticks to avoid a T-junction, one ring for the outer
# perimeter) is unchanged from that general version and stays written
# generically over `n` columns internally - it was already proven
# correct for arbitrary column/row combinations, so reusing it here
# (always called with the fixed 3-column layout) carries far less risk
# than hand-unrolling a new left/middle/right-specific version.

from ._geo import VertPool, variable_grid_boundaries, face_x, face_y, face_z

MIN_POCKET_DIM_MM = 4.0
MIN_POCKET_DEPTH_MM = 2.0


def tray_cubbies_params(side_width_mm, middle_width_mm, row_count,
                        row_depth_mm, height_mm, wall_thickness_mm,
                        base_thickness_mm):
    """Validate parameters and return derived geometry values."""
    if side_width_mm < MIN_POCKET_DIM_MM:
        raise ValueError(
            "side columns must be at least %.1fmm wide" % MIN_POCKET_DIM_MM)
    if middle_width_mm < MIN_POCKET_DIM_MM:
        raise ValueError(
            "middle column must be at least %.1fmm wide" % MIN_POCKET_DIM_MM)
    if row_count < 1:
        raise ValueError("need at least one row per side column")
    if row_depth_mm < MIN_POCKET_DIM_MM:
        raise ValueError(
            "each row must be at least %.1fmm deep" % MIN_POCKET_DIM_MM)
    if height_mm <= 0:
        raise ValueError("height_mm must be positive")
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")

    pocket_depth = height_mm - base_thickness_mm
    if pocket_depth < MIN_POCKET_DEPTH_MM:
        raise ValueError(
            "height minus base thickness leaves no usable pocket depth")

    # side columns' own total depth (row_count identical rows + the
    # walls between/around them) - the middle column's single row is
    # sized to reach exactly this, so the tray always stays rectangular.
    depth_mm = (row_count * row_depth_mm
               + (row_count + 1) * wall_thickness_mm)
    middle_row_depth_mm = depth_mm - 2.0 * wall_thickness_mm
    width_mm = (2.0 * side_width_mm + middle_width_mm
               + 4.0 * wall_thickness_mm)

    return {
        "width_mm": width_mm,
        "depth_mm": depth_mm,
        "pocket_depth": pocket_depth,
        "middle_row_depth_mm": middle_row_depth_mm,
    }


def generate_tray_cubbies(side_width_mm=36.0, middle_width_mm=36.0,
                          row_count=2, row_depth_mm=22.2, height_mm=40.0,
                          wall_thickness_mm=1.6, base_thickness_mm=2.4):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = tray_cubbies_params(side_width_mm, middle_width_mm, row_count,
                            row_depth_mm, height_mm, wall_thickness_mm,
                            base_thickness_mm)
    H = height_mm
    B = base_thickness_mm
    depth_mm = p["depth_mm"]

    col_widths_mm = [side_width_mm, middle_width_mm, side_width_mm]
    col_rows_mm = [[row_depth_mm] * row_count,
                  [p["middle_row_depth_mm"]],
                  [row_depth_mm] * row_count]

    xs = variable_grid_boundaries(col_widths_mm, wall_thickness_mm)
    col_ys = [variable_grid_boundaries(rows, wall_thickness_mm)
             for rows in col_rows_mm]
    n = len(col_widths_mm)

    pool = VertPool()
    faces = []

    # --- each column's own top land + pockets (a 1-column drawer_divider,
    # scoped to that column's x-range and its OWN row boundaries) --------
    for i in range(n):
        x0, x1 = xs[2 * i + 1], xs[2 * i + 2]
        ys = col_ys[i]
        for ky in range(len(ys) - 1):
            y0, y1 = ys[ky], ys[ky + 1]
            if ky % 2 == 1:
                faces.append(face_x(pool, x0, y0, y1, B, H, plus=True))
                faces.append(face_x(pool, x1, y0, y1, B, H, plus=False))
                faces.append(face_y(pool, y0, x0, x1, B, H, plus=True))
                faces.append(face_y(pool, y1, x0, x1, B, H, plus=False))
                faces.append(face_z(pool, x0, x1, y0, y1, B, up=True))
            else:
                faces.append(face_z(pool, x0, x1, y0, y1, H, up=True))

    # --- outer wall-strip top faces (before column 0, after the last
    # column): always fully solid, subdivided to match the ONE column
    # that borders each of them (their other side is the tray's own
    # outer skin, already ticked the same way in the ring below) --------
    ys0 = col_ys[0]
    for ky in range(len(ys0) - 1):
        faces.append(face_z(pool, xs[0], xs[1], ys0[ky], ys0[ky + 1], H,
                            up=True))
    ysN = col_ys[-1]
    for ky in range(len(ysN) - 1):
        faces.append(face_z(pool, xs[-2], xs[-1], ysN[ky], ysN[ky + 1],
                            H, up=True))

    # --- interior column-divider walls: side faces already come from
    # each neighboring column's own per-row loop above (it places its
    # pocket walls exactly at the divider's x-position); only the top
    # needs building here, as one n-gon walking both neighbors' ticks --
    for j in range(1, n):
        wx0, wx1 = xs[2 * j], xs[2 * j + 1]
        ys_left = col_ys[j - 1]
        ys_right = col_ys[j]
        pts_xy = [(wx0, 0.0), (wx1, 0.0)]
        for y in ys_right[1:-1]:
            pts_xy.append((wx1, y))
        pts_xy.append((wx1, depth_mm))
        pts_xy.append((wx0, depth_mm))
        for y in reversed(ys_left[1:-1]):
            pts_xy.append((wx0, y))
        faces.append(tuple(pool.add(px, py, H) for (px, py) in pts_xy))

    # --- outer perimeter (bottom slab + walls), one ring: front/back
    # tick at the shared column boundaries, left/right tick at the
    # first/last column's own row boundaries (the only columns that
    # actually touch those two walls) - same ring-building technique as
    # drawer_divider, just a fancier boundary_xy construction ------------
    boundary_xy = []
    for k in range(len(xs) - 1):
        boundary_xy.append((xs[k], 0.0))
    ys_r = col_ys[-1]
    for k in range(len(ys_r) - 1):
        boundary_xy.append((xs[-1], ys_r[k]))
    for k in range(len(xs) - 1, 0, -1):
        boundary_xy.append((xs[k], depth_mm))
    ys_l = col_ys[0]
    for k in range(len(ys_l) - 1, 0, -1):
        boundary_xy.append((xs[0], ys_l[k]))

    top_ring = [pool.add(px, py, H) for (px, py) in boundary_xy]
    bot_ring = [pool.add(px, py, 0.0) for (px, py) in boundary_xy]
    L = len(top_ring)
    for k in range(L):
        k1 = (k + 1) % L
        faces.append((bot_ring[k], bot_ring[k1], top_ring[k1], top_ring[k]))
    faces.append(tuple(reversed(bot_ring)))  # solid bottom slab

    return pool.verts, [], faces


def expected_volume_mm3(side_width_mm, middle_width_mm, row_count,
                        row_depth_mm, height_mm, wall_thickness_mm,
                        base_thickness_mm):
    """Exact analytic volume (axis-aligned geometry, no approximation)."""
    p = tray_cubbies_params(side_width_mm, middle_width_mm, row_count,
                            row_depth_mm, height_mm, wall_thickness_mm,
                            base_thickness_mm)
    block = p["width_mm"] * p["depth_mm"] * height_mm
    side_pockets = (2.0 * side_width_mm * row_count * row_depth_mm
                   * p["pocket_depth"])
    middle_pocket = (middle_width_mm * p["middle_row_depth_mm"]
                     * p["pocket_depth"])
    return block - side_pockets - middle_pocket
