# Drawer Divider generator - pure from_pydata geometry, no bpy imports.
#
# A solid rectangular block with a grid of rectangular pockets bored
# from the top - each COLUMN has its own width and each ROW its own
# depth (not a uniform cols x rows grid of identical cells), leaving
# wall_thickness_mm between adjacent pockets and around the perimeter,
# and base_thickness_mm of solid floor underneath. Overall width/depth
# are DERIVED from the column widths / row depths plus the walls
# between and around them, rather than being independent inputs the
# column/row sizes have to be validated against.
#
# All geometry is axis-aligned, so (like the original uniform-grid
# version) this is exact - no polygon-approximation error - useful as a
# volume cross-check for the shared vertex-pool/DFM machinery.

from ._geo import VertPool, variable_grid_boundaries, face_x, face_y, face_z

MIN_POCKET_DIM_MM = 4.0
MIN_POCKET_DEPTH_MM = 2.0


def drawer_divider_params(col_widths_mm, row_depths_mm, height_mm,
                          wall_thickness_mm, base_thickness_mm):
    """Validate parameters and return derived geometry values."""
    if not col_widths_mm or not row_depths_mm:
        raise ValueError("need at least one column and one row")
    if any(w <= 0 for w in col_widths_mm):
        raise ValueError("column widths must be positive")
    if any(d <= 0 for d in row_depths_mm):
        raise ValueError("row depths must be positive")
    if min(col_widths_mm) < MIN_POCKET_DIM_MM:
        raise ValueError(
            "every column must be at least %.1fmm wide" % MIN_POCKET_DIM_MM)
    if min(row_depths_mm) < MIN_POCKET_DIM_MM:
        raise ValueError(
            "every row must be at least %.1fmm deep" % MIN_POCKET_DIM_MM)
    if height_mm <= 0:
        raise ValueError("height_mm must be positive")
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")

    pocket_depth = height_mm - base_thickness_mm
    if pocket_depth < MIN_POCKET_DEPTH_MM:
        raise ValueError(
            "height minus base thickness leaves no usable pocket depth")

    cols = len(col_widths_mm)
    rows = len(row_depths_mm)
    width_mm = sum(col_widths_mm) + (cols + 1) * wall_thickness_mm
    depth_mm = sum(row_depths_mm) + (rows + 1) * wall_thickness_mm

    return {
        "width_mm": width_mm,
        "depth_mm": depth_mm,
        "pocket_depth": pocket_depth,
        "cols": cols,
        "rows": rows,
    }


def generate_drawer_divider(col_widths_mm=(36.0, 36.0, 36.0),
                            row_depths_mm=(46.0, 46.0), height_mm=40.0,
                            wall_thickness_mm=1.6, base_thickness_mm=2.4):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    drawer_divider_params(col_widths_mm, row_depths_mm, height_mm,
                          wall_thickness_mm, base_thickness_mm)
    H = height_mm
    B = base_thickness_mm

    xs = variable_grid_boundaries(col_widths_mm, wall_thickness_mm)
    ys = variable_grid_boundaries(row_depths_mm, wall_thickness_mm)

    pool = VertPool()
    faces = []

    # Top "land" surface: every grid cell except the pocket cells (odd
    # x-segment AND odd y-segment) gets a quad at z=height.
    for kx in range(len(xs) - 1):
        pocket_col = kx % 2 == 1
        for ky in range(len(ys) - 1):
            pocket_row = ky % 2 == 1
            if pocket_col and pocket_row:
                continue
            faces.append(face_z(pool, xs[kx], xs[kx + 1],
                                ys[ky], ys[ky + 1], H, up=True))

    # Outer perimeter, subdivided at every grid tick so its edges line up
    # exactly with the top-land grid above (a plain 4-corner quad would
    # create a T-junction against the subdivided top surface).
    boundary_xy = []
    for k in range(len(xs) - 1):
        boundary_xy.append((xs[k], ys[0]))
    for k in range(len(ys) - 1):
        boundary_xy.append((xs[-1], ys[k]))
    for k in range(len(xs) - 1, 0, -1):
        boundary_xy.append((xs[k], ys[-1]))
    for k in range(len(ys) - 1, 0, -1):
        boundary_xy.append((xs[0], ys[k]))

    top_ring = [pool.add(px, py, H) for (px, py) in boundary_xy]
    bot_ring = [pool.add(px, py, 0.0) for (px, py) in boundary_xy]
    L = len(top_ring)
    for k in range(L):
        k1 = (k + 1) % L
        faces.append((bot_ring[k], bot_ring[k1], top_ring[k1], top_ring[k]))
    faces.append(tuple(reversed(bot_ring)))  # solid bottom slab

    # Each pocket: 4 walls + floor.
    for kx in range(1, len(xs) - 1, 2):
        px0, px1 = xs[kx], xs[kx + 1]
        for ky in range(1, len(ys) - 1, 2):
            py0, py1 = ys[ky], ys[ky + 1]
            faces.append(face_x(pool, px0, py0, py1, B, H, plus=True))
            faces.append(face_x(pool, px1, py0, py1, B, H, plus=False))
            faces.append(face_y(pool, py0, px0, px1, B, H, plus=True))
            faces.append(face_y(pool, py1, px0, px1, B, H, plus=False))
            faces.append(face_z(pool, px0, px1, py0, py1, B, up=True))

    return pool.verts, [], faces


def expected_volume_mm3(col_widths_mm, row_depths_mm, height_mm,
                        wall_thickness_mm, base_thickness_mm):
    """Exact analytic volume (axis-aligned geometry, no approximation).
    sum(w*d for w in widths for d in depths) == (sum widths)*(sum
    depths) by the distributive law - same total pocket footprint area
    either way, just summed per-cell here to mirror the mesh's own
    per-pocket construction."""
    p = drawer_divider_params(col_widths_mm, row_depths_mm, height_mm,
                              wall_thickness_mm, base_thickness_mm)
    block = p["width_mm"] * p["depth_mm"] * height_mm
    pockets = sum(w * d * p["pocket_depth"]
                 for w in col_widths_mm for d in row_depths_mm)
    return block - pockets
