# File Sorter generator - pure from_pydata geometry, no bpy imports.
#
# A staircase-profile paper/mail sorter: slot_count+1 vertical dividers on
# a common base, each one wall_thickness_mm apart from the last and one
# step_rise_mm taller, so the divider tops ascend like stairs and every
# pocket's label edge stays visible. Dividers are kept perfectly vertical
# (not leaned back) so the whole part prints with zero overhangs - the
# staircase look comes entirely from the ascending heights, not from any
# angled face.
#
# The side silhouette (in the XZ plane: X=front-to-back depth, Z=height)
# is a single closed "comb" outline - the pocket gaps are concave notches
# in that outline, not separate enclosed holes - which lets the whole
# solid be built with one extrude-along-Y call.

from ._geo import VertPool, grid_boundaries, extrude_profile_along_y

MIN_POCKET_DEPTH_MM = 6.0
MIN_WALL_HEIGHT_MM = 15.0


def file_sorter_params(width_mm, depth_mm, slot_count, wall_thickness_mm,
                       base_thickness_mm, min_wall_height_mm, step_rise_mm):
    """Validate parameters and return derived geometry values."""
    if slot_count < 2:
        raise ValueError("slot_count must be >= 2")
    if width_mm <= 0 or depth_mm <= 0:
        raise ValueError("dimensions must be positive")
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")
    if step_rise_mm < 0:
        raise ValueError("step_rise_mm must be >= 0")
    if min_wall_height_mm < MIN_WALL_HEIGHT_MM:
        raise ValueError(
            "min wall height must be at least %.1fmm to hold papers"
            % MIN_WALL_HEIGHT_MM)

    n_dividers = slot_count + 1
    pocket_depth = (depth_mm - n_dividers * wall_thickness_mm) / slot_count
    if pocket_depth < MIN_POCKET_DEPTH_MM:
        raise ValueError(
            "too many slots (or sorter too shallow) for this wall "
            "thickness - each pocket would be under %.1fmm front-to-back"
            % MIN_POCKET_DEPTH_MM)

    heights = [base_thickness_mm + min_wall_height_mm + i * step_rise_mm
              for i in range(n_dividers)]
    return {
        "n_dividers": n_dividers,
        "pocket_depth_mm": pocket_depth,
        "divider_heights_mm": heights,
        "overall_height_mm": heights[-1],
    }


def _comb_outline(xs, base_thickness_mm, heights):
    """Build the closed staircase silhouette in the XZ plane, CCW."""
    n_div = len(heights)
    pts = [(0.0, 0.0), (xs[-1], 0.0), (xs[-1], heights[-1])]
    for di in range(n_div - 1, -1, -1):
        x0 = xs[2 * di]
        pts.append((x0, heights[di]))
        if di > 0:
            gap_x0 = xs[2 * di - 1]
            pts.append((x0, base_thickness_mm))
            pts.append((gap_x0, base_thickness_mm))
            pts.append((gap_x0, heights[di - 1]))
    return pts


def generate_file_sorter(width_mm=230.0, depth_mm=150.0, slot_count=5,
                         wall_thickness_mm=2.0, base_thickness_mm=3.0,
                         min_wall_height_mm=25.0, step_rise_mm=15.0):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = file_sorter_params(width_mm, depth_mm, slot_count, wall_thickness_mm,
                           base_thickness_mm, min_wall_height_mm,
                           step_rise_mm)
    xs = grid_boundaries(depth_mm, slot_count, wall_thickness_mm,
                         p["pocket_depth_mm"])
    outline = _comb_outline(xs, base_thickness_mm, p["divider_heights_mm"])

    pool = VertPool()
    faces = extrude_profile_along_y(pool, outline, 0.0, width_mm)
    return pool.verts, [], faces


def expected_volume_mm3(width_mm, depth_mm, slot_count, wall_thickness_mm,
                        base_thickness_mm, min_wall_height_mm, step_rise_mm):
    """Exact analytic volume: cross-section area (base slab + each
    divider's rectangle above the base) times the extrusion width."""
    p = file_sorter_params(width_mm, depth_mm, slot_count, wall_thickness_mm,
                           base_thickness_mm, min_wall_height_mm,
                           step_rise_mm)
    area = depth_mm * base_thickness_mm
    area += wall_thickness_mm * sum(h - base_thickness_mm
                                    for h in p["divider_heights_mm"])
    return area * width_mm
