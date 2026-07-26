# Generic Tray generator - pure from_pydata geometry, no bpy imports.
#
# The simplest organizer in this project: a single open rectangular
# pocket - four walls and a floor, no internal dividers at all.
# Geometrically this is exactly drawer_divider's own construction with
# one column and one row, so rather than re-deriving that geometry
# (and its winding/manifold proof) from scratch, this module just
# translates its user-facing width_mm/depth_mm (the tray's OVERALL
# size) into drawer_divider's interior column/row size and delegates.
#
# width_mm/depth_mm round-trip exactly through that translation:
# drawer_divider derives its own width_mm as
# sum(col_widths_mm) + (cols+1)*wall_thickness_mm, which for a single
# column of size (width_mm - 2*wall_thickness_mm) comes back out to
# width_mm again, bit-for-bit (plain addition/subtraction, no
# accumulated error) - so there's no discrepancy between what the user
# requests and what the mesh actually measures.

from . import drawer_divider

MIN_INTERIOR_MM = drawer_divider.MIN_POCKET_DIM_MM
MIN_POCKET_DEPTH_MM = drawer_divider.MIN_POCKET_DEPTH_MM


def generic_tray_params(width_mm, depth_mm, height_mm, wall_thickness_mm,
                        base_thickness_mm):
    """Validate parameters and return derived geometry values."""
    if width_mm <= 0 or depth_mm <= 0:
        raise ValueError("width_mm and depth_mm must be positive")
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")

    interior_width_mm = width_mm - 2.0 * wall_thickness_mm
    interior_depth_mm = depth_mm - 2.0 * wall_thickness_mm
    if interior_width_mm < MIN_INTERIOR_MM or interior_depth_mm < MIN_INTERIOR_MM:
        raise ValueError(
            "width/depth too small for this wall thickness - the "
            "interior would be under %.1fmm" % MIN_INTERIOR_MM)

    dd = drawer_divider.drawer_divider_params(
        col_widths_mm=[interior_width_mm], row_depths_mm=[interior_depth_mm],
        height_mm=height_mm, wall_thickness_mm=wall_thickness_mm,
        base_thickness_mm=base_thickness_mm)

    return {
        "width_mm": dd["width_mm"],
        "depth_mm": dd["depth_mm"],
        "pocket_depth": dd["pocket_depth"],
        "interior_width_mm": interior_width_mm,
        "interior_depth_mm": interior_depth_mm,
    }


def generate_generic_tray(width_mm=100.0, depth_mm=80.0, height_mm=40.0,
                          wall_thickness_mm=1.6, base_thickness_mm=2.4):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = generic_tray_params(width_mm, depth_mm, height_mm, wall_thickness_mm,
                            base_thickness_mm)
    return drawer_divider.generate_drawer_divider(
        col_widths_mm=[p["interior_width_mm"]],
        row_depths_mm=[p["interior_depth_mm"]],
        height_mm=height_mm, wall_thickness_mm=wall_thickness_mm,
        base_thickness_mm=base_thickness_mm)


def expected_volume_mm3(width_mm, depth_mm, height_mm, wall_thickness_mm,
                        base_thickness_mm):
    """Exact analytic volume (axis-aligned geometry, no approximation)."""
    p = generic_tray_params(width_mm, depth_mm, height_mm, wall_thickness_mm,
                            base_thickness_mm)
    return drawer_divider.expected_volume_mm3(
        col_widths_mm=[p["interior_width_mm"]],
        row_depths_mm=[p["interior_depth_mm"]],
        height_mm=height_mm, wall_thickness_mm=wall_thickness_mm,
        base_thickness_mm=base_thickness_mm)
