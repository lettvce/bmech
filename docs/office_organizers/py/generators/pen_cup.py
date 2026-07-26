# Pen Cup generator - pure from_pydata geometry, no bpy imports.
#
# A single hollow cylinder (solid tube + floor) whose top rim is cut by
# one flat, tilted plane instead of a horizontal one - the "angled mouth"
# scoop shape common on desktop pen cups. The cut plane is defined as
#   z_cut(x) = min_height_mm + (x + outer_radius_mm) * tan(mouth_angle_deg)
# so the lowest point of the rim (at x = -outer_radius, the very front of
# the outer wall) sits at exactly min_height_mm, and the plane rises
# linearly toward the back (x = +outer_radius). The same absolute plane
# trims both the outer wall and the inner bore, so the rim reads as one
# continuous bevel rather than a stepped edge.
#
# The tilt only ever pushes the rim's own normal a few degrees off +Z (up
# to mouth_angle_deg from vertical); the walls stay perfectly cylindrical
# (radial normals). Nothing in this shape ever faces downward past
# horizontal, so it prints with zero overhangs regardless of the chosen
# angle.

import math

from ._geo import VertPool

MIN_DIAMETER_MM = 20.0
MIN_INNER_RADIUS_MM = 5.0
MIN_POCKET_DEPTH_MM = 15.0


def pen_cup_params(diameter_mm, min_height_mm, mouth_angle_deg,
                   wall_thickness_mm, base_thickness_mm, segments):
    """Validate parameters and return derived geometry values."""
    if diameter_mm < MIN_DIAMETER_MM:
        raise ValueError("diameter must be at least %.1fmm" % MIN_DIAMETER_MM)
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")
    if min_height_mm <= 0:
        raise ValueError("min_height_mm must be positive")
    if not (0.0 <= mouth_angle_deg < 90.0):
        raise ValueError("mouth_angle_deg must be within [0, 90)")
    if segments < 8:
        raise ValueError("segments must be >= 8")

    R = diameter_mm / 2.0
    r = R - wall_thickness_mm
    if r < MIN_INNER_RADIUS_MM:
        raise ValueError(
            "wall too thick (or diameter too small) - interior radius "
            "would be under %.1fmm" % MIN_INNER_RADIUS_MM)

    tan_a = math.tan(math.radians(mouth_angle_deg))
    z_center = min_height_mm + R * tan_a
    front_inner_height = min_height_mm + wall_thickness_mm * tan_a
    pocket_depth_at_front = front_inner_height - base_thickness_mm
    if pocket_depth_at_front < MIN_POCKET_DEPTH_MM:
        raise ValueError(
            "front of the mouth is too low for this base thickness - "
            "interior depth at the front would be under %.1fmm"
            % MIN_POCKET_DEPTH_MM)

    return {
        "outer_radius_mm": R,
        "inner_radius_mm": r,
        "z_center_mm": z_center,
        "max_height_mm": min_height_mm + diameter_mm * tan_a,
        "front_inner_height_mm": front_inner_height,
        "pocket_depth_at_front_mm": pocket_depth_at_front,
    }


def generate_pen_cup(diameter_mm=80.0, min_height_mm=90.0,
                     mouth_angle_deg=20.0, wall_thickness_mm=2.0,
                     base_thickness_mm=3.0, segments=48):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = pen_cup_params(diameter_mm, min_height_mm, mouth_angle_deg,
                       wall_thickness_mm, base_thickness_mm, segments)
    R, r = p["outer_radius_mm"], p["inner_radius_mm"]
    tan_a = math.tan(math.radians(mouth_angle_deg))

    def z_cut(x):
        return min_height_mm + (x + R) * tan_a

    S = segments
    thetas = [2.0 * math.pi * i / S for i in range(S)]
    cos_t = [math.cos(t) for t in thetas]
    sin_t = [math.sin(t) for t in thetas]

    pool = VertPool()
    outer_bot = [pool.add(R * cos_t[i], R * sin_t[i], 0.0) for i in range(S)]
    outer_top = [pool.add(R * cos_t[i], R * sin_t[i], z_cut(R * cos_t[i]))
                for i in range(S)]
    inner_top = [pool.add(r * cos_t[i], r * sin_t[i], z_cut(r * cos_t[i]))
                for i in range(S)]
    inner_bot = [pool.add(r * cos_t[i], r * sin_t[i], base_thickness_mm)
                for i in range(S)]

    faces = []
    for k in range(S):
        k1 = (k + 1) % S
        faces.append((outer_bot[k], outer_bot[k1],
                      outer_top[k1], outer_top[k]))       # outer wall
        faces.append((outer_top[k], outer_top[k1],
                      inner_top[k1], inner_top[k]))        # tilted rim
        faces.append((inner_top[k], inner_top[k1],
                      inner_bot[k1], inner_bot[k]))        # bore wall
    faces.append(tuple(reversed(outer_bot)))                # bottom slab
    faces.append(tuple(inner_bot))                          # bore floor

    return pool.verts, [], faces


def _regular_polygon_area(radius, segments):
    return 0.5 * segments * radius * radius * math.sin(2.0 * math.pi / segments)


def expected_volume_mm3(diameter_mm, min_height_mm, mouth_angle_deg,
                        wall_thickness_mm, base_thickness_mm, segments):
    """Exact analytic volume matching the generated N-gon mesh.

    For a solid with a regular-polygon footprint centered at the origin
    and a planar (possibly tilted) top, the volume equals footprint area
    times the top plane's height AT THE CENTROID - and a regular polygon
    centered at the origin always has its centroid at the origin (fixed
    point of the polygon's rotational symmetry), regardless of whether
    segments is odd or even. So volume = area * z_cut(0) = area *
    z_center exactly, for both the outer solid and the inner bore.
    """
    p = pen_cup_params(diameter_mm, min_height_mm, mouth_angle_deg,
                       wall_thickness_mm, base_thickness_mm, segments)
    R, r = p["outer_radius_mm"], p["inner_radius_mm"]
    z_center = p["z_center_mm"]
    outer_area = _regular_polygon_area(R, segments)
    inner_area = _regular_polygon_area(r, segments)
    v_outer_full = outer_area * z_center
    v_bore = inner_area * (z_center - base_thickness_mm)
    return v_outer_full - v_bore
