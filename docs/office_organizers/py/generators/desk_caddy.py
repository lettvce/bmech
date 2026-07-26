# Desk Caddy generator - parametric version of modeled_orgs/desk_caddy.blend.
#
# A linear row of N raked square compartments: each cell's front wall is
# shorter than its back wall, with the two side walls sloping linearly
# between them - an easy-access scooped opening, repeated side by side,
# replacing the earlier "solid block with tapered round pockets" design
# (that reference geometry had no textural character of its own; this
# does, and it's shared with every cell rather than drilled per-hole).
#
# Every cell shares the SAME front/back height (unlike tray_cubbies,
# where side columns could differ from the middle one), so the wall
# between two adjacent cells needs no special per-neighbor handling -
# it's the same ramped profile on both sides, matching wall_cubby's own
# _wedge_x shape. The front and back margins (the solid strips before
# the first row and after it, y=[0,wall] and y=[depth-wall,depth]) are
# uniformly flat regardless of x, at the front/back cell height
# respectively - like tray_cubbies' land segments, they need no
# internal subdivision except to avoid a T-junction against the per-
# cell/per-divider structure they border.
#
# X = width (spans all N cells), Y = depth (0 at the front, increasing
# toward the back), Z = height (0 at the bottom).

from ._geo import VertPool, variable_grid_boundaries, face_x, face_y, face_z

MIN_WIDTH_MM = 30.0
MIN_DEPTH_MM = 30.0
MIN_CELL_WIDTH_MM = 4.0
MIN_POCKET_DEPTH_MM = 2.0


def _ramp_face_x(pool, x, y0, y1, z0, z1_at_y0, z1_at_y1, plus):
    """Vertical-ish quad at fixed x, flat bottom at z0, but a sloped
    top edge running from z1_at_y0 (at y0) to z1_at_y1 (at y1) - a
    cell's own ramped side wall. Planar by construction (every point
    shares the same x)."""
    pts = [(x, y0, z0), (x, y1, z0), (x, y1, z1_at_y1), (x, y0, z1_at_y0)]
    if not plus:
        pts.reverse()
    return tuple(pool.add(*p) for p in pts)


def _ramp_face_z(pool, x0, x1, y0, y1, z_at_y0, z_at_y1, up):
    """Roughly-horizontal quad spanning x0..x1, y0..y1, whose height
    depends only on y (z_at_y0 at y0, z_at_y1 at y1) - a divider's own
    ramped top surface. Planar since z is constant across x for any
    given y."""
    pts = [(x0, y0, z_at_y0), (x1, y0, z_at_y0),
          (x1, y1, z_at_y1), (x0, y1, z_at_y1)]
    if not up:
        pts.reverse()
    return tuple(pool.add(*p) for p in pts)


def desk_caddy_params(width_mm, depth_mm, compartment_count, front_height_mm,
                      height_mm, wall_thickness_mm, base_thickness_mm):
    """Validate parameters and return derived geometry values.

    height_mm and front_height_mm are both ABSOLUTE Z heights (like
    every other generator's "overall height" - measured from the build
    plate, not from the floor), with height_mm being the back wall's
    (taller) height and front_height_mm the shorter front wall's - the
    two side walls of each cell ramp linearly between them."""
    if width_mm < MIN_WIDTH_MM:
        raise ValueError("width_mm must be at least %.1fmm" % MIN_WIDTH_MM)
    if depth_mm < MIN_DEPTH_MM:
        raise ValueError("depth_mm must be at least %.1fmm" % MIN_DEPTH_MM)
    if compartment_count < 1:
        raise ValueError("compartment_count must be >= 1")
    if wall_thickness_mm <= 0 or base_thickness_mm <= 0:
        raise ValueError("thicknesses must be positive")
    if height_mm < front_height_mm:
        raise ValueError(
            "height_mm (back wall) cannot be shorter than front_height_mm")

    front_pocket_depth = front_height_mm - base_thickness_mm
    if front_pocket_depth < MIN_POCKET_DEPTH_MM:
        raise ValueError(
            "front_height_mm minus base_thickness_mm leaves no usable "
            "pocket depth")

    cavity_depth_mm = depth_mm - 2.0 * wall_thickness_mm
    if cavity_depth_mm < MIN_CELL_WIDTH_MM:
        raise ValueError(
            "depth_mm too small for this wall_thickness_mm")

    cell_width_mm = ((width_mm - (compartment_count + 1) * wall_thickness_mm)
                     / compartment_count)
    if cell_width_mm < MIN_CELL_WIDTH_MM:
        raise ValueError(
            "too many compartments (or caddy too narrow) for this wall "
            "thickness - each cell would be under %.1fmm wide"
            % MIN_CELL_WIDTH_MM)

    return {
        "cell_width_mm": cell_width_mm,
        "cavity_depth_mm": cavity_depth_mm,
    }


def generate_desk_caddy(width_mm=120.0, depth_mm=40.0, compartment_count=4,
                        front_height_mm=20.0, height_mm=30.0,
                        wall_thickness_mm=2.0, base_thickness_mm=2.0):
    """Returns (verts, edges, faces) in mm, ready for mesh.from_pydata()."""
    p = desk_caddy_params(width_mm, depth_mm, compartment_count,
                          front_height_mm, height_mm, wall_thickness_mm,
                          base_thickness_mm)
    B = base_thickness_mm
    zf, zb = front_height_mm, height_mm  # absolute Z tops

    xs = variable_grid_boundaries([p["cell_width_mm"]] * compartment_count,
                                  wall_thickness_mm)
    ys = [0.0, wall_thickness_mm, depth_mm - wall_thickness_mm, depth_mm]

    pool = VertPool()
    faces = []

    for kx in range(len(xs) - 1):
        x0, x1 = xs[kx], xs[kx + 1]
        is_cell = kx % 2 == 1
        # front margin: always flat land at the front wall's own height
        faces.append(face_z(pool, x0, x1, ys[0], ys[1], zf, up=True))
        # back margin: always flat land at the back wall's own height
        faces.append(face_z(pool, x0, x1, ys[2], ys[3], zb, up=True))
        if is_cell:
            faces.append(face_z(pool, x0, x1, ys[1], ys[2], B, up=True))
            faces.append(face_y(pool, ys[1], x0, x1, B, zf, plus=True))
            faces.append(face_y(pool, ys[2], x0, x1, B, zb, plus=False))
            faces.append(_ramp_face_x(pool, x0, ys[1], ys[2], B, zf, zb,
                                      plus=True))
            faces.append(_ramp_face_x(pool, x1, ys[1], ys[2], B, zf, zb,
                                      plus=False))
        else:
            # divider (or outer edge strip): side faces already come
            # from the neighboring cell(s)' own pocket-wall loop above
            # (at exactly this x-position) - only the ramped top needs
            # building here.
            faces.append(_ramp_face_z(pool, x0, x1, ys[1], ys[2], zf, zb,
                                      up=True))

    # Outer perimeter (bottom slab + 4 walls), one ring: front/back
    # outer walls are flat (their whole span sits within one flat
    # margin band); left/right outer walls ramp the same way every
    # divider does, using the same 4 y-ticks (the ramp is linear, so
    # its two endpoints are all the geometry needs).
    def h_at(y):
        if y <= ys[1] + 1e-9:
            return zf
        if y >= ys[2] - 1e-9:
            return zb
        t = (y - ys[1]) / (ys[2] - ys[1])
        return zf + (zb - zf) * t

    boundary_xy = []
    for k in range(len(xs) - 1):
        boundary_xy.append((xs[k], ys[0]))
    for k in range(len(ys) - 1):
        boundary_xy.append((xs[-1], ys[k]))
    for k in range(len(xs) - 1, 0, -1):
        boundary_xy.append((xs[k], ys[-1]))
    for k in range(len(ys) - 1, 0, -1):
        boundary_xy.append((xs[0], ys[k]))

    top_ring = [pool.add(px, py, h_at(py)) for (px, py) in boundary_xy]
    bot_ring = [pool.add(px, py, 0.0) for (px, py) in boundary_xy]
    L = len(top_ring)
    for k in range(L):
        k1 = (k + 1) % L
        faces.append((bot_ring[k], bot_ring[k1], top_ring[k1], top_ring[k]))
    faces.append(tuple(reversed(bot_ring)))  # solid bottom slab

    return pool.verts, [], faces


def expected_volume_mm3(width_mm, depth_mm, compartment_count,
                        front_height_mm, height_mm, wall_thickness_mm,
                        base_thickness_mm):
    """Exact analytic volume (axis-aligned/linear-ramp geometry, no
    approximation)."""
    p = desk_caddy_params(width_mm, depth_mm, compartment_count,
                          front_height_mm, height_mm, wall_thickness_mm,
                          base_thickness_mm)
    cavity_depth = p["cavity_depth_mm"]
    wall = wall_thickness_mm
    n = compartment_count

    # front_height_mm/height_mm are absolute Z tops; the actual wall
    # material above the floor is that minus base_thickness_mm.
    wall_h_front = front_height_mm - base_thickness_mm
    wall_h_back = height_mm - base_thickness_mm

    # front_margin/back_margin span the FULL width (every x-segment,
    # cell or divider) - a cell's own front/back wall is the same
    # solid material as that margin's own back edge, not extra volume
    # on top of it, so there's no separate per-cell wall term here.
    bottom_slab = width_mm * depth_mm * base_thickness_mm
    front_margin = width_mm * wall * wall_h_front
    back_margin = width_mm * wall * wall_h_back

    # only DIVIDER segments contribute solid volume in the cavity band
    # (a ramped wedge); cell segments are open pockets there (zero).
    n_dividers = n + 1
    dividers_width = n_dividers * wall
    wedge_vol = (dividers_width * cavity_depth
                * (wall_h_front + wall_h_back) / 2.0)

    return bottom_slab + front_margin + back_margin + wedge_vol
