"""
Pure geometry core for ratchet_pawl.py - build_ratchet_wheel_mesh /
build_ratchet_pawl_mesh / build_ratchet_mechanism are plain Python +
geo_helpers math, zero bpy/bmesh/mathutils dependency. Extracted verbatim
from ratchet_pawl.py for the web configurator (Pyodide) - see that
file's own module docstring (the numbered "implementation notes") for
the full design rationale this reproduces unchanged.

`mathutils.Vector` is replaced with plain (x, y, z) tuples + a handful of
tiny vector helpers (_vec_add/_vec_sub/_vec_scale/_vec_length/
_vec_normalize) - the only bpy-adjacent dependency this file had, and the
one thing that kept it from running outside Blender at all.

Like the gears family's planetary sets, build_ratchet_mechanism returns
each part (wheel, pawl) with its own world-space location + Z rotation
already resolved, so a caller can rotate+translate each part's verts and
concatenate for one flat mesh - the same merge-in-Python trick
threaded_jar_set_core.py / planetary_gear_set_core.py use for their own
multi-part assemblies.
"""

import sys
import os
import math

GEO_HELPERS_DIR = r"C:\Users\Safu\OneDrive\Desktop\Blender\Mechanism Library\mechanisms_core"


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


HOLE_SEGMENTS = 32           # circle resolution for axle / pivot holes
PIVOT_BIAS_ANGLE_DEG = 35.0  # tangent-vs-radial blend for the auto pivot solve
PIVOT_PAD_MM = 2.0           # minimum wall thickness around a pivot/axle hole


# ── Plain-tuple vector helpers (replaces mathutils.Vector) ─────────────

def _vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _vec_length(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def _vec_normalize(a):
    length = _vec_length(a)
    if length < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


# ==============================================================================
# Pure-math helpers (verbatim from ratchet_pawl.py) - shared by all three
# operators so the wheel geometry is computed exactly once and can't
# drift between the standalone wheel operator and the combined mechanism
# operator.
# ==============================================================================

def solve_wheel_radii(sizing_mode, tooth_count, module, outer_diameter_mm,
                       tooth_depth_mm, tooth_depth_auto):
    """Returns (root_radius, outer_radius, tooth_depth_mm)."""
    if sizing_mode == 'MODULE':
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * module
        root_radius = (module * tooth_count) / 2.0
        outer_radius = root_radius + tooth_depth_mm
    else:  # 'OUTER_DIAMETER'
        outer_radius = outer_diameter_mm / 2.0
        effective_module = outer_diameter_mm / tooth_count  # approx, auto-depth only
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * effective_module
        root_radius = outer_radius - tooth_depth_mm
    return root_radius, outer_radius, tooth_depth_mm


def validate_wheel_params(tooth_count, root_radius, outer_radius, tooth_depth_mm, axle_hole_diameter_mm):
    """Raises ValueError with a human-readable message on any invalid combo.
    No clamping, ever -- the spec is explicit about that."""
    if tooth_count < 4:
        raise ValueError("tooth_count must be >= 4 (got %d)" % tooth_count)
    if tooth_depth_mm >= root_radius:
        raise ValueError(
            "tooth_depth_mm (%.3f) is >= root_radius (%.3f) -- this would collapse "
            "the wheel through its own center" % (tooth_depth_mm, root_radius)
        )
    if axle_hole_diameter_mm >= 2.0 * root_radius:
        raise ValueError(
            "axle_hole_diameter_mm (%.3f) is >= the wheel's root diameter (%.3f) -- "
            "the axle hole is bigger than the wheel" % (axle_hole_diameter_mm, 2.0 * root_radius)
        )


def build_wheel_profile_points(tooth_count, root_radius, outer_radius):
    """Builds the closed outer-boundary loop for one ratchet wheel as a flat
    list of (x, y) tuples, going around CCW starting at tooth 0's root
    vertex. v1.1: 2 vertices per tooth (root, tip), alternating root-tip-root.
    The back face is a single straight segment from tip_i to root_{i+1};
    its angle is no longer an input, it's whatever falls out of tooth_count,
    tooth_depth_mm, and the radii."""
    sector_angle = 2.0 * math.pi / tooth_count
    tooth_depth = outer_radius - root_radius

    points = []
    for i in range(tooth_count):
        theta = i * sector_angle
        points.append((root_radius * math.cos(theta), root_radius * math.sin(theta)))
        points.append((outer_radius * math.cos(theta), outer_radius * math.sin(theta)))

    return points, sector_angle, tooth_depth


def compute_back_face_angle_deg(root_radius, outer_radius, sector_angle):
    """v1.1: back_face_angle_deg is no longer a free input -- it's derived
    from the single-segment back face's actual geometry and reported back to
    the user. Computed at theta_i = 0 without loss of generality -- every
    tooth is identical by rotational symmetry, so there's only ever one
    answer for the whole wheel."""
    tip_x, tip_y = outer_radius, 0.0
    next_root_x = root_radius * math.cos(sector_angle)
    next_root_y = root_radius * math.sin(sector_angle)
    vx, vy = next_root_x - tip_x, next_root_y - tip_y
    length = math.hypot(vx, vy)
    if length < 1e-9:
        return 0.0  # degenerate (zero tooth depth) -- nothing to report
    # Inward radial direction at the tip (theta=0) is (-1, 0).
    cos_angle = max(-1.0, min(1.0, -vx / length))
    return math.degrees(math.acos(cos_angle))


def estimate_valley_width_mm(root_radius, outer_radius, sector_angle):
    """Approximate linear width of a tooth valley, measured at tooth
    mid-depth (the practical reference radius for "will a pawl tip of
    width X actually seat without binding")."""
    mid_radius = (root_radius + outer_radius) / 2.0
    return mid_radius * sector_angle / 2.0


def build_pawl_profile_points(arm_length, arm_width, tip_width, hole_radius):
    """Local-frame (pivot at origin, arm extends along +X) closed boundary
    loop for the pawl arm: straight-sided bar tapering to a wedge point."""
    pivot_pad = hole_radius + PIVOT_PAD_MM  # material behind the pivot axis so the hole isn't half-exposed
    taper_start_x = arm_length * 0.3
    wedge_length = max(tip_width, 1.0)  # guard against a degenerate zero-length wedge
    tip_base_x = arm_length - wedge_length
    if tip_base_x <= taper_start_x:
        # Defensive fallback only -- real validation of arm_length happens
        # in build_ratchet_pawl_mesh.
        tip_base_x = taper_start_x + 0.001

    return [
        (-pivot_pad, arm_width / 2.0),
        (taper_start_x, arm_width / 2.0),
        (tip_base_x, tip_width / 2.0),
        (arm_length, 0.0),            # wedge apex -- the actual tooth-valley contact point
        (tip_base_x, -tip_width / 2.0),
        (taper_start_x, -arm_width / 2.0),
        (-pivot_pad, -arm_width / 2.0),
    ]


# ==============================================================================
# geo_helpers DSL construction helpers (verbatim from ratchet_pawl.py)
# ==============================================================================

def _matched_hole_segments(outer_point_count, target=HOLE_SEGMENTS):
    """Smallest whole-number multiplier that brings outer_point_count up to
    at least `target`, while staying an EXACT multiple of outer_point_count
    -- see ratchet_pawl.py's own docstring for why that's what lets the
    hole circle be built at that same count for cap_annulus."""
    per_edge = max(1, math.ceil(target / outer_point_count))
    return per_edge, outer_point_count * per_edge


def build_profile_with_hole_mesh(outer_points, width_mm, inner_points=None):
    """Closed solid: extrude the outer 2D profile to width_mm and cap both
    ends, with an optional inner boundary cut through as a hole. See
    ratchet_pawl.py's own docstring for the full "must use the SAME
    resampled points for both wall and caps" rationale. Raises ValueError
    if the resulting mesh fails validation."""
    outer = gh.Loop([(x, y, 0.0) for x, y in outer_points], True, "outer")
    outer_top = gh.translate(outer, (0.0, 0.0, width_mm))
    outer_wall = gh.bridge(outer, outer_top)

    if inner_points:
        inner = gh.Loop([(x, y, 0.0) for x, y in inner_points], True, "inner")
        inner_top = gh.translate(inner, (0.0, 0.0, width_mm))
        inner_wall = gh.flip_normals(gh.bridge(inner, inner_top))
        bottom = gh.cap_annulus(outer, inner, reverse=True)
        top = gh.cap_annulus(outer_top, inner_top, reverse=False)
        mesh = outer_wall + inner_wall + bottom + top
    else:
        bottom = gh.cap(outer, fill_mode="earclip", reverse=True)
        top = gh.cap(outer_top, fill_mode="earclip")
        mesh = outer_wall + bottom + top

    if mesh.signed_volume() < 0.0:
        gh.flip_normals(mesh)
    report = mesh.validate(require_closed=True)
    if not report.is_valid:
        raise ValueError("Profile mesh failed validation: %s" % "; ".join(report.errors))
    return mesh


def _hole_outline(points, hole_radius):
    """Shared by wheel/pawl builds: resamples `points` (if needed) so its
    point count matches a hole circle built at HOLE_SEGMENTS-ish
    resolution, returning (outer_points, inner_points)."""
    outer_points = points
    inner_points = None
    if hole_radius > 0.0:
        per_edge, n = _matched_hole_segments(len(points))
        if per_edge > 1:
            resampled = gh.resample_loop(
                gh.Loop([(x, y, 0.0) for x, y in points], True, "outline"), per_edge)
            outer_points = [(x, y) for x, y, z in resampled.verts]
        inner_points = [(x, y) for x, y, z in gh.circle(hole_radius, len(outer_points)).verts]
    return outer_points, inner_points


def solve_pawl_pivot(wheel_center, root_radius, outer_radius, sector_angle,
                      tooth_count, engagement_side, engagement_angle_deg,
                      tip_engagement_depth_mm, pawl_arm_length_mm, tooth_depth_mm):
    """Auto-positioning solve per ratchet_pawl.py's own spec notes (decision
    #4 - a tangent-biased heuristic, not a rigorous statics/moment
    derivation). Returns (pivot_world, rotation_z, contact_point_world,
    theta_snap). wheel_center is a plain (x, y, z) tuple."""
    if tip_engagement_depth_mm >= tooth_depth_mm:
        raise ValueError(
            "tip_engagement_depth_mm (%.3f) must be less than the wheel's tooth_depth_mm "
            "(%.3f) -- the tip can't project deeper than the tooth itself"
            % (tip_engagement_depth_mm, tooth_depth_mm)
        )

    side_angles_deg = {'+X': 0.0, '-X': 180.0, '+Y': 90.0, '-Y': 270.0}
    target_deg = side_angles_deg.get(engagement_side, engagement_angle_deg)  # CUSTOM falls through here
    target_rad = math.radians(target_deg)

    # Snap to the nearest actual tooth drive-face angle so the contact point
    # lands exactly on a real drive face instead of in mid-air between teeth.
    tooth_index = int(round(target_rad / sector_angle)) % tooth_count
    theta_snap = tooth_index * sector_angle

    contact_radius = outer_radius - tip_engagement_depth_mm
    contact_local = (contact_radius * math.cos(theta_snap),
                      contact_radius * math.sin(theta_snap), 0.0)
    contact_world = _vec_add(wheel_center, contact_local)

    # Lock-direction surface velocity at the contact point, per the wheel
    # winding convention documented in ratchet_pawl.py: +Z rotation (CCW)
    # is LOCK, so the tangent direction below points the way the wheel's
    # surface is moving during a lock-direction push.
    tangent_lock = (-math.sin(theta_snap), math.cos(theta_snap), 0.0)
    radial_dir = (math.cos(theta_snap), math.sin(theta_snap), 0.0)

    bias = math.radians(PIVOT_BIAS_ANGLE_DEG)
    pivot_dir = _vec_add(_vec_scale(tangent_lock, -math.cos(bias)), _vec_scale(radial_dir, math.sin(bias)))
    pivot_dir = _vec_normalize(pivot_dir)

    pivot_world = _vec_add(contact_world, _vec_scale(pivot_dir, pawl_arm_length_mm))

    if _vec_length(_vec_sub(pivot_world, wheel_center)) <= outer_radius:
        raise ValueError(
            "pawl_arm_length_mm (%.3f) is too short to reach a pivot position outside "
            "the wheel's outer radius (%.3f) at the chosen engagement point -- increase "
            "pawl_arm_length_mm or pick a different engagement_side/engagement_angle_deg."
            % (pawl_arm_length_mm, outer_radius)
        )

    arm_dir = _vec_sub(contact_world, pivot_world)
    rotation_z = math.atan2(arm_dir[1], arm_dir[0])

    return pivot_world, rotation_z, contact_world, theta_snap


# ── Derived values + top-level build: wheel ─────────────────────────────

def derive_ratchet_wheel(sizing_mode, tooth_count, module, outer_diameter_mm,
                          tooth_depth_mm, tooth_depth_auto, axle_hole_diameter_mm):
    """Pure derivation, matching MESH_OT_add_ratchet_wheel's own draw()
    logic. Never raises - callers check validate_wheel_params separately
    (or catch build_ratchet_wheel_mesh's own ValueError) for the
    reachable error states."""
    root_radius, outer_radius, resolved_tooth_depth_mm = solve_wheel_radii(
        sizing_mode, tooth_count, module, outer_diameter_mm, tooth_depth_mm, tooth_depth_auto)
    sector_angle = 2.0 * math.pi / tooth_count if tooth_count > 0 else 0.0
    back_face_angle_deg = compute_back_face_angle_deg(root_radius, outer_radius, sector_angle) \
        if tooth_count > 0 else 0.0
    return dict(
        root_radius=root_radius, outer_radius=outer_radius, tooth_depth_mm=resolved_tooth_depth_mm,
        sector_angle=sector_angle, back_face_angle_deg=back_face_angle_deg,
    )


def build_ratchet_wheel_mesh(tooth_count=12, sizing_mode='MODULE', module=3.0, outer_diameter_mm=40.0,
                              tooth_depth_mm=1.8, tooth_depth_auto=True, width_mm=6.0,
                              axle_hole_diameter_mm=5.0, axle_hole_compensation_mm=0.0):
    """Returns (mesh, geometry_dict) - a closed, single-solid ratchet wheel
    (local frame, centered at origin). Raises ValueError for invalid
    geometry (see validate_wheel_params)."""
    root_radius, outer_radius, resolved_tooth_depth_mm = solve_wheel_radii(
        sizing_mode, tooth_count, module, outer_diameter_mm, tooth_depth_mm, tooth_depth_auto)
    validate_wheel_params(tooth_count, root_radius, outer_radius, resolved_tooth_depth_mm, axle_hole_diameter_mm)

    points, sector_angle, resolved_tooth_depth_mm = build_wheel_profile_points(
        tooth_count, root_radius, outer_radius)
    back_face_angle_deg = compute_back_face_angle_deg(root_radius, outer_radius, sector_angle)

    hole_radius = (axle_hole_diameter_mm + axle_hole_compensation_mm) / 2.0
    outer_points, inner_points = _hole_outline(points, hole_radius)

    mesh = build_profile_with_hole_mesh(outer_points, width_mm, inner_points)

    geometry = dict(
        root_radius=root_radius, outer_radius=outer_radius, tooth_depth_mm=resolved_tooth_depth_mm,
        sector_angle=sector_angle, back_face_angle_deg=back_face_angle_deg,
    )
    return mesh, geometry


# ── Derived values + top-level build: pawl ──────────────────────────────

def build_ratchet_pawl_mesh(pawl_arm_length_mm=25.0, pawl_arm_width_mm=6.0, pawl_tip_width_mm=3.0,
                             pivot_hole_diameter_mm=5.0, pivot_hole_compensation_mm=0.0, width_mm=6.0):
    """Returns mesh - a closed, single-solid pawl arm (local frame, pivot
    at origin, arm extends along +X). Raises ValueError for invalid
    geometry (arm too short for the pivot hole, or the hole too large for
    the arm width)."""
    hole_dia = pivot_hole_diameter_mm + pivot_hole_compensation_mm
    if pawl_arm_length_mm <= hole_dia:
        raise ValueError("Arm length must exceed pivot hole diameter")
    if hole_dia >= pawl_arm_width_mm:
        raise ValueError("Pivot hole too large for arm width")

    hole_radius = hole_dia / 2.0
    points = build_pawl_profile_points(pawl_arm_length_mm, pawl_arm_width_mm, pawl_tip_width_mm, hole_radius)
    outer_points, inner_points = _hole_outline(points, hole_radius)
    return build_profile_with_hole_mesh(outer_points, width_mm, inner_points)


# ── Derived values + top-level build: combined mechanism ────────────────

def derive_ratchet_mechanism(sizing_mode, tooth_count, module, outer_diameter_mm, tooth_depth_mm,
                              tooth_depth_auto, axle_hole_diameter_mm, pawl_arm_length_mm,
                              pawl_arm_width_mm, pawl_tip_width_mm, pivot_hole_diameter_mm,
                              pivot_hole_compensation_mm):
    """Pure derivation, matching OBJECT_OT_add_ratchet_mechanism's own
    draw() cross-checks (arm-vs-pivot-hole, pivot-hole-vs-arm-width,
    pawl-tip-vs-valley-width). Never raises."""
    wheel_d = derive_ratchet_wheel(sizing_mode, tooth_count, module, outer_diameter_mm,
                                    tooth_depth_mm, tooth_depth_auto, axle_hole_diameter_mm)
    valley_width = estimate_valley_width_mm(wheel_d["root_radius"], wheel_d["outer_radius"], wheel_d["sector_angle"])
    pivot_hole_dia = pivot_hole_diameter_mm + pivot_hole_compensation_mm

    wheel_d.update(
        valley_width_mm=valley_width,
        arm_too_short=(pawl_arm_length_mm <= pivot_hole_dia),
        pivot_hole_too_large=(pivot_hole_dia >= pawl_arm_width_mm),
        tip_too_wide=(pawl_tip_width_mm >= valley_width),
    )
    return wheel_d


def build_ratchet_mechanism(sizing_mode='MODULE', tooth_count=12, module=3.0, outer_diameter_mm=40.0,
                             tooth_depth_mm=1.8, tooth_depth_auto=True, width_mm=6.0,
                             axle_hole_diameter_mm=5.0, axle_hole_compensation_mm=0.0,
                             pawl_arm_length_mm=25.0, pawl_arm_width_mm=6.0, pawl_tip_width_mm=3.0,
                             pivot_hole_diameter_mm=5.0, pivot_hole_compensation_mm=0.0,
                             tip_engagement_depth_mm=1.0, engagement_side='+X', engagement_angle_deg=0.0):
    """Returns (parts, geometry_dict). `parts` is
    {"wheel": {mesh, location, rotation_z}, "pawl": {mesh, location, rotation_z}}
    - the wheel sits at the origin, unrotated; the pawl is placed by
    solve_pawl_pivot's own auto-positioning solve (AUTO mode only - this
    web/pure-Python entry point doesn't expose MANUAL pivot placement,
    since that mode exists in the Blender operator purely for interactive
    override). Raises ValueError for any of: invalid wheel geometry, pawl
    structural invalidity (arm/pivot-hole), tip too wide for the tooth
    valley, or the auto pivot solve itself failing (arm too short)."""
    root_radius, outer_radius, resolved_tooth_depth_mm = solve_wheel_radii(
        sizing_mode, tooth_count, module, outer_diameter_mm, tooth_depth_mm, tooth_depth_auto)
    validate_wheel_params(tooth_count, root_radius, outer_radius, resolved_tooth_depth_mm, axle_hole_diameter_mm)
    sector_angle = 2.0 * math.pi / tooth_count
    back_face_angle_deg = compute_back_face_angle_deg(root_radius, outer_radius, sector_angle)

    pivot_hole_dia = pivot_hole_diameter_mm + pivot_hole_compensation_mm
    if pawl_arm_length_mm <= pivot_hole_dia:
        raise ValueError("Arm length must exceed pivot hole diameter")
    if pivot_hole_dia >= pawl_arm_width_mm:
        raise ValueError("Pivot hole too large for arm width")

    valley_width = estimate_valley_width_mm(root_radius, outer_radius, sector_angle)
    if pawl_tip_width_mm >= valley_width:
        raise ValueError("Tip too wide for tooth valley (%.2f mm)" % valley_width)

    wheel_center = (0.0, 0.0, 0.0)
    pivot_world, rotation_z, contact_world, theta_snap = solve_pawl_pivot(
        wheel_center, root_radius, outer_radius, sector_angle, tooth_count,
        engagement_side, engagement_angle_deg, tip_engagement_depth_mm,
        pawl_arm_length_mm, resolved_tooth_depth_mm)

    wheel_points, _, _ = build_wheel_profile_points(tooth_count, root_radius, outer_radius)
    wheel_hole_radius = (axle_hole_diameter_mm + axle_hole_compensation_mm) / 2.0
    wheel_outer, wheel_inner = _hole_outline(wheel_points, wheel_hole_radius)
    wheel_mesh = build_profile_with_hole_mesh(wheel_outer, width_mm, wheel_inner)

    pawl_hole_radius = pivot_hole_dia / 2.0
    pawl_points = build_pawl_profile_points(pawl_arm_length_mm, pawl_arm_width_mm, pawl_tip_width_mm, pawl_hole_radius)
    pawl_outer, pawl_inner = _hole_outline(pawl_points, pawl_hole_radius)
    pawl_mesh = build_profile_with_hole_mesh(pawl_outer, width_mm, pawl_inner)

    parts = dict(
        wheel=dict(mesh=wheel_mesh, location=wheel_center, rotation_z=0.0),
        pawl=dict(mesh=pawl_mesh, location=pivot_world, rotation_z=rotation_z),
    )
    geometry = dict(
        root_radius=root_radius, outer_radius=outer_radius, tooth_depth_mm=resolved_tooth_depth_mm,
        sector_angle=sector_angle, back_face_angle_deg=back_face_angle_deg,
        valley_width_mm=valley_width, contact_world=contact_world, theta_snap=theta_snap,
    )
    return parts, geometry
