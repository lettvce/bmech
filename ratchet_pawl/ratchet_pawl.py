bl_info = {
    "name": "Ratchet & Pawl Generator (Rigid, No Spring)",
    "author": "Claude (for Blake)",
    "version": (1, 0, 0),
    "blender": (5, 1, 0),
    "location": "View3D > Add > Mesh",
    "description": "Parametric ratchet wheel + pawl pair for FDM printing. v1: rigid geometry only, no retention spring.",
    "category": "Add Mesh",
}

"""
RATCHET & PAWL GENERATOR -- v1.0 implementation notes
======================================================
A few places in the spec are internally ambiguous or mildly self-contradictory.
Per instructions, I didn't stop to ask -- I picked the most physically sane
reading, implemented it, and I'm flagging the decisions right here so they're
easy to find and argue with.

1) [SUPERSEDED by v1.1 addendum, kept for history] DRIVE FACE RADIAL +
   BACK_FACE_ANGLE_DEG AS A FREE PARAMETER:
   The original spec wanted the drive face exactly radial AND wanted
   back_face_angle_deg as an independently tunable input AND wanted uniform
   tooth_count spacing -- over-constrained, can't have all three. v1.0
   resolved it with a two-segment back face (tip -> bend point -> next root)
   so back_face_angle_deg got exact local control right at the tip.
   v1.1 ADDENDUM (current behavior): back_face_angle_deg is removed as an
   input entirely. Single-segment back face, 2 vertices per tooth (root,
   tip), and the ramp angle is now a *derived/reported* value computed from
   tooth_count + tooth_depth_mm + radii via compute_back_face_angle_deg().
   Standard convention for real ratchets -- ramp angle is a consequence of
   tooth proportions, not an independent dial. This also deletes the
   "ramp would overshoot" edge case entirely, since there's no bend point
   left to overshoot with.

2) "Four boundary vertices per tooth" (spec 3.1, bullet list) only actually
   lists two vertex bullets (root-start, tip) plus two face-description
   bullets. Combined with "no fillets, root-start and back-face-end are
   coincident," that means each tooth really contributes 2 *unique* vertices
   in a strict reading -- which is exactly what v1.1 now builds: root and
   tip only, single-segment back face straight to the next tooth's root.

3) Pawl arm silhouette: straight-sided bar, tapering to a wedge tip that
   comes to a point (matches "wedge tip / rounded to a point" from the open
   assumptions list). Pivot end is padded out behind the true pivot axis so
   the pivot hole has wall material around it on all sides -- if I'd put the
   hole right at the flat end of the bar, half of it would hang off into
   thin air.

4) Auto-positioning solve (spec 3.3.2, flagged by the spec itself as the
   riskiest part): implemented as a tangent-biased heuristic, not a rigorous
   statics/moment derivation. The pivot is offset from the contact point
   mostly *against* the lock-direction surface velocity (so the wheel's push
   tends to rotate the arm further into engagement, not out of it), with a
   constant outward radial lean so the pivot reliably clears the wheel body.
   PIVOT_BIAS_ANGLE_DEG below is the tuning knob if the default geometry
   doesn't lock the way you want when you actually swing it by hand in the
   viewport. Tooth winding convention used throughout: wheel rotation in
   +Z (CCW from above) is LOCK, -Z (CW) is FREE. engagement_side only picks
   *where* around the wheel the pawl sits -- it does not flip CW/FREE vs
   CCW/LOCK, since tooth chirality is the same all the way around the wheel.

5) Parenting under an Empty (open assumption #4): implemented as a toggle
   (parent_under_empty, default True) instead of guessing -- you get to
   flip it live in the redo panel rather than me picking for you.

6) pawl_tip_width_mm vs. valley-width validation (spec 3.2 Validation) needs
   wheel geometry, which the standalone mesh.add_ratchet_pawl operator
   doesn't have (its own parameter table never lists wheel inputs). That
   check is only meaningful -- and only performed -- inside the combined
   object.add_ratchet_mechanism operator, where both halves' parameters
   coexist. Running the pawl operator solo skips it; that's documented in
   the operator's docstring, not silently dropped.

7) "width_mm mismatch between wheel and pawl" validation is moot by
   construction in v1: the combined operator exposes exactly one width_mm
   field shared by both parts, so there's nothing that *can* diverge.

8) v1.1: back_face_angle_deg's reported value uses layout.label() in the
   redo panel rather than a disabled/greyed-out property field, even though
   the addendum text offered "disabled in the redo panel" as an option.
   Matches the established suite convention (computed/derived display values
   use row.label(), never row.enabled = False on a stored property) --
   applying it here too rather than introducing a one-off exception. Also
   reported via the operator's info-bar message and stashed as a read-only
   ID property on the generated wheel object, covering the other option the
   addendum mentioned.
"""

import bpy
import bmesh
import math
from mathutils import Vector
from bpy.props import (
    FloatProperty,
    IntProperty,
    EnumProperty,
    BoolProperty,
    FloatVectorProperty,
)

# ----------------------------------------------------------------------------
# Tunables that aren't exposed as operator properties because the spec didn't
# ask for them, but the geometry needs *something* here.
# ----------------------------------------------------------------------------
HOLE_SEGMENTS = 32          # circle resolution for axle / pivot holes
PIVOT_BIAS_ANGLE_DEG = 35.0  # tangent-vs-radial blend for the auto pivot solve
PIVOT_PAD_MM = 2.0           # minimum wall thickness around a pivot/axle hole


# ==============================================================================
# Pure-math helpers (no bpy calls) -- shared by all three operators so the
# wheel geometry is computed exactly once and can't drift between the
# standalone wheel operator and the combined mechanism operator.
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
    tooth_depth_mm, and the radii. See compute_back_face_angle_deg() to find
    out what you got, and decision #1 in the module docstring for why.
    """
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
    the user. Defined as the angle between the back face (tip_i -> root_{i+1})
    and the inward radial direction at the tip, matching the original
    semantics (small angle = steep ramp = more free-spin resistance, large
    angle = shallow ramp = easier free spin). Computed at theta_i = 0
    without loss of generality -- every tooth is identical by rotational
    symmetry, so there's only ever one answer for the whole wheel."""
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
    """Approximate linear width of a tooth valley, measured at tooth mid-depth
    (valleys are sharp V-notches at the root itself -- zero width there by
    definition -- so mid-depth is the practical reference radius for "will a
    pawl tip of width X actually seat without binding"). v1.1: the back face
    is now a single straight chord that runs from full sector width at the
    tip down to zero at the sharp root point; at the radius midpoint this is
    approximated as half the angular pitch evaluated at the mean radius."""
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
        # Defensive fallback only -- real validation of arm_length happens in execute().
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
# bmesh / bpy construction helpers
# ==============================================================================

def create_filled_profile_with_hole(bm, outer_points, hole_radius, hole_segments=HOLE_SEGMENTS):
    """Builds an outer boundary loop and an inner hole loop as two separate,
    explicit edge loops (no fan-from-center-vertex anywhere), then fills the
    region between them. This is the bmesh equivalent of selecting two
    coplanar edge loops in edit mode and hitting F -- it correctly respects
    the hole rather than triangulating across it."""
    outer_verts = [bm.verts.new((x, y, 0.0)) for x, y in outer_points]
    n = len(outer_verts)
    outer_edges = [bm.edges.new((outer_verts[i], outer_verts[(i + 1) % n])) for i in range(n)]

    inner_verts = []
    for i in range(hole_segments):
        ang = 2.0 * math.pi * i / hole_segments
        inner_verts.append(bm.verts.new((hole_radius * math.cos(ang), hole_radius * math.sin(ang), 0.0)))
    m = len(inner_verts)
    inner_edges = [bm.edges.new((inner_verts[i], inner_verts[(i + 1) % m])) for i in range(m)]

    bm.verts.index_update()
    bm.edges.index_update()

    bmesh.ops.triangle_fill(
        bm, use_beauty=True, use_dissolve=True,
        edges=outer_edges + inner_edges, normal=(0.0, 0.0, 1.0),
    )
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return outer_verts, inner_verts


def finalize_mesh_object(context, bm, name, width_mm, location, rotation_z=0.0):
    """Bakes the bmesh into a mesh datablock, links a new object, adds a
    (non-applied, redo-panel-friendly) Solidify modifier, and positions it.
    Because the bmesh is always built around local (0,0,0), the object
    origin lands exactly on that point for free -- no origin_set juggling."""
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(name, me)
    context.collection.objects.link(obj)
    obj.location = location
    obj.rotation_euler = (0.0, 0.0, rotation_z)

    mod = obj.modifiers.new(name="Solidify", type='SOLIDIFY')
    mod.thickness = width_mm
    mod.offset = 0.0  # symmetric about the source plane -> centered on Z

    return obj


def build_wheel_object(context, name, tooth_count, sizing_mode, module, outer_diameter_mm,
                        tooth_depth_mm, tooth_depth_auto, width_mm,
                        axle_hole_diameter_mm, axle_hole_compensation_mm, center_location):
    """Full wheel build, used by both mesh.add_ratchet_wheel and
    object.add_ratchet_mechanism. Returns (obj, geometry_dict)."""
    root_radius, outer_radius, tooth_depth_mm = solve_wheel_radii(
        sizing_mode, tooth_count, module, outer_diameter_mm, tooth_depth_mm, tooth_depth_auto)

    validate_wheel_params(tooth_count, root_radius, outer_radius, tooth_depth_mm, axle_hole_diameter_mm)

    points, sector_angle, tooth_depth_mm = build_wheel_profile_points(
        tooth_count, root_radius, outer_radius)

    # v1.1: back_face_angle_deg is derived, not user-supplied -- compute it
    # here so both the standalone wheel op and the combined op get the same
    # number without duplicating the math.
    back_face_angle_deg = compute_back_face_angle_deg(root_radius, outer_radius, sector_angle)

    hole_radius = (axle_hole_diameter_mm + axle_hole_compensation_mm) / 2.0

    bm = bmesh.new()
    create_filled_profile_with_hole(bm, points, hole_radius)
    obj = finalize_mesh_object(context, bm, name, width_mm, center_location)

    # Stash derived geometry as ID properties -- handy for a future "snap
    # pawl onto existing wheel" tool, and good for debugging in the N-panel.
    # back_face_angle_deg here is read-only/reported, never an editable
    # input (see decision #8 in the module docstring).
    obj["root_radius"] = root_radius
    obj["outer_radius"] = outer_radius
    obj["tooth_depth_mm"] = tooth_depth_mm
    obj["sector_angle_deg"] = math.degrees(sector_angle)
    obj["tooth_count"] = tooth_count
    obj["back_face_angle_deg"] = back_face_angle_deg

    geometry = {
        'root_radius': root_radius,
        'outer_radius': outer_radius,
        'tooth_depth_mm': tooth_depth_mm,
        'sector_angle': sector_angle,
        'back_face_angle_deg': back_face_angle_deg,
    }
    return obj, geometry


def build_pawl_object(context, name, pawl_arm_length_mm, pawl_arm_width_mm, pawl_tip_width_mm,
                       pivot_hole_diameter_mm, pivot_hole_compensation_mm, width_mm,
                       pivot_location, rotation_z=0.0):
    """Full pawl build, used by both mesh.add_ratchet_pawl and
    object.add_ratchet_mechanism. Returns the object."""
    hole_radius = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0

    points = build_pawl_profile_points(pawl_arm_length_mm, pawl_arm_width_mm,
                                        pawl_tip_width_mm, hole_radius)

    bm = bmesh.new()
    create_filled_profile_with_hole(bm, points, hole_radius)
    obj = finalize_mesh_object(context, bm, name, width_mm, pivot_location, rotation_z)

    obj["pawl_arm_length_mm"] = pawl_arm_length_mm
    obj["pivot_hole_diameter_mm"] = pivot_hole_diameter_mm
    return obj


def solve_pawl_pivot(wheel_center, root_radius, outer_radius, sector_angle,
                      tooth_count, engagement_side, engagement_angle_deg,
                      tip_engagement_depth_mm, pawl_arm_length_mm, tooth_depth_mm):
    """Auto-positioning solve per spec 3.3.2 (see decision #4 up top for the
    honesty disclaimer). Returns (pivot_world, rotation_z, contact_point_world,
    theta_snap)."""
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
    contact_local = Vector((contact_radius * math.cos(theta_snap),
                             contact_radius * math.sin(theta_snap), 0.0))
    contact_world = wheel_center + contact_local

    # Lock-direction surface velocity at the contact point, per the wheel
    # winding convention documented at the top of this file: +Z rotation
    # (CCW) is LOCK, so the tangent direction below points the way the
    # wheel's surface is moving during a lock-direction push.
    tangent_lock = Vector((-math.sin(theta_snap), math.cos(theta_snap), 0.0))
    radial_dir = Vector((math.cos(theta_snap), math.sin(theta_snap), 0.0))

    bias = math.radians(PIVOT_BIAS_ANGLE_DEG)
    pivot_dir = (-tangent_lock * math.cos(bias)) + (radial_dir * math.sin(bias))
    pivot_dir.normalize()

    pivot_world = contact_world + pawl_arm_length_mm * pivot_dir

    if (pivot_world - wheel_center).length <= outer_radius:
        raise ValueError(
            "pawl_arm_length_mm (%.3f) is too short to reach a pivot position outside "
            "the wheel's outer radius (%.3f) at the chosen engagement point -- increase "
            "pawl_arm_length_mm or pick a different engagement_side/engagement_angle_deg."
            % (pawl_arm_length_mm, outer_radius)
        )

    arm_dir = contact_world - pivot_world
    rotation_z = math.atan2(arm_dir.y, arm_dir.x)

    return pivot_world, rotation_z, contact_world, theta_snap


# ==============================================================================
# Operators
# ==============================================================================

class MESH_OT_add_ratchet_wheel(bpy.types.Operator):
    """Add a ratchet wheel (asymmetric sawtooth, no spring/pawl included)"""
    bl_idname = "mesh.add_ratchet_wheel"
    bl_label = "Add Ratchet Wheel"
    bl_options = {'REGISTER', 'UNDO'}

    sizing_mode: EnumProperty(
        name="Sizing Mode",
        items=[
            ('MODULE', "Module", "Size via module + tooth count (matches involute gear generator convention)"),
            ('OUTER_DIAMETER', "Outer Diameter", "Size via outer diameter + tooth count; module back-solved"),
        ],
        default='MODULE',
    )
    tooth_count: IntProperty(name="Tooth Count", default=12, soft_min=4, soft_max=200)
    module: FloatProperty(name="Module (mm)", default=3.0, soft_min=0.1, soft_max=20.0)
    outer_diameter_mm: FloatProperty(name="Outer Diameter (mm)", default=40.0, soft_min=2.0, soft_max=500.0)
    tooth_depth_auto: BoolProperty(
        name="Auto Tooth Depth",
        description="Derive tooth_depth_mm as 0.6 * module (or 0.6 * back-solved module in Outer Diameter mode)",
        default=True,
    )
    tooth_depth_mm: FloatProperty(
        name="Tooth Depth (mm)", default=1.8, soft_min=0.1, soft_max=50.0,
        description="Radial depth of each tooth, tip to root. Only used when Auto Tooth Depth is off.",
    )
    width_mm: FloatProperty(name="Width (mm)", default=6.0, soft_min=0.5, soft_max=200.0)
    axle_hole_diameter_mm: FloatProperty(name="Axle Hole Diameter (mm)", default=5.0, soft_min=0.0, soft_max=100.0)
    axle_hole_compensation_mm: FloatProperty(
        name="Axle Hole Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0,
        description="Additive FDM hole-diameter compensation, same convention as the press-fit cutter spec.",
    )
    center_location: FloatVectorProperty(name="Center Location", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION')

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "sizing_mode")
        if self.sizing_mode == 'MODULE':
            layout.prop(self, "module")
        else:
            layout.prop(self, "outer_diameter_mm")
        layout.prop(self, "tooth_count")
        layout.prop(self, "tooth_depth_auto")
        sub = layout.row()
        sub.enabled = not self.tooth_depth_auto
        sub.prop(self, "tooth_depth_mm")
        # back_face_angle_deg is derived, not editable (v1.1) -- shown as a
        # plain label per the suite's "no row.enabled=False on computed
        # values" convention, not as a greyed-out property field.
        layout.label(text="Back Face Angle (derived): %.2f deg" % getattr(self, "computed_back_face_angle_deg", 0.0))
        layout.prop(self, "width_mm")
        layout.prop(self, "axle_hole_diameter_mm")
        layout.prop(self, "axle_hole_compensation_mm")
        layout.prop(self, "center_location")

    def execute(self, context):
        try:
            obj, geom = build_wheel_object(
                context, "RatchetWheel", self.tooth_count, self.sizing_mode, self.module,
                self.outer_diameter_mm, self.tooth_depth_mm, self.tooth_depth_auto,
                self.width_mm, self.axle_hole_diameter_mm,
                self.axle_hole_compensation_mm, self.center_location,
            )
        except ValueError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        except RuntimeError as e:
            # bmesh ops occasionally throw RuntimeError on truly degenerate
            # input (e.g. self-intersecting profile from extreme params).
            self.report({'ERROR'}, "Mesh construction failed: %s" % e)
            return {'CANCELLED'}

        # Plain instance attribute (not a registered bpy property) so it
        # shows up in draw() for the redo panel without being user-editable.
        self.computed_back_face_angle_deg = geom['back_face_angle_deg']

        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj
        self.report(
            {'INFO'},
            "Ratchet wheel created: %d teeth, back face angle %.2f deg (derived)"
            % (self.tooth_count, geom['back_face_angle_deg'])
        )
        return {'FINISHED'}


class MESH_OT_add_ratchet_pawl(bpy.types.Operator):
    """Add a ratchet pawl arm (rigid, no spring -- you swing/hold it into engagement yourself)"""
    bl_idname = "mesh.add_ratchet_pawl"
    bl_label = "Add Ratchet Pawl"
    bl_options = {'REGISTER', 'UNDO'}

    pawl_arm_length_mm: FloatProperty(name="Arm Length (mm)", default=25.0, soft_min=2.0, soft_max=300.0)
    pawl_arm_width_mm: FloatProperty(name="Arm Width (mm)", default=6.0, soft_min=1.0, soft_max=50.0)
    pawl_tip_width_mm: FloatProperty(name="Tip Width (mm)", default=3.0, soft_min=0.1, soft_max=50.0)
    pivot_hole_diameter_mm: FloatProperty(name="Pivot Hole Diameter (mm)", default=5.0, soft_min=0.0, soft_max=50.0)
    pivot_hole_compensation_mm: FloatProperty(
        name="Pivot Hole Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0,
    )
    width_mm: FloatProperty(name="Width (mm)", default=6.0, soft_min=0.5, soft_max=200.0)
    tip_engagement_depth_mm: FloatProperty(
        name="Tip Engagement Depth (mm)", default=1.0, soft_min=0.0, soft_max=20.0,
        description="How far the tip projects into the tooth valley at full engagement. "
                    "Informational here; only consumed by the auto-positioning solve in the combined operator.",
    )
    pivot_location: FloatVectorProperty(
        name="Pivot Location", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION',
        description="World-space pivot placement. Not in the original spec table -- added so this "
                    "operator is independently usable; the combined operator overrides this via its own solve.",
    )
    pivot_rotation_deg: FloatProperty(name="Pivot Rotation (deg)", default=0.0, soft_min=-360.0, soft_max=360.0)

    def execute(self, context):
        # NOTE: pawl_tip_width_mm vs. tooth-valley-width validation is
        # intentionally NOT performed here -- this operator has no wheel
        # geometry to check against (see decision #6 in the module
        # docstring). Use object.add_ratchet_mechanism if you want that check.
        if self.pawl_arm_length_mm <= self.pivot_hole_diameter_mm + self.pivot_hole_compensation_mm:
            self.report({'ERROR'}, "pawl_arm_length_mm must be greater than the pivot hole diameter (degenerate arm)")
            return {'CANCELLED'}
        hole_dia = self.pivot_hole_diameter_mm + self.pivot_hole_compensation_mm
        if hole_dia >= self.pawl_arm_width_mm:
            self.report({'ERROR'}, "Pivot hole diameter is too large for the arm width at the pivot end")
            return {'CANCELLED'}

        try:
            obj = build_pawl_object(
                context, "Pawl", self.pawl_arm_length_mm, self.pawl_arm_width_mm,
                self.pawl_tip_width_mm, self.pivot_hole_diameter_mm, self.pivot_hole_compensation_mm,
                self.width_mm, self.pivot_location, math.radians(self.pivot_rotation_deg),
            )
        except ValueError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        except RuntimeError as e:
            self.report({'ERROR'}, "Mesh construction failed: %s" % e)
            return {'CANCELLED'}

        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj
        return {'FINISHED'}


class OBJECT_OT_add_ratchet_mechanism(bpy.types.Operator):
    """Add a matched ratchet wheel + pawl pair, auto-positioned for engagement (rigid, no spring)"""
    bl_idname = "object.add_ratchet_mechanism"
    bl_label = "Add Ratchet & Pawl"
    bl_options = {'REGISTER', 'UNDO'}

    # --- Wheel params ---
    sizing_mode: EnumProperty(
        name="Sizing Mode",
        items=[
            ('MODULE', "Module", "Size via module + tooth count"),
            ('OUTER_DIAMETER', "Outer Diameter", "Size via outer diameter + tooth count"),
        ],
        default='MODULE',
    )
    tooth_count: IntProperty(name="Tooth Count", default=12, soft_min=4, soft_max=200)
    module: FloatProperty(name="Module (mm)", default=3.0, soft_min=0.1, soft_max=20.0)
    outer_diameter_mm: FloatProperty(name="Outer Diameter (mm)", default=40.0, soft_min=2.0, soft_max=500.0)
    tooth_depth_auto: BoolProperty(name="Auto Tooth Depth", default=True)
    tooth_depth_mm: FloatProperty(name="Tooth Depth (mm)", default=1.8, soft_min=0.1, soft_max=50.0)
    axle_hole_diameter_mm: FloatProperty(name="Axle Hole Diameter (mm)", default=5.0, soft_min=0.0, soft_max=100.0)
    axle_hole_compensation_mm: FloatProperty(name="Axle Hole Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0)
    center_location: FloatVectorProperty(name="Wheel Center", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION')

    # --- Shared ---
    width_mm: FloatProperty(
        name="Width (mm)", default=6.0, soft_min=0.5, soft_max=200.0,
        description="Solidify depth, shared by both wheel and pawl -- one field, can't diverge.",
    )

    # --- Pawl params ---
    pawl_arm_length_mm: FloatProperty(name="Pawl Arm Length (mm)", default=25.0, soft_min=2.0, soft_max=300.0)
    pawl_arm_width_mm: FloatProperty(name="Pawl Arm Width (mm)", default=6.0, soft_min=1.0, soft_max=50.0)
    pawl_tip_width_mm: FloatProperty(name="Pawl Tip Width (mm)", default=3.0, soft_min=0.1, soft_max=50.0)
    pivot_hole_diameter_mm: FloatProperty(name="Pivot Hole Diameter (mm)", default=5.0, soft_min=0.0, soft_max=50.0)
    pivot_hole_compensation_mm: FloatProperty(name="Pivot Hole Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0)
    tip_engagement_depth_mm: FloatProperty(name="Tip Engagement Depth (mm)", default=1.0, soft_min=0.01, soft_max=20.0)

    # --- Auto-positioning ---
    center_distance_solve_mode: EnumProperty(
        name="Pivot Placement",
        items=[
            ('AUTO', "Auto", "Solve pivot position from pawl_arm_length_mm and engagement_side"),
            ('MANUAL', "Manual", "Use pawl_pivot_location directly"),
        ],
        default='AUTO',
    )
    pawl_pivot_location: FloatVectorProperty(
        name="Pawl Pivot Location", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION',
        description="Only used when Pivot Placement is Manual.",
    )
    engagement_side: EnumProperty(
        name="Engagement Side",
        items=[
            ('+X', "+X", "Pawl sits on the +X side of the wheel"),
            ('-X', "-X", "Pawl sits on the -X side of the wheel"),
            ('+Y', "+Y", "Pawl sits on the +Y side of the wheel"),
            ('-Y', "-Y", "Pawl sits on the -Y side of the wheel"),
            ('CUSTOM', "Custom", "Use engagement_angle_deg"),
        ],
        default='+X',
    )
    engagement_angle_deg: FloatProperty(name="Engagement Angle (deg)", default=0.0, soft_min=-360.0, soft_max=360.0)

    parent_under_empty: BoolProperty(
        name="Parent Under Empty",
        description="Parent both parts under a new 'RatchetMechanism' Empty for convenient manipulation "
                    "(open assumption from the spec -- exposed as a toggle instead of a guess).",
        default=True,
    )

    def draw(self, context):
        layout = self.layout

        box = layout.box()
        box.label(text="Wheel")
        box.prop(self, "sizing_mode")
        if self.sizing_mode == 'MODULE':
            box.prop(self, "module")
        else:
            box.prop(self, "outer_diameter_mm")
        box.prop(self, "tooth_count")
        box.prop(self, "tooth_depth_auto")
        sub = box.row()
        sub.enabled = not self.tooth_depth_auto
        sub.prop(self, "tooth_depth_mm")
        box.label(text="Back Face Angle (derived): %.2f deg" % getattr(self, "computed_back_face_angle_deg", 0.0))
        box.prop(self, "axle_hole_diameter_mm")
        box.prop(self, "axle_hole_compensation_mm")
        box.prop(self, "center_location")

        box = layout.box()
        box.label(text="Pawl")
        box.prop(self, "pawl_arm_length_mm")
        box.prop(self, "pawl_arm_width_mm")
        box.prop(self, "pawl_tip_width_mm")
        box.prop(self, "pivot_hole_diameter_mm")
        box.prop(self, "pivot_hole_compensation_mm")
        box.prop(self, "tip_engagement_depth_mm")

        box = layout.box()
        box.label(text="Shared / Positioning")
        box.prop(self, "width_mm")
        box.prop(self, "center_distance_solve_mode")
        if self.center_distance_solve_mode == 'MANUAL':
            box.prop(self, "pawl_pivot_location")
        else:
            box.prop(self, "engagement_side")
            if self.engagement_side == 'CUSTOM':
                box.prop(self, "engagement_angle_deg")
        box.prop(self, "parent_under_empty")

    def execute(self, context):
        # --- Wheel geometry + validation first; everything downstream needs it. ---
        try:
            root_radius, outer_radius, tooth_depth_mm = solve_wheel_radii(
                self.sizing_mode, self.tooth_count, self.module, self.outer_diameter_mm,
                self.tooth_depth_mm, self.tooth_depth_auto)
            validate_wheel_params(self.tooth_count, root_radius,
                                   outer_radius, tooth_depth_mm, self.axle_hole_diameter_mm)
            sector_angle = 2.0 * math.pi / self.tooth_count
        except ValueError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}

        # Derived/reported back face angle (v1.1) -- computed early so it's
        # available for the redo panel label even if a later validation
        # step below cancels the operator.
        self.computed_back_face_angle_deg = compute_back_face_angle_deg(root_radius, outer_radius, sector_angle)

        # --- Pawl structural validation ---
        pivot_hole_dia = self.pivot_hole_diameter_mm + self.pivot_hole_compensation_mm
        if self.pawl_arm_length_mm <= pivot_hole_dia:
            self.report({'ERROR'}, "pawl_arm_length_mm must be greater than the pivot hole diameter (degenerate arm)")
            return {'CANCELLED'}
        if pivot_hole_dia >= self.pawl_arm_width_mm:
            self.report({'ERROR'}, "Pivot hole diameter is too large for the arm width at the pivot end")
            return {'CANCELLED'}

        # --- Cross-part validation: tip width vs. tooth valley (only possible here) ---
        valley_width = estimate_valley_width_mm(root_radius, outer_radius, sector_angle)
        if self.pawl_tip_width_mm >= valley_width:
            self.report(
                {'ERROR'},
                "pawl_tip_width_mm (%.3f) is too wide to seat in a tooth valley "
                "(valley width approx %.3f mm at mid-depth)" % (self.pawl_tip_width_mm, valley_width)
            )
            return {'CANCELLED'}

        wheel_center = Vector(self.center_location)

        # --- Pivot placement ---
        if self.center_distance_solve_mode == 'AUTO':
            try:
                pivot_world, rotation_z, contact_world, theta_snap = solve_pawl_pivot(
                    wheel_center, root_radius, outer_radius, sector_angle, self.tooth_count,
                    self.engagement_side, self.engagement_angle_deg, self.tip_engagement_depth_mm,
                    self.pawl_arm_length_mm, tooth_depth_mm,
                )
            except ValueError as e:
                self.report({'ERROR'}, str(e))
                return {'CANCELLED'}
        else:  # MANUAL
            pivot_world = Vector(self.pawl_pivot_location)
            if (pivot_world - wheel_center).length <= outer_radius:
                self.report({'ERROR'}, "pawl_pivot_location is inside the wheel's outer radius -- the parts would intersect")
                return {'CANCELLED'}
            # Aim the arm at a nominal contact point so it at least visually
            # engages; if pawl_arm_length_mm doesn't match the actual pivot-to-
            # contact distance, the tip simply won't land exactly on it -- your
            # call, since you're overriding the auto solve.
            target_deg = {'+X': 0.0, '-X': 180.0, '+Y': 90.0, '-Y': 270.0}.get(self.engagement_side, self.engagement_angle_deg)
            theta_snap = (round(math.radians(target_deg) / sector_angle) * sector_angle) % (2.0 * math.pi)
            contact_radius = outer_radius - self.tip_engagement_depth_mm
            contact_world = wheel_center + Vector((contact_radius * math.cos(theta_snap),
                                                     contact_radius * math.sin(theta_snap), 0.0))
            arm_dir = contact_world - pivot_world
            rotation_z = math.atan2(arm_dir.y, arm_dir.x)
            actual_dist = arm_dir.length
            if abs(actual_dist - self.pawl_arm_length_mm) > 0.5:
                self.report(
                    {'WARNING'},
                    "Manual pivot is %.2f mm from the contact point but pawl_arm_length_mm is %.2f mm "
                    "-- the tip won't land exactly on the drive face" % (actual_dist, self.pawl_arm_length_mm)
                )

        # --- Build both objects ---
        try:
            wheel_obj, _geom = build_wheel_object(
                context, "RatchetWheel", self.tooth_count, self.sizing_mode, self.module,
                self.outer_diameter_mm, self.tooth_depth_mm, self.tooth_depth_auto,
                self.width_mm, self.axle_hole_diameter_mm,
                self.axle_hole_compensation_mm, self.center_location,
            )
            pawl_obj = build_pawl_object(
                context, "Pawl", self.pawl_arm_length_mm, self.pawl_arm_width_mm,
                self.pawl_tip_width_mm, self.pivot_hole_diameter_mm, self.pivot_hole_compensation_mm,
                self.width_mm, pivot_world, rotation_z,
            )
        except ValueError as e:
            self.report({'ERROR'}, str(e))
            return {'CANCELLED'}
        except RuntimeError as e:
            self.report({'ERROR'}, "Mesh construction failed: %s" % e)
            return {'CANCELLED'}

        # --- Optional parenting ---
        if self.parent_under_empty:
            empty = bpy.data.objects.new("RatchetMechanism", None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = max(_geom['outer_radius'] * 0.3, 5.0)
            empty.location = wheel_center
            context.collection.objects.link(empty)
            for child in (wheel_obj, pawl_obj):
                child.parent = empty
                child.matrix_parent_inverse = empty.matrix_world.inverted()

        for o in context.selected_objects:
            o.select_set(False)
        wheel_obj.select_set(True)
        pawl_obj.select_set(True)
        context.view_layer.objects.active = pawl_obj

        self.report(
            {'INFO'},
            "Ratchet mechanism created: %d teeth, back face angle %.2f deg (derived), pawl pivot %s engagement"
            % (self.tooth_count, _geom['back_face_angle_deg'], self.engagement_side)
        )
        return {'FINISHED'}


classes = (
    MESH_OT_add_ratchet_wheel,
    MESH_OT_add_ratchet_pawl,
    OBJECT_OT_add_ratchet_mechanism,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
