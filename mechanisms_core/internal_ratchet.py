"""
Internal Freewheel Ratchet Generator
=====================================
Generates a bicycle-freewheel-style internal ratchet: an outer ring with
inward-pointing sawtooth teeth, an inner hub, and pawl_count pawls mounted
on the hub surface pointing radially outward.

Lock direction convention: CW rotation of the inner hub (viewed from +Z) =
LOCK (forward pedaling). CCW = FREE (freewheeling). This matches the drive-side
view of a standard bicycle rear hub.

Geometry helpers (finalize_mesh_object, build_pawl_profile_points,
create_filled_profile_with_hole) are copied from ratchet_pawl.py so this module
has no inter-module dependency.
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

# ── Constants ──────────────────────────────────────────────────────────────────
HOLE_SEGMENTS     = 32
RING_SEGMENTS_OUT = 64
PIVOT_PAD_MM  = 2.0


# ── Low-level mesh helpers (copied from ratchet_pawl.py) ──────────────────────

def _circle_pts(radius, n=HOLE_SEGMENTS):
    return [
        (radius * math.cos(2 * math.pi * i / n),
         radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def finalize_mesh_object(context, bm, name, width_mm, location, rotation_z=0.0):
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
    mod.offset = 0.0
    return obj


def create_filled_profile_with_hole(bm, outer_points, inner_points):
    """Fill the annular region between two polygon loops with triangle_fill."""
    outer_verts = [bm.verts.new((x, y, 0.0)) for x, y in outer_points]
    n = len(outer_verts)
    outer_edges = [bm.edges.new((outer_verts[i], outer_verts[(i + 1) % n])) for i in range(n)]

    inner_verts = [bm.verts.new((x, y, 0.0)) for x, y in inner_points]
    m = len(inner_verts)
    inner_edges = [bm.edges.new((inner_verts[i], inner_verts[(i + 1) % m])) for i in range(m)]

    bm.verts.index_update()
    bm.edges.index_update()
    bmesh.ops.triangle_fill(
        bm, use_beauty=True, use_dissolve=True,
        edges=outer_edges + inner_edges, normal=(0.0, 0.0, 1.0),
    )
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])


def build_pawl_profile_points(arm_length, arm_width, tip_width, hole_radius):
    """Local-frame pawl boundary: pivot at origin, arm along +X."""
    pivot_pad   = hole_radius + PIVOT_PAD_MM
    taper_start = arm_length * 0.3
    wedge_len   = max(tip_width, 1.0)
    tip_base_x  = arm_length - wedge_len
    if tip_base_x <= taper_start:
        tip_base_x = taper_start + 0.001
    return [
        (-pivot_pad,   arm_width / 2.0),
        (taper_start,  arm_width / 2.0),
        (tip_base_x,   tip_width / 2.0),
        (arm_length,   0.0),
        (tip_base_x,  -tip_width / 2.0),
        (taper_start, -arm_width / 2.0),
        (-pivot_pad,  -arm_width / 2.0),
    ]


# ── Freewheel geometry helpers ─────────────────────────────────────────────────

def solve_ring_radii(sizing_mode, tooth_count, module, outer_diameter_mm,
                     ring_wall_thickness_mm, tooth_depth_mm, tooth_depth_auto):
    """Returns (ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm).

    ring_inner_r = root of inner teeth (where teeth originate on the ring wall).
    tip_r        = innermost point of each tooth, pointing toward center.
    """
    if sizing_mode == 'MODULE':
        # module controls tooth pitch on the inner toothed surface
        ring_inner_r = module * tooth_count / 2.0
        ring_outer_r = ring_inner_r + ring_wall_thickness_mm
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * module
    else:  # OUTER_DIAMETER
        ring_outer_r = outer_diameter_mm / 2.0
        ring_inner_r = ring_outer_r - ring_wall_thickness_mm
        eff_module   = ring_inner_r * 2.0 / max(tooth_count, 1)
        if tooth_depth_auto:
            tooth_depth_mm = 0.6 * eff_module
    tip_r = ring_inner_r - tooth_depth_mm
    return ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm


def build_inner_teeth_points(tooth_count, ring_inner_r, tip_r):
    """
    Closed polygon for the inner toothed surface of the ring, going CCW.
    Each tooth: root at ring_inner_r, tip at tip_r (pointing inward toward center).

    Drive face: root_i → tip_i  (radial, catches the pawl for CW inner-hub rotation)
    Ramp face:  tip_i → root_{i+1}  (angled chord, pawl slides over for CCW = FREE)

    Pattern is identical to build_wheel_profile_points in ratchet_pawl.py, with
    the outer_radius and root_radius roles swapped so teeth point inward.
    """
    sector_angle = 2.0 * math.pi / tooth_count
    points = []
    for i in range(tooth_count):
        theta = i * sector_angle
        points.append((ring_inner_r * math.cos(theta), ring_inner_r * math.sin(theta)))
        points.append((tip_r       * math.cos(theta), tip_r        * math.sin(theta)))
    return points


def pawl_pivot_radius(hub_outer_r, hole_r):
    """Radial distance from assembly centre to pawl pivot.
    Pivot sits far enough outside the hub that the hole is fully clear of the hub body."""
    return hub_outer_r + hole_r + PIVOT_PAD_MM


def auto_pawl_arm_length(hub_outer_r, tip_r, tip_engagement_depth_mm, hole_r):
    """Arm length so the tip reaches tip_r + tip_engagement_depth from the pivot."""
    pivot_r   = pawl_pivot_radius(hub_outer_r, hole_r)
    contact_r = tip_r + tip_engagement_depth_mm
    return max(contact_r - pivot_r, 1.0)


def validate_freewheel(ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, tooth_count,
                        hub_outer_r, bore_r, clearance_mm,
                        pawl_count, pawl_arm_length_mm, pawl_arm_width_mm,
                        pivot_hole_diameter_mm, pivot_hole_compensation_mm):
    errs = []
    if ring_inner_r <= 0:
        errs.append("Ring inner radius ≤ 0 — reduce wall thickness or increase size.")
    if tip_r <= 0:
        errs.append("Tip radius ≤ 0 — reduce tooth depth.")
    if tooth_depth_mm >= ring_inner_r:
        errs.append("Tooth depth (%.2f) ≥ ring inner radius (%.2f)." % (tooth_depth_mm, ring_inner_r))
    max_hub_r = tip_r - clearance_mm
    if hub_outer_r >= max_hub_r:
        errs.append(
            "Hub outer radius (%.2f mm) must be < tip radius − clearance (%.2f mm). "
            "Reduce hub diameter or increase ring size." % (hub_outer_r, max_hub_r)
        )
    if bore_r >= hub_outer_r:
        errs.append("Bore radius (%.2f mm) ≥ hub outer radius (%.2f mm)." % (bore_r, hub_outer_r))
    if tooth_count < 4:
        errs.append("Tooth count must be ≥ 4.")
    if pawl_count < 1:
        errs.append("Pawl count must be ≥ 1.")
    pivot_hole_r = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    if pivot_hole_r * 2 >= pawl_arm_width_mm:
        errs.append("Pivot hole diameter too large for pawl arm width.")
    if pawl_arm_length_mm <= pivot_hole_diameter_mm + pivot_hole_compensation_mm:
        errs.append("Pawl arm length must be greater than pivot hole diameter.")
    return errs


# ── Object builders ────────────────────────────────────────────────────────────

def build_ring_object(context, tooth_count, ring_outer_r, ring_inner_r, tip_r,
                       width_mm, location):
    from mathutils.geometry import tessellate_polygon

    outer_pts = _circle_pts(ring_outer_r, RING_SEGMENTS_OUT)
    inner_pts = build_inner_teeth_points(tooth_count, ring_inner_r, tip_r)

    # tessellate_polygon handles multi-contour polygons correctly:
    # first contour = outer boundary (CCW), second = hole (must be CW = reversed).
    # triangle_fill treats each enclosed pocket as a separate region, which breaks
    # non-convex inner boundaries like the toothed surface.
    outer_3d = [(x, y, 0.0) for x, y in outer_pts]
    inner_3d = [(x, y, 0.0) for x, y in reversed(inner_pts)]
    tris     = tessellate_polygon([outer_3d, inner_3d])

    all_pts = outer_3d + inner_3d
    bm      = bmesh.new()
    verts   = [bm.verts.new(p) for p in all_pts]
    bm.verts.index_update()
    for tri in tris:
        bm.faces.new([verts[i] for i in tri])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    return finalize_mesh_object(context, bm, "FreewheelRing", width_mm, location)


def build_hub_object(context, hub_outer_r, bore_r, width_mm, location):
    outer_pts = _circle_pts(hub_outer_r, HOLE_SEGMENTS)
    inner_pts = _circle_pts(bore_r,      HOLE_SEGMENTS)
    bm = bmesh.new()
    create_filled_profile_with_hole(bm, outer_pts, inner_pts)
    return finalize_mesh_object(context, bm, "FreewheelHub", width_mm, location)


def build_pawl_object(context, index, arm_length, arm_width, tip_width,
                       pivot_hole_diameter_mm, pivot_hole_compensation_mm,
                       width_mm, hub_outer_r, angle, base_location):
    hole_r = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    pts    = build_pawl_profile_points(arm_length, arm_width, tip_width, hole_r)

    bm         = bmesh.new()
    outer_verts = [bm.verts.new((x, y, 0.0)) for x, y in pts]
    n           = len(outer_verts)
    outer_edges = [bm.edges.new((outer_verts[i], outer_verts[(i + 1) % n])) for i in range(n)]

    inner_verts = []
    for j in range(HOLE_SEGMENTS):
        a = 2.0 * math.pi * j / HOLE_SEGMENTS
        inner_verts.append(bm.verts.new((hole_r * math.cos(a), hole_r * math.sin(a), 0.0)))
    m           = len(inner_verts)
    inner_edges = [bm.edges.new((inner_verts[j], inner_verts[(j + 1) % m])) for j in range(m)]

    bm.verts.index_update()
    bm.edges.index_update()
    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                            edges=outer_edges + inner_edges, normal=(0.0, 0.0, 1.0))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    hole_r  = (pivot_hole_diameter_mm + pivot_hole_compensation_mm) / 2.0
    pivot_r = pawl_pivot_radius(hub_outer_r, hole_r)
    pivot_x = base_location[0] + pivot_r * math.cos(angle)
    pivot_y = base_location[1] + pivot_r * math.sin(angle)
    pivot_z = base_location[2]

    return finalize_mesh_object(
        context, bm, "FreewheelPawl.%03d" % index,
        width_mm, (pivot_x, pivot_y, pivot_z), rotation_z=angle,
    )


# ── Operator ───────────────────────────────────────────────────────────────────

class OBJECT_OT_add_internal_ratchet(bpy.types.Operator):
    """Add an internal freewheel ratchet (bicycle hub style): outer ring with
    inward teeth, inner hub, and evenly-spaced pawls. CW inner hub = LOCK."""
    bl_idname  = "object.add_internal_ratchet"
    bl_label   = "Add Internal Freewheel Ratchet"
    bl_options = {'REGISTER', 'UNDO'}

    # ── Ring ──────────────────────────────────────────────────────────────────
    sizing_mode: EnumProperty(
        name="Sizing Mode",
        items=[
            ('MODULE',         "Module",         "Size via module + tooth count"),
            ('OUTER_DIAMETER', "Outer Diameter",  "Size via outer ring diameter + tooth count"),
        ],
        default='MODULE',
    )
    tooth_count: IntProperty(
        name="Tooth Count", default=24, soft_min=4, soft_max=200,
    )
    module: FloatProperty(
        name="Module (mm)", default=2.0, soft_min=0.1, soft_max=20.0,
    )
    outer_diameter_mm: FloatProperty(
        name="Outer Diameter (mm)", default=50.0, soft_min=5.0, soft_max=500.0,
    )
    ring_wall_thickness_mm: FloatProperty(
        name="Ring Wall Thickness (mm)", default=4.0, soft_min=0.5, soft_max=50.0,
        description="Radial thickness of the outer ring wall, from outer surface to tooth roots",
    )
    tooth_depth_auto: BoolProperty(
        name="Auto Tooth Depth",
        description="Derive tooth depth as 0.6 × module",
        default=True,
    )
    tooth_depth_mm: FloatProperty(
        name="Tooth Depth (mm)", default=1.2, soft_min=0.1, soft_max=20.0,
        description="Radial depth of each tooth from root inward toward center. Only used when Auto is off.",
    )

    # ── Hub ───────────────────────────────────────────────────────────────────
    hub_outer_diameter_mm: FloatProperty(
        name="Hub Outer Diameter (mm)", default=20.0, soft_min=1.0, soft_max=200.0,
        description="Outer diameter of the inner hub. Must be smaller than the ring tip diameter minus clearance.",
    )
    bore_diameter_mm: FloatProperty(
        name="Bore Diameter (mm)", default=5.0, soft_min=0.0, soft_max=100.0,
        description="Center axle hole through the hub",
    )
    bore_compensation_mm: FloatProperty(
        name="Bore Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0,
        description="FDM hole-size compensation added to bore diameter",
    )
    clearance_mm: FloatProperty(
        name="Running Clearance (mm)", default=0.3, soft_min=0.0, soft_max=5.0,
        description="Radial gap between hub outer surface and ring tooth tips — keeps parts from binding when printed",
    )

    # ── Pawls ─────────────────────────────────────────────────────────────────
    pawl_count: IntProperty(
        name="Pawl Count", default=3, min=1, soft_max=12,
        description="Number of pawls distributed evenly around the hub",
    )
    pawl_arm_length_auto: BoolProperty(
        name="Auto Arm Length",
        description="Compute arm length from ring/hub geometry so the tip just reaches the tooth valley",
        default=True,
    )
    pawl_arm_length_mm: FloatProperty(
        name="Arm Length (mm)", default=8.0, soft_min=1.0, soft_max=100.0,
        description="Distance from pivot to tip. Only used when Auto Arm Length is off.",
    )
    pawl_arm_width_mm: FloatProperty(
        name="Arm Width (mm)", default=4.0, soft_min=1.0, soft_max=30.0,
    )
    pawl_tip_width_mm: FloatProperty(
        name="Tip Width (mm)", default=1.5, soft_min=0.1, soft_max=20.0,
    )
    tip_engagement_depth_mm: FloatProperty(
        name="Tip Engagement Depth (mm)", default=0.6, soft_min=0.0, soft_max=10.0,
        description="How far the tip projects into the tooth valley past the tooth tips",
    )
    pivot_hole_diameter_mm: FloatProperty(
        name="Pivot Hole Diameter (mm)", default=2.0, soft_min=0.0, soft_max=20.0,
    )
    pivot_hole_compensation_mm: FloatProperty(
        name="Pivot Hole Compensation (mm)", default=0.0, soft_min=-2.0, soft_max=2.0,
    )

    # ── Shared ────────────────────────────────────────────────────────────────
    width_mm: FloatProperty(
        name="Width (mm)", default=6.0, soft_min=0.5, soft_max=200.0,
        description="Solidify depth — shared by ring, hub, and all pawls",
    )
    center_location: FloatVectorProperty(
        name="Center", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION',
    )
    parent_under_empty: BoolProperty(
        name="Parent Under Empty", default=True,
        description="Group all parts under a FreewheelRatchet empty for easy manipulation",
    )

    def _compute(self):
        """Return derived geometry values used by both draw() and execute()."""
        ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm = solve_ring_radii(
            self.sizing_mode, self.tooth_count, self.module, self.outer_diameter_mm,
            self.ring_wall_thickness_mm, self.tooth_depth_mm, self.tooth_depth_auto,
        )
        hub_outer_r = self.hub_outer_diameter_mm / 2.0
        bore_r      = (self.bore_diameter_mm + self.bore_compensation_mm) / 2.0
        hole_r      = (self.pivot_hole_diameter_mm + self.pivot_hole_compensation_mm) / 2.0
        arm_length  = (auto_pawl_arm_length(hub_outer_r, tip_r, self.tip_engagement_depth_mm, hole_r)
                       if self.pawl_arm_length_auto else self.pawl_arm_length_mm)
        return ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, hub_outer_r, bore_r, arm_length

    def draw(self, context):
        layout = self.layout
        ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, hub_outer_r, bore_r, arm_length = self._compute()

        box = layout.box()
        box.label(text="Ring")
        box.prop(self, "sizing_mode")
        if self.sizing_mode == 'MODULE':
            box.prop(self, "module")
        else:
            box.prop(self, "outer_diameter_mm")
        box.prop(self, "tooth_count")
        box.prop(self, "ring_wall_thickness_mm")
        box.prop(self, "tooth_depth_auto")
        sub = box.row()
        sub.enabled = not self.tooth_depth_auto
        sub.prop(self, "tooth_depth_mm")
        box.label(text="Ring Ø %.2f mm  ·  Tooth root Ø %.2f mm  ·  Tip Ø %.2f mm"
                  % (ring_outer_r * 2, ring_inner_r * 2, tip_r * 2))

        box = layout.box()
        box.label(text="Hub")
        box.prop(self, "hub_outer_diameter_mm")
        box.prop(self, "bore_diameter_mm")
        box.prop(self, "bore_compensation_mm")
        box.prop(self, "clearance_mm")
        max_hub_d = (tip_r - self.clearance_mm) * 2
        if self.hub_outer_diameter_mm >= max_hub_d:
            box.label(text="Hub too large — max %.2f mm" % max_hub_d, icon='ERROR')

        box = layout.box()
        box.label(text="Pawls")
        box.prop(self, "pawl_count")
        box.prop(self, "tip_engagement_depth_mm")
        box.prop(self, "pawl_arm_length_auto")
        sub = box.row()
        sub.enabled = not self.pawl_arm_length_auto
        sub.prop(self, "pawl_arm_length_mm")
        if self.pawl_arm_length_auto:
            box.label(text="Arm length (auto): %.2f mm" % arm_length)
        box.prop(self, "pawl_arm_width_mm")
        box.prop(self, "pawl_tip_width_mm")
        box.prop(self, "pivot_hole_diameter_mm")
        box.prop(self, "pivot_hole_compensation_mm")

        box = layout.box()
        box.label(text="Shared")
        box.prop(self, "width_mm")
        box.prop(self, "center_location")
        box.prop(self, "parent_under_empty")

    def execute(self, context):
        ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, hub_outer_r, bore_r, arm_length = self._compute()

        errs = validate_freewheel(
            ring_outer_r, ring_inner_r, tip_r, tooth_depth_mm, self.tooth_count,
            hub_outer_r, bore_r, self.clearance_mm,
            self.pawl_count, arm_length, self.pawl_arm_width_mm,
            self.pivot_hole_diameter_mm, self.pivot_hole_compensation_mm,
        )
        if errs:
            self.report({'ERROR'}, errs[0])
            return {'CANCELLED'}

        loc = tuple(self.center_location)
        try:
            ring_obj = build_ring_object(
                context, self.tooth_count, ring_outer_r, ring_inner_r, tip_r,
                self.width_mm, loc,
            )
            hub_obj = build_hub_object(context, hub_outer_r, bore_r, self.width_mm, loc)
            pawl_objs = [
                build_pawl_object(
                    context, i, arm_length, self.pawl_arm_width_mm, self.pawl_tip_width_mm,
                    self.pivot_hole_diameter_mm, self.pivot_hole_compensation_mm,
                    self.width_mm, hub_outer_r,
                    angle=i * 2.0 * math.pi / self.pawl_count + math.pi / self.tooth_count,
                    base_location=loc,
                )
                for i in range(self.pawl_count)
            ]
        except (ValueError, RuntimeError) as e:
            self.report({'ERROR'}, "Mesh construction failed: %s" % e)
            return {'CANCELLED'}

        all_objs = [ring_obj, hub_obj] + pawl_objs

        if self.parent_under_empty:
            empty = bpy.data.objects.new("FreewheelRatchet", None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = ring_outer_r * 0.3
            empty.location           = loc
            context.collection.objects.link(empty)
            for child in all_objs:
                child.parent = empty
                child.matrix_parent_inverse = empty.matrix_world.inverted()

        for o in context.selected_objects:
            o.select_set(False)
        for o in all_objs:
            o.select_set(True)
        context.view_layer.objects.active = ring_obj

        self.report({'INFO'},
            "Internal freewheel ratchet: %d teeth, %d pawls, ring Ø %.1f mm"
            % (self.tooth_count, self.pawl_count, ring_outer_r * 2))
        return {'FINISHED'}


# ── Registration ───────────────────────────────────────────────────────────────

classes = (OBJECT_OT_add_internal_ratchet,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
