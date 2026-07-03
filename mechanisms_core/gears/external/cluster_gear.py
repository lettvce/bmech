"""
Cluster Gear Generator

A cluster gear is two involute gears of different sizes sharing a single axle,
printed as one monolithic piece. The smaller gear sits on top of the larger,
both centered on the same bore.

Mesh strategy:
  1. Build a single continuous solid in bmesh — bottom n-gon, bottom gear side
     walls, shoulder face connecting the two profiles at the junction, top gear
     side walls, top n-gon. No coincident faces.
  2. If axle_hole_mm > 0, cut the bore with a boolean difference cylinder
     (epsilon 0.001 on each end). The cutter is created, applied, and deleted
     before the operator returns.
"""

import bpy
import bmesh
from math import cos, sin, pi
from bpy.props import FloatProperty, IntProperty, BoolProperty, FloatVectorProperty

from . import involute_gear_rack
from .. import gear_matching

BORE_SEGMENTS = 32
BOOL_EPSILON  = 0.001


# ── Mesh builder ───────────────────────────────────────────────────────────────

def _build_cluster_solid(bm, bottom_profile, top_profile, width_bottom, width_top):
    """
    Build a solid cluster gear with no bore hole.

    At the junction (Z = width_bottom) a shoulder face fills the step between
    the two different gear profiles using triangle_fill — same technique used
    for bore annular faces. Works correctly as long as one profile is entirely
    inside the other at that Z plane, which is the normal cluster gear case
    (small gear OD < large gear dedendum).
    """
    n_bot   = len(bottom_profile)
    n_top   = len(top_profile)
    total_h = width_bottom + width_top

    vb0 = [bm.verts.new((x, y, 0.0))          for x, y in bottom_profile]
    vb1 = [bm.verts.new((x, y, width_bottom))  for x, y in bottom_profile]
    vt0 = [bm.verts.new((x, y, width_bottom))  for x, y in top_profile]
    vt1 = [bm.verts.new((x, y, total_h))       for x, y in top_profile]
    bm.verts.index_update()

    # Bottom face
    bm.faces.new(vb0)

    # Bottom gear side walls
    for i in range(n_bot):
        ni = (i + 1) % n_bot
        bm.faces.new([vb0[i], vb0[ni], vb1[ni], vb1[i]])

    # Shoulder at Z = width_bottom — fills step between the two outer profiles.
    # vb1 edges already exist (created implicitly by the side wall faces above),
    # so look them up rather than re-creating them.
    e_outer = [bm.edges.get((vb1[i], vb1[(i + 1) % n_bot])) for i in range(n_bot)]
    e_inner = [bm.edges.new((vt0[i], vt0[(i + 1) % n_top])) for i in range(n_top)]
    bm.edges.index_update()
    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                             edges=e_outer + e_inner, normal=(0.0, 0.0, 1.0))

    # Top gear side walls
    for i in range(n_top):
        ni = (i + 1) % n_top
        bm.faces.new([vt0[i], vt0[ni], vt1[ni], vt1[i]])

    # Top face
    bm.faces.new(list(reversed(vt1)))

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])


def _apply_bore(context, obj, bore_r, total_h, location):
    """Boolean-difference a bore cylinder through obj. Cutter is deleted after."""
    bm = bmesh.new()
    segs   = BORE_SEGMENTS
    angles = [2.0 * pi * i / segs for i in range(segs)]
    z0     = -BOOL_EPSILON
    z1     = total_h + BOOL_EPSILON
    vbot   = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angles]
    vtop   = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angles]
    bm.verts.index_update()
    for i in range(segs):
        ni = (i + 1) % segs
        bm.faces.new([vbot[i], vbot[ni], vtop[ni], vtop[i]])
    bm.faces.new(vbot)
    bm.faces.new(list(reversed(vtop)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me_cut = bpy.data.meshes.new("__ClusterBoreCutMesh")
    bm.to_mesh(me_cut)
    bm.free()
    me_cut.update()

    cutter          = bpy.data.objects.new("__ClusterBoreCut", me_cut)
    cutter.location = location
    context.collection.objects.link(cutter)

    mod          = obj.modifiers.new("Bore", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object    = cutter
    mod.solver    = 'EXACT'

    with context.temp_override(active_object=obj):
        bpy.ops.object.modifier_apply(modifier="Bore")

    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Operator ───────────────────────────────────────────────────────────────────

class OBJECT_OT_add_cluster_gear(bpy.types.Operator):
    """Cluster gear: two involute gears on one axle, printed as a single piece."""
    bl_idname  = "object.add_cluster_gear"
    bl_label   = "Add Cluster Gear"
    bl_options = {'REGISTER', 'UNDO'}

    bottom_teeth: IntProperty(
        name="Bottom Teeth", default=36, min=4, soft_max=200,
        description="Tooth count of the gear at the base",
    )
    top_teeth: IntProperty(
        name="Top Teeth", default=12, min=4, soft_max=200,
        description="Tooth count of the gear on top",
    )
    module:             FloatProperty(name="Module (mm)",        default=1.0,  min=0.1, max=50.0)
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    width_bottom:       FloatProperty(name="Bottom Width (mm)",  default=6.0,  min=0.5, soft_max=50.0)
    width_top:          FloatProperty(name="Top Width (mm)",     default=6.0,  min=0.5, soft_max=50.0)
    bore_enable: BoolProperty(name="Bore Hole", default=True)
    axle_hole_mm: FloatProperty(
        name="Bore Ø (mm)", default=5.0, min=0.1, soft_max=50.0,
    )
    axle_compensation_mm: FloatProperty(
        name="Compensation (mm)", default=0.2, min=0.0, soft_max=1.0,
        description="FDM: added to hole radius — printed holes come out tight",
    )
    def _compute(self):
        bore_r    = (self.axle_hole_mm / 2.0 + self.axle_compensation_mm) if self.bore_enable else 0.0
        total_h   = self.width_bottom + self.width_top
        bottom_od = self.module * (self.bottom_teeth + 2 * involute_gear_rack.ADDENDUM_COEFF)
        top_od    = self.module * (self.top_teeth    + 2 * involute_gear_rack.ADDENDUM_COEFF)
        min_ded_r = min(
            self.module * (t / 2.0 - involute_gear_rack.DEDENDUM_COEFF)
            for t in (self.bottom_teeth, self.top_teeth)
        )
        pa_max = min(
            gear_matching.max_pressure_angle_deg(t, involute_gear_rack.ADDENDUM_COEFF)
            for t in (self.bottom_teeth, self.top_teeth)
        )
        return bore_r, total_h, bottom_od, top_od, min_ded_r, pa_max

    def draw(self, context):
        layout = self.layout
        bore_r, total_h, bottom_od, top_od, min_ded_r, pa_max = self._compute()

        box = layout.box()
        box.label(text="Bottom Gear")
        box.prop(self, "bottom_teeth")
        box.prop(self, "width_bottom")
        box.label(text="OD: %.2f mm" % bottom_od)

        box = layout.box()
        box.label(text="Top Gear")
        box.prop(self, "top_teeth")
        box.prop(self, "width_top")
        box.label(text="OD: %.2f mm" % top_od)

        box = layout.box()
        box.label(text="Shared")
        box.prop(self, "module")
        box.prop(self, "pressure_angle_deg")
        box.prop(self, "bore_enable")
        if self.bore_enable:
            sub = box.column(align=True)
            sub.prop(self, "axle_hole_mm")
            sub.prop(self, "axle_compensation_mm")
        box.label(text="Total height: %.2f mm" % total_h)
        if bore_r > 0 and bore_r >= min_ded_r:
            box.label(text="Axle hole too large — max Ø %.2f mm for smallest gear"
                      % (min_ded_r * 2), icon='ERROR')
        box.label(text="Max pressure angle for these teeth: %.1f°" % pa_max)

    def execute(self, context):
        gear_matching.clamp_pressure_angle(
            self,
            (self.bottom_teeth, involute_gear_rack.ADDENDUM_COEFF),
            (self.top_teeth, involute_gear_rack.ADDENDUM_COEFF),
        )
        bore_r, total_h, _, _, _, pa_max = self._compute()
        loc = tuple(context.scene.cursor.location)

        bottom_prof = involute_gear_rack.build_gear_profile(
            self.module, self.bottom_teeth, self.pressure_angle_deg)
        top_prof = involute_gear_rack.build_gear_profile(
            self.module, self.top_teeth, self.pressure_angle_deg)

        bm = bmesh.new()
        _build_cluster_solid(bm, bottom_prof, top_prof, self.width_bottom, self.width_top)

        me = bpy.data.meshes.new("ClusterGearMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj          = bpy.data.objects.new("ClusterGear", me)
        obj.location = loc
        context.collection.objects.link(obj)

        if bore_r > 0:
            _apply_bore(context, obj, bore_r, total_h, loc)

        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'}, "Cluster gear: %d / %d teeth, %.2f mm tall"
                    % (self.bottom_teeth, self.top_teeth, total_h))
        return {'FINISHED'}


# ── Registration ───────────────────────────────────────────────────────────────

classes = (OBJECT_OT_add_cluster_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
