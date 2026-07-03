"""
Sprocket Prototype
──────────────────
Run with Alt+P in the Text Editor. Popup dialog appears for parameter editing.

Geometry:
  N circular-arc pockets equally spaced on the pitch circle. Each pocket cradles
  a track pin. Between pockets, straight flanks rise to tooth tips.

  Pitch circle radius: R_p = pitch / (2 * sin(π / N))
  Pocket radius:       r_s = pin_radius + clearance
  Tip radius:          R_tip = sqrt(R_p² + r_s²) + tooth_height

  The profile is traced CCW: tooth tip → straight flank → pocket arc (CW, concave)
  → straight flank → tooth tip → ...

  PITCH must match the track's pin-to-pin distance.
  PIN Ø must match the track's pin outer diameter.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, BoolProperty
from math import pi, cos, sin, sqrt

BOOL_EPSILON = 0.001
BORE_SEGS    = 32


# ── Profile (pure function — copy to mechanisms_core unchanged) ───────────────

def sprocket_profile(N, pitch, roller_r, clearance_mm, tooth_height_mm, n_arc=8):
    """
    Returns a list of 2D (x, y) points tracing the sprocket profile CCW.
    Each pocket is a concave arc (CW) of radius roller_r + clearance_mm,
    centered on the pitch circle.
    """
    R_p   = pitch / (2.0 * sin(pi / N))
    r_s   = roller_r + clearance_mm
    R_tip = sqrt(R_p**2 + r_s**2) + tooth_height_mm

    profile = []

    for i in range(N):
        theta = i * 2.0 * pi / N
        cx    = R_p * cos(theta)
        cy    = R_p * sin(theta)

        # Tooth tip before this pocket (between tooth i-1 and tooth i)
        theta_tip = theta - pi / N
        profile.append((R_tip * cos(theta_tip), R_tip * sin(theta_tip)))

        # Approach point — CW side of pocket (angle slightly < theta from origin)
        profile.append((cx + r_s * sin(theta), cy - r_s * cos(theta)))

        # Pocket arc: CW from approach through inward bottom to exit
        # Parameterised as: arc_angle = (theta - π/2) - t*π  for t in (0, 1)
        for k in range(1, n_arc):
            t         = k / n_arc
            arc_angle = (theta - pi / 2.0) - t * pi
            profile.append((cx + r_s * cos(arc_angle), cy + r_s * sin(arc_angle)))

        # Exit point — CCW side of pocket (angle slightly > theta from origin)
        profile.append((cx - r_s * sin(theta), cy + r_s * cos(theta)))

    return profile


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_sprocket(bpy.types.Operator):
    """Sprocket — pin-pocket teeth mesh with a track or roller chain."""
    bl_idname  = "object.sprocket"
    bl_label   = "Sprocket"
    bl_options = {'REGISTER', 'UNDO'}

    tooth_count:       IntProperty(  name="Tooth Count",          default=16,  min=6,   soft_max=60)
    pitch:             FloatProperty(name="Pitch (mm)",            default=5.0, min=1.0, soft_max=30.0,
                                     description="Pin-to-pin distance — must match the track")
    pin_diameter:      FloatProperty(name="Pin Ø (mm)",            default=2.0, min=0.5, soft_max=10.0,
                                     description="Outer diameter of the track pin/roller")
    clearance_mm:      FloatProperty(name="Pocket Clearance (mm)", default=0.3, min=0.0, soft_max=1.0,
                                     description="Added to pin radius for the pocket — tune per printer")
    tooth_height_mm:   FloatProperty(name="Tooth Height (mm)",     default=1.5, min=0.1, soft_max=10.0,
                                     description="Tooth tip protrusion above pocket entry")
    width_mm:          FloatProperty(name="Width (mm)",            default=8.0, min=1.0, soft_max=50.0)
    bore_enable:       BoolProperty( name="Bore Hole",              default=True)
    bore_diameter:     FloatProperty(name="Bore Ø (mm)",           default=5.0, min=0.1, soft_max=50.0)
    bore_compensation: FloatProperty(name="Compensation (mm)",     default=0.2, min=0.0, soft_max=1.0,
                                     description="FDM printed holes come out tight — added to bore radius")
    n_arc:             IntProperty(  name="Pocket Resolution",     default=8,   min=3,   soft_max=24,
                                     description="Points per pocket arc")

    def _derived(self):
        R_p    = self.pitch / (2.0 * sin(pi / self.tooth_count))
        r_s    = self.pin_diameter / 2.0 + self.clearance_mm
        R_tip  = sqrt(R_p**2 + r_s**2) + self.tooth_height_mm
        bore_r = (self.bore_diameter / 2.0 + self.bore_compensation) if self.bore_enable else 0.0
        return R_p, r_s, R_tip, bore_r

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout
        R_p, r_s, R_tip, bore_r = self._derived()

        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "pitch")
        col.prop(self, "pin_diameter")
        col.prop(self, "clearance_mm")
        col.prop(self, "tooth_height_mm")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "width_mm")
        col.prop(self, "n_arc")
        layout.prop(self, "bore_enable")
        if self.bore_enable:
            sub = layout.column(align=True)
            sub.prop(self, "bore_diameter")
            sub.prop(self, "bore_compensation")

        layout.separator()
        box = layout.box()
        box.label(text="Pitch Ø: %.2f mm" % (R_p * 2))
        box.label(text="Outer Ø: %.2f mm" % (R_tip * 2))

        if r_s >= self.pitch / 2.0:
            layout.label(
                text="Pocket radius %.2f ≥ half pitch %.2f — pockets overlap"
                     % (r_s, self.pitch / 2.0),
                icon='ERROR',
            )
        if bore_r > 0 and bore_r >= R_p - r_s:
            layout.label(text="Bore too large for pocket radius", icon='ERROR')

    def execute(self, context):
        R_p, r_s, R_tip, bore_r = self._derived()

        if r_s >= self.pitch / 2.0:
            return {'CANCELLED'}

        profile = sprocket_profile(
            self.tooth_count,
            self.pitch,
            self.pin_diameter / 2.0,
            self.clearance_mm,
            self.tooth_height_mm,
            self.n_arc,
        )

        bm = bmesh.new()
        vb = [bm.verts.new((x, y, 0.0))           for x, y in profile]
        vt = [bm.verts.new((x, y, self.width_mm)) for x, y in profile]
        bm.verts.index_update()

        bm.faces.new(list(reversed(vb)))  # bottom — normals down
        bm.faces.new(vt)                  # top    — normals up

        n = len(profile)
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        me = bpy.data.meshes.new("SprocketMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj = bpy.data.objects.new("Sprocket", me)
        context.collection.objects.link(obj)

        if bore_r > 0:
            _apply_bore(context, obj, bore_r, self.width_mm)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {'FINISHED'}


def _apply_bore(context, obj, bore_r, width_mm):
    bm     = bmesh.new()
    angles = [2.0 * pi * i / BORE_SEGS for i in range(BORE_SEGS)]
    z0, z1 = -BOOL_EPSILON, width_mm + BOOL_EPSILON

    vb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angles]
    vt = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angles]
    bm.verts.index_update()

    for i in range(BORE_SEGS):
        ni = (i + 1) % BORE_SEGS
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])
    bm.faces.new(vb)
    bm.faces.new(list(reversed(vt)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me_cut  = bpy.data.meshes.new("__SprocketBoreMesh")
    bm.to_mesh(me_cut)
    bm.free()
    me_cut.update()

    cutter          = bpy.data.objects.new("__SprocketBore", me_cut)
    context.collection.objects.link(cutter)

    mod             = obj.modifiers.new("Bore", 'BOOLEAN')
    mod.operation   = 'DIFFERENCE'
    mod.object      = cutter
    mod.solver      = 'EXACT'

    with context.temp_override(active_object=obj):
        bpy.ops.object.modifier_apply(modifier="Bore")

    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_sprocket)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_sprocket)
bpy.ops.object.sprocket('INVOKE_DEFAULT')
