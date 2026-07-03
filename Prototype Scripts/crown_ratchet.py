"""
Crown Ratchet Prototype
────────────────────────
Run with Alt+P in the Text Editor. Popup appears for parameter editing.

Sawtooth teeth on the flat face of a disk — axial teeth instead of radial.
Each tooth has a steep locking wall and a gradual ramp.

  Lock angle  = 0°: straight vertical wall — reliable catch
  Lock angle  < 0°: undercut — stronger catch, may bind on release
  Lock angle  > 0°: forward lean — easier to disengage (not a true ratchet)

Two crown ratchets facing each other (one flipped) form a one-way clutch.
The ramp lets them slip past each other in one direction; the steep face locks
them in the other. A spring holds them in contact.

Mesh strategy: disk body with flat top face. Tooth root faces overlap the
disk top — slicers union them. Teeth are triangular prisms extruded radially
from r_inner to r_outer.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, tan, radians

BOOL_EPSILON = 0.001
BORE_SEGS    = 32


# ── Mesh helpers ──────────────────────────────────────────────────────────────

def _make_disk(bm, outer_r, bore_r, z_face, segs=64):
    angles = [2.0 * pi * i / segs for i in range(segs)]
    n      = segs

    vb_out = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), 0.0))     for a in angles]
    vt_out = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), z_face))  for a in angles]

    if bore_r > 0:
        vb_in = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), 0.0))    for a in angles]
        vt_in = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z_face)) for a in angles]
        bm.verts.index_update()

        eb_out = [bm.edges.new((vb_out[i], vb_out[(i+1)%n])) for i in range(n)]
        eb_in  = [bm.edges.new((vb_in[i],  vb_in[(i+1)%n]))  for i in range(n)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=eb_out + eb_in, normal=(0.0, 0.0, -1.0))

        et_out = [bm.edges.new((vt_out[i], vt_out[(i+1)%n])) for i in range(n)]
        et_in  = [bm.edges.new((vt_in[i],  vt_in[(i+1)%n]))  for i in range(n)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=et_out + et_in, normal=(0.0, 0.0, 1.0))

        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([vb_in[ni], vb_in[i], vt_in[i], vt_in[ni]])
    else:
        bm.verts.index_update()
        bm.faces.new(vb_out)
        bm.faces.new(list(reversed(vt_out)))

    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb_out[i], vb_out[ni], vt_out[ni], vt_out[i]])


def _make_tooth(bm, tooth_idx, N, r_inner, r_outer, z_face, tooth_height,
                lock_angle_deg, n_radial):
    """
    One crown ratchet tooth. Triangular prism extruded radially.

    In the (tangential, Z) plane at each radius r:
      V0: root at theta_start (base of steep face)
      V1: tip at theta_start + lock_tang/r, z = z_face + tooth_height
      V2: root at theta_end (end of ramp, start of next tooth)

    lock_angle = 0  → V1 directly above V0 (vertical wall)
    lock_angle < 0  → V1 behind V0 (undercut)
    lock_angle > 0  → V1 ahead of V0 (forward lean)
    """
    pitch_arc   = 2.0 * pi / N
    theta_start = tooth_idx * pitch_arc
    theta_end   = (tooth_idx + 1) * pitch_arc
    lock_tang   = tooth_height * tan(radians(lock_angle_deg))  # tangential tip offset at any r

    r_vals = [r_inner + (r_outer - r_inner) * j / (n_radial - 1) for j in range(n_radial)]

    V0, V1, V2 = [], [], []
    for r in r_vals:
        lock_ang = lock_tang / r   # angular offset varies with r
        theta_tip = theta_start + lock_ang

        V0.append(bm.verts.new((r * cos(theta_start), r * sin(theta_start), z_face)))
        V1.append(bm.verts.new((r * cos(theta_tip),   r * sin(theta_tip),   z_face + tooth_height)))
        V2.append(bm.verts.new((r * cos(theta_end),   r * sin(theta_end),   z_face)))

    bm.verts.index_update()
    NR = n_radial

    # Steep face (locking wall) — faces "backward" (against ratchet direction)
    for j in range(NR - 1):
        bm.faces.new([V0[j], V0[j+1], V1[j+1], V1[j]])

    # Ramp face — faces "forward" (allows slip in ratchet direction)
    for j in range(NR - 1):
        bm.faces.new([V1[j], V1[j+1], V2[j+1], V2[j]])

    # Root face (at z_face — overlaps disk top, slicer unions)
    for j in range(NR - 1):
        bm.faces.new([V0[j], V2[j+1], V2[j], V0[j+1]])

    # Inner and outer radial caps (triangular)
    bm.faces.new([V0[0],  V1[0],  V2[0]])
    bm.faces.new([V0[-1], V2[-1], V1[-1]])


def _apply_bore(context, obj, bore_r, total_h):
    bm     = bmesh.new()
    angles = [2.0 * pi * i / BORE_SEGS for i in range(BORE_SEGS)]
    z0, z1 = -BOOL_EPSILON, total_h + BOOL_EPSILON

    vb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angles]
    vt = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angles]
    bm.verts.index_update()

    for i in range(BORE_SEGS):
        ni = (i + 1) % BORE_SEGS
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])
    bm.faces.new(vb)
    bm.faces.new(list(reversed(vt)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me_cut = bpy.data.meshes.new("__CRBoreMesh")
    bm.to_mesh(me_cut)
    bm.free()
    me_cut.update()

    cutter = bpy.data.objects.new("__CRBore", me_cut)
    context.collection.objects.link(cutter)

    mod           = obj.modifiers.new("Bore", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object    = cutter
    mod.solver    = 'EXACT'

    with context.temp_override(active_object=obj):
        bpy.ops.object.modifier_apply(modifier="Bore")

    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_crown_ratchet(bpy.types.Operator):
    """Crown ratchet — face sawtooth teeth for axial one-way locking."""
    bl_idname  = "object.crown_ratchet"
    bl_label   = "Crown Ratchet"
    bl_options = {'REGISTER', 'UNDO'}

    tooth_count:       IntProperty(  name="Tooth Count",           default=12,  min=3,    soft_max=72)
    tooth_height:      FloatProperty(name="Tooth Height (mm)",     default=3.0, min=0.5,  soft_max=20.0,
                                     description="How far teeth protrude above the disk face")
    r_inner:           FloatProperty(name="Inner Radius (mm)",     default=8.0, min=1.0,  soft_max=100.0,
                                     description="Inner edge of the tooth ring")
    r_outer:           FloatProperty(name="Outer Radius (mm)",     default=20.0, min=2.0, soft_max=200.0,
                                     description="Outer edge of the tooth ring = disk outer radius")
    disk_thickness:    FloatProperty(name="Disk Thickness (mm)",   default=4.0, min=1.0,  soft_max=50.0,
                                     description="Body height below teeth")
    bore_enable:       BoolProperty( name="Bore Hole",              default=True)
    bore_diameter:     FloatProperty(name="Bore Ø (mm)",           default=6.0, min=0.1,  soft_max=50.0)
    bore_compensation: FloatProperty(name="Compensation (mm)",     default=0.2, min=0.0,  soft_max=1.0,
                                     description="FDM holes print tight — added to bore radius")
    lock_angle_deg:    FloatProperty(name="Lock Angle (°)",        default=0.0, min=-30.0, max=30.0,
                                     description="Steep face angle: 0=vertical, <0=undercut, >0=forward lean")
    n_radial:          IntProperty(  name="Radial Slices",         default=4,   min=2,    soft_max=20,
                                     description="Radial divisions per tooth")

    def _derived(self):
        bore_r   = (self.bore_diameter / 2.0 + self.bore_compensation) if self.bore_enable else 0.0
        total_h  = self.disk_thickness + self.tooth_height
        ramp_deg = 90.0 - (self.tooth_height /
                   (self.r_outer * 2.0 * pi / self.tooth_count) * 90.0)
        return bore_r, total_h, ramp_deg

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout
        bore_r, total_h, ramp_deg = self._derived()

        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "tooth_height")
        col.prop(self, "lock_angle_deg")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "r_inner")
        col.prop(self, "r_outer")
        col.prop(self, "disk_thickness")
        col.prop(self, "n_radial")
        layout.prop(self, "bore_enable")
        if self.bore_enable:
            sub = layout.column(align=True)
            sub.prop(self, "bore_diameter")
            sub.prop(self, "bore_compensation")

        layout.separator()
        box = layout.box()
        box.label(text="Total height: %.2f mm" % total_h)
        box.label(text="Outer Ø: %.2f mm"      % (self.r_outer * 2))

        if self.r_inner >= self.r_outer:
            layout.label(text="Inner radius must be less than outer radius", icon='ERROR')
        if self.bore_enable and bore_r > 0 and bore_r >= self.r_inner:
            layout.label(text="Bore exceeds inner tooth radius", icon='ERROR')

    def execute(self, context):
        bore_r, total_h, _ = self._derived()

        if self.r_inner >= self.r_outer:
            return {'CANCELLED'}

        bm = bmesh.new()

        _make_disk(bm, self.r_outer, bore_r if bore_r > 0 else 0.0,
                   self.disk_thickness)

        for i in range(self.tooth_count):
            _make_tooth(bm, i, self.tooth_count,
                        self.r_inner, self.r_outer,
                        self.disk_thickness, self.tooth_height,
                        self.lock_angle_deg, self.n_radial)

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        me = bpy.data.meshes.new("CrownRatchetMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj = bpy.data.objects.new("CrownRatchet", me)
        context.collection.objects.link(obj)

        if bore_r > 0 and bore_r < self.r_inner:
            _apply_bore(context, obj, bore_r, total_h)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_crown_ratchet)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_crown_ratchet)
bpy.ops.object.crown_ratchet('INVOKE_DEFAULT')
