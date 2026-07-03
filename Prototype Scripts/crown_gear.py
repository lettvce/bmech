"""
Crown Gear Prototype
────────────────────
Run with Alt+P. Tweak parameters live in the Redo panel (F9 or bottom-left
of viewport). Every change rebuilds the mesh immediately.

Tooth geometry:
  Straight-flank (rack) approximation with correct pressure angle.
  Profile varies with radius — teeth taper correctly from root to tip and from
  inner to outer radius. Pitch plane sits DEDENDUM above the disk face so the
  full tooth depth (2.25 × module) protrudes above the face.

  At radius r, height z:
      angular half-width = (r × pitch_arc/4 + (z_pitch − z) × tan(PA)) / r

  Tooth root faces overlap the disk top face — slicers union them automatically.
  Use Merge by Distance in Blender if you need a clean manifold before export.

Meshes with: a standard spur gear of same module, perpendicular shaft.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, tan, radians


# ── Mesh helpers (pure functions — move to mechanisms_core unchanged) ──────────

def _ang_half(r, tooth_pitch_arc, z_pitch, pa_rad, z):
    tang = r * tooth_pitch_arc / 4.0 + (z_pitch - z) * tan(pa_rad)
    return max(tang / r, 1e-6)


def _make_disk(bm, outer_r, bore_r, z_face, segs):
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


def _make_tooth(bm, center_angle, r_inner, r_outer, z_face, tooth_height,
                tooth_pitch_arc, z_pitch, pa_rad, n_radial, n_z):
    r_vals = [r_inner + (r_outer - r_inner) * j / (n_radial - 1) for j in range(n_radial)]
    z_vals = [z_face  + tooth_height        * k / (n_z - 1)      for k in range(n_z)]

    Lv, Rv = [], []
    for r in r_vals:
        lr, rr = [], []
        for z in z_vals:
            ah = _ang_half(r, tooth_pitch_arc, z_pitch, pa_rad, z)
            lr.append(bm.verts.new((r * cos(center_angle - ah), r * sin(center_angle - ah), z)))
            rr.append(bm.verts.new((r * cos(center_angle + ah), r * sin(center_angle + ah), z)))
        Lv.append(lr)
        Rv.append(rr)

    bm.verts.index_update()
    NR, NZ = n_radial, n_z

    for j in range(NR - 1):
        for k in range(NZ - 1):
            bm.faces.new([Lv[j][k], Lv[j+1][k], Lv[j+1][k+1], Lv[j][k+1]])  # left flank
            bm.faces.new([Rv[j][k], Rv[j][k+1], Rv[j+1][k+1], Rv[j+1][k]])  # right flank

    for j in range(NR - 1):
        bm.faces.new([Lv[j][-1], Rv[j][-1], Rv[j+1][-1], Lv[j+1][-1]])      # tip
        bm.faces.new([Lv[j][0],  Lv[j+1][0], Rv[j+1][0], Rv[j][0]])          # root

    for k in range(NZ - 1):
        bm.faces.new([Lv[0][k],  Lv[0][k+1],  Rv[0][k+1],  Rv[0][k]])        # inner cap
        bm.faces.new([Lv[-1][k], Rv[-1][k],   Rv[-1][k+1], Lv[-1][k+1]])     # outer cap


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_crown_gear(bpy.types.Operator):
    """Crown gear — face teeth mesh with a perpendicular spur gear."""
    bl_idname  = "object.crown_gear"
    bl_label   = "Crown Gear"
    bl_options = {'REGISTER', 'UNDO'}

    tooth_count:        IntProperty(  name="Tooth Count",         default=24,   min=6,   soft_max=200)
    module:             FloatProperty(name="Module (mm)",          default=2.0,  min=0.1, soft_max=10.0)
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)",   default=20.0, min=10.0, max=45.0)
    disk_thickness:     FloatProperty(name="Disk Thickness (mm)",  default=6.0,  min=1.0, soft_max=50.0)
    outer_radius:       FloatProperty(name="Outer Radius (mm)",    default=40.0, min=1.0, soft_max=200.0)
    bore_enable:        BoolProperty( name="Bore Hole",             default=True)
    bore_diameter:      FloatProperty(name="Bore Ø (mm)",          default=8.0,  min=0.1, soft_max=50.0)
    bore_compensation:  FloatProperty(name="Compensation (mm)",    default=0.2,  min=0.0, soft_max=1.0,
                                      description="FDM holes print tight — added to bore radius")
    n_radial:           IntProperty(  name="Radial Resolution",    default=6,    min=2,   soft_max=20,
                                      description="Slices per tooth radially — more = smoother taper")
    n_z:                IntProperty(  name="Height Resolution",    default=5,    min=2,   soft_max=20,
                                      description="Z samples per tooth profile")

    def _derived(self):
        pitch_r      = self.module * self.tooth_count / 2.0
        addendum     = self.module * 1.0
        dedendum     = self.module * 1.25
        tooth_height = addendum + dedendum
        tooth_radial = self.module * 2.0
        r_inner      = pitch_r - tooth_radial / 2.0
        r_outer      = pitch_r + tooth_radial / 2.0
        pitch_arc    = 2.0 * pi / self.tooth_count
        pa_rad       = radians(self.pressure_angle_deg)
        z_face       = self.disk_thickness
        z_pitch      = z_face + dedendum
        return pitch_r, tooth_height, r_inner, r_outer, pitch_arc, pa_rad, z_face, z_pitch

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    def draw(self, context):
        layout = self.layout
        pitch_r, tooth_height, r_inner, r_outer, *_ = self._derived()

        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "disk_thickness")
        col.prop(self, "outer_radius")
        col.prop(self, "bore_enable")
        if self.bore_enable:
            sub = col.column(align=True)
            sub.prop(self, "bore_diameter")
            sub.prop(self, "bore_compensation")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "n_radial")
        col.prop(self, "n_z")

        layout.separator()
        info = layout.box()
        info.label(text="Pitch Ø: %.2f mm" % (pitch_r * 2))
        info.label(text="Tooth height: %.2f mm" % tooth_height)

        if r_inner <= 0:
            layout.label(text="Tooth radial depth exceeds pitch radius — reduce module", icon='ERROR')
        if self.bore_enable and (self.bore_diameter / 2.0 + self.bore_compensation) >= r_inner:
            layout.label(text="Bore too large for tooth inner radius", icon='ERROR')
        if r_outer >= self.outer_radius:
            layout.label(text="Teeth extend past disk outer radius — increase outer radius", icon='ERROR')

    def execute(self, context):
        pitch_r, tooth_height, r_inner, r_outer, pitch_arc, pa_rad, z_face, z_pitch = self._derived()
        bore_r = (self.bore_diameter / 2.0 + self.bore_compensation) if self.bore_enable else 0.0

        if r_inner <= 0 or r_outer >= self.outer_radius:
            return {'CANCELLED'}

        bm = bmesh.new()
        _make_disk(bm, self.outer_radius, bore_r, z_face, segs=64)
        for i in range(self.tooth_count):
            _make_tooth(bm, i * pitch_arc, r_inner, r_outer, z_face, tooth_height,
                        pitch_arc, z_pitch, pa_rad, self.n_radial, self.n_z)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        me = bpy.data.meshes.new("CrownGearMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj = bpy.data.objects.new("CrownGear", me)
        context.collection.objects.link(obj)
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_crown_gear)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_crown_gear)
bpy.ops.object.crown_gear('INVOKE_DEFAULT')
