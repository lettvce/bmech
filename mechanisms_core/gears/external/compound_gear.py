"""
Compound Gear Generator

N meshing stages where each intermediate shaft carries two gears — the driven gear
from the previous stage and the driver into the next. Overall ratio = product of all
stage ratios (driven / driver per stage).

Each stage occupies its own Z plane separated by width + 1 mm gap, so the two gears
on each shared shaft sit at the same XY but different Z, showing they are co-axial
separate parts.

Solid mesh geometry (no Solidify modifier) — reuses build_gear_profile() from
involute_gear_rack for profile points, then extrudes and bores here.
"""

import bpy
import bmesh
import math
from math import cos, sin, pi
from bpy.props import (
    FloatProperty, IntProperty, BoolProperty, FloatVectorProperty,
)
from . import involute_gear_rack
from .. import gear_matching

BORE_SEGMENTS = 32
STAGE_GAP_MM  = 1.0   # Z gap between stage planes on shared shafts


# ── Solid gear mesh builder ────────────────────────────────────────────────────

def _build_solid_gear(context, name, profile_2d, width_mm, bore_r, location, rotation_z=0.0):
    """
    Solid gear with an axle bore. No Solidify modifier.
    bore_r == 0 → solid n-gon top/bottom faces (no through-hole).
    """
    bm = bmesh.new()
    n  = len(profile_2d)

    vb_out = [bm.verts.new((x, y, 0.0))      for x, y in profile_2d]
    vt_out = [bm.verts.new((x, y, width_mm)) for x, y in profile_2d]
    bm.verts.index_update()

    if bore_r > 0:
        m      = BORE_SEGMENTS
        angles = [2.0 * pi * i / m for i in range(m)]
        vb_in  = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), 0.0))      for a in angles]
        vt_in  = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), width_mm)) for a in angles]
        bm.verts.index_update()

        # Bottom annular face (outer profile + bore circle)
        eb_out = [bm.edges.new((vb_out[i], vb_out[(i + 1) % n])) for i in range(n)]
        eb_in  = [bm.edges.new((vb_in[i],  vb_in[(i + 1) % m]))  for i in range(m)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=eb_out + eb_in, normal=(0.0, 0.0, -1.0))

        # Top annular face
        et_out = [bm.edges.new((vt_out[i], vt_out[(i + 1) % n])) for i in range(n)]
        et_in  = [bm.edges.new((vt_in[i],  vt_in[(i + 1) % m]))  for i in range(m)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=et_out + et_in, normal=(0.0, 0.0, 1.0))

        # Bore walls — inward normals, reversed winding
        for i in range(m):
            ni = (i + 1) % m
            bm.faces.new([vb_in[ni], vb_in[i], vt_in[i], vt_in[ni]])
    else:
        bm.faces.new(vb_out)
        bm.faces.new(list(reversed(vt_out)))

    # Outer side walls
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb_out[i], vb_out[ni], vt_out[ni], vt_out[i]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj                = bpy.data.objects.new(name, me)
    obj.location       = location
    obj.rotation_euler = (0.0, 0.0, rotation_z)
    context.collection.objects.link(obj)
    return obj


# ── Operator ───────────────────────────────────────────────────────────────────

class OBJECT_OT_add_compound_gear(bpy.types.Operator):
    """Compound gear train: N stages of driver/driven pairs, auto-positioned along X."""
    bl_idname  = "object.add_compound_gear"
    bl_label   = "Add Compound Gear"
    bl_options = {'REGISTER', 'UNDO'}

    stage_count: IntProperty(name="Stage Count", default=2, min=1, max=4)

    # Fixed per-stage tooth counts — up to 4 stages
    s1_driver: IntProperty(name="Driver Teeth", default=12, min=4, soft_max=200)
    s1_driven: IntProperty(name="Driven Teeth", default=36, min=4, soft_max=200)
    s2_driver: IntProperty(name="Driver Teeth", default=12, min=4, soft_max=200)
    s2_driven: IntProperty(name="Driven Teeth", default=36, min=4, soft_max=200)
    s3_driver: IntProperty(name="Driver Teeth", default=12, min=4, soft_max=200)
    s3_driven: IntProperty(name="Driven Teeth", default=24, min=4, soft_max=200)
    s4_driver: IntProperty(name="Driver Teeth", default=12, min=4, soft_max=200)
    s4_driven: IntProperty(name="Driven Teeth", default=24, min=4, soft_max=200)

    module:               FloatProperty(name="Module (mm)",          default=1.0,  min=0.1, max=50.0)
    pressure_angle_deg:   FloatProperty(name="Pressure Angle (°)",   default=20.0, min=10.0, max=45.0)
    width_mm:             FloatProperty(name="Width (mm)",            default=6.0,  min=0.5, soft_max=50.0)
    bore_enable:          BoolProperty( name="Bore Hole",              default=True)
    axle_hole_mm:         FloatProperty(name="Bore Ø (mm)",            default=5.0,  min=0.1, soft_max=50.0)
    axle_compensation_mm: FloatProperty(name="Compensation (mm)",       default=0.2,  min=0.0, soft_max=1.0,
                                        description="FDM: added to hole radius — printed holes come out tight")

    parent_under_empty: BoolProperty(name="Parent Under Empty", default=True)

    # ── Property name lists (avoids getattr gymnastics in execute) ─────────────
    _DRIVER_PROPS = ('s1_driver', 's2_driver', 's3_driver', 's4_driver')
    _DRIVEN_PROPS = ('s1_driven', 's2_driven', 's3_driven', 's4_driven')

    def _compute(self):
        stages = []
        for i in range(self.stage_count):
            dr = getattr(self, self._DRIVER_PROPS[i])
            dn = getattr(self, self._DRIVEN_PROPS[i])
            dr_r = self.module * dr / 2.0
            dn_r = self.module * dn / 2.0
            stages.append({
                'driver_t': dr, 'driven_t': dn,
                'driver_r': dr_r, 'driven_r': dn_r,
                'ratio': dn / dr,
                'center_dist': dr_r + dn_r,
            })
        overall = 1.0
        for s in stages:
            overall *= s['ratio']
        shaft_x = [0.0]
        for s in stages:
            shaft_x.append(shaft_x[-1] + s['center_dist'])
        pa_max = min(
            gear_matching.max_pressure_angle_deg(t, involute_gear_rack.ADDENDUM_COEFF)
            for s in stages
            for t in (s['driver_t'], s['driven_t'])
        )
        return stages, overall, shaft_x, pa_max

    def draw(self, context):
        layout = self.layout
        stages, overall, shaft_x, pa_max = self._compute()

        layout.prop(self, "stage_count")

        for i, s in enumerate(stages):
            box = layout.box()
            box.label(text="Stage %d  —  1 : %.3f" % (i + 1, s['ratio']))
            box.prop(self, self._DRIVER_PROPS[i])
            box.prop(self, self._DRIVEN_PROPS[i])

        box = layout.box()
        box.label(text="Overall ratio: 1 : %.3f" % overall)
        box.label(text="Total span: %.2f mm" % shaft_x[-1])

        box = layout.box()
        box.label(text="Gear Parameters")
        box.prop(self, "module")
        box.prop(self, "pressure_angle_deg")
        box.prop(self, "width_mm")
        box.prop(self, "bore_enable")
        if self.bore_enable:
            sub = box.column(align=True)
            sub.prop(self, "axle_hole_mm")
            sub.prop(self, "axle_compensation_mm")

        # Warn if axle bore eats into the smallest gear's dedendum circle
        bore_r = (self.axle_hole_mm / 2.0 + self.axle_compensation_mm) if self.bore_enable else 0.0
        if bore_r > 0:
            min_ded = min(
                self.module * (t / 2.0 - involute_gear_rack.DEDENDUM_COEFF)
                for s in stages
                for t in (s['driver_t'], s['driven_t'])
            )
            if bore_r >= min_ded:
                box.label(text="Axle hole too large — max Ø %.2f mm for smallest gear" % (min_ded * 2), icon='ERROR')
        box.label(text="Max pressure angle for these teeth: %.1f°" % pa_max)

        box = layout.box()
        box.label(text="Placement")
        box.prop(self, "parent_under_empty")

    def execute(self, context):
        stages, overall, shaft_x, pa_max = self._compute()
        gear_matching.clamp_pressure_angle(
            self,
            *[(t, involute_gear_rack.ADDENDUM_COEFF)
              for s in stages for t in (s['driver_t'], s['driven_t'])]
        )
        bore_r = (self.axle_hole_mm / 2.0 + self.axle_compensation_mm) if self.bore_enable else 0.0
        cx, cy, cz = context.scene.cursor.location
        all_objs = []

        for i, s in enumerate(stages):
            stage_z = cz + i * (self.width_mm + STAGE_GAP_MM)

            driver_prof = involute_gear_rack.build_gear_profile(
                self.module, s['driver_t'], self.pressure_angle_deg)
            driven_prof = involute_gear_rack.build_gear_profile(
                self.module, s['driven_t'], self.pressure_angle_deg)

            driver_name = "GearInput"  if i == 0                    else "GearS%dDriver" % (i + 1)
            driven_name = "GearOutput" if i == self.stage_count - 1 else "GearS%dDriven" % (i + 1)

            driver_obj = _build_solid_gear(
                context, driver_name, driver_prof, self.width_mm, bore_r,
                location=(cx + shaft_x[i], cy, stage_z),
            )
            driven_obj = _build_solid_gear(
                context, driven_name, driven_prof, self.width_mm, bore_r,
                location=(cx + shaft_x[i + 1], cy, stage_z),
            )
            all_objs.extend([driver_obj, driven_obj])

        if self.parent_under_empty:
            empty = bpy.data.objects.new("CompoundGear", None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = max(shaft_x[-1] * 0.1, 5.0)
            empty.location = (cx, cy, cz)
            context.collection.objects.link(empty)
            for obj in all_objs:
                obj.parent = empty
                obj.matrix_parent_inverse = empty.matrix_world.inverted()

        for o in context.selected_objects:
            o.select_set(False)
        for o in all_objs:
            o.select_set(True)
        context.view_layer.objects.active = all_objs[0]

        self.report({'INFO'}, "Compound gear: %d stage%s, 1 : %.3f overall ratio"
                    % (self.stage_count, 's' if self.stage_count > 1 else '', overall))
        return {'FINISHED'}


# ── Registration ───────────────────────────────────────────────────────────────

classes = (OBJECT_OT_add_compound_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
