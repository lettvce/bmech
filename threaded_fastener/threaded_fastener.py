bl_info = {
    "name": "Threaded Fastener Generator",
    "author": "",
    "version": (1, 0, 0),
    "blender": (5, 1, 0),
    "location": "View3D > Add > Mechanisms > Threaded Fastener",
    "description": "Parametric thread generator: bolt, nut body, clearance cutter, tap cutter.",
    "category": "Add Mesh",
}

"""
Threaded Fastener Generator

Outputs a helical thread solid. The user applies booleans manually:
  External + Additive    → union with a shaft cylinder to make a bolt
  External + Subtractive → difference from a cylinder to cut external threads
  Internal + Additive    → union with a tube bore to build nut thread ridges
  Internal + Subtractive → difference from a solid block to tap a threaded hole

All modes produce the same helix geometry; the distinction is which FDM
compensation is applied and how the result is named.
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil
from bpy.props import (
    FloatProperty, IntProperty, EnumProperty, FloatVectorProperty,
)


# ── Thread geometry ────────────────────────────────────────────────────────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    """Return (minor_r, crest_flat, flank_dz, thread_depth)."""
    ha    = max(radians(flank_deg / 2.0), radians(0.5))  # prevent tan(0) division
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)  # clamp negative flank_dz
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points outward (bolt ridge on outside of shaft)."""
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
    ]


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward (nut ridge on inside of bore)."""
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
    ]


# ── Mesh builder ───────────────────────────────────────────────────────────────

def _build_helix(bm, profile, pitch, height, res):
    """
    Sweep profile along a helix. Closed manifold:
    - Thread strip faces (flanks + crest) between consecutive rings
    - Root flat faces (minor_r quad) closing the gap between strips
    - Start/end cap quads sealing the open ends
    """
    n            = len(profile)
    profile_span = max(dz for _, dz in profile)
    steps        = int(ceil((height - profile_span) * res / pitch)) + 1
    rings = []
    for i in range(steps):
        ang  = 2.0 * pi * i / res
        zb   = pitch * i / res
        ring = [bm.verts.new((r * cos(ang), r * sin(ang), zb + dz))
                for r, dz in profile]
        rings.append(ring)

    for i in range(len(rings) - 1):
        for k in range(n - 1):
            bm.faces.new([rings[i][k], rings[i][k + 1],
                          rings[i + 1][k + 1], rings[i + 1][k]])
        bm.faces.new([rings[i][0], rings[i][n - 1],
                      rings[i + 1][n - 1], rings[i + 1][0]])

    bm.faces.new(list(reversed(rings[0])))
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])


def _link(context, bm, name, loc):
    me  = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    obj.location = loc
    context.collection.objects.link(obj)
    return obj


# ── Operator ───────────────────────────────────────────────────────────────────

class OBJECT_OT_add_threaded_fastener(bpy.types.Operator):
    """Parametric thread helix. Apply booleans manually to create bolts, nuts, or tapped holes."""
    bl_idname  = "object.add_threaded_fastener"
    bl_label   = "Add Threaded Fastener"
    bl_options = {'REGISTER', 'UNDO'}

    thread_type: EnumProperty(
        name="Thread Type",
        items=[
            ('EXTERNAL', "External", "Bolt-type: ridge on outside of shaft"),
            ('INTERNAL', "Internal", "Nut-type: female thread in bore"),
        ],
        default='EXTERNAL',
    )
    operation: EnumProperty(
        name="Operation",
        items=[
            ('ADDITIVE',    "Additive",    "Union with a shaft/bore"),
            ('SUBTRACTIVE', "Subtractive", "Difference to cut threads or tap a hole"),
        ],
        default='ADDITIVE',
    )

    diameter_mm: FloatProperty(
        name="Diameter (mm)", default=8.0, min=0.5, soft_max=100.0,
        description="Nominal major diameter (crest-to-crest)",
    )
    pitch_mm: FloatProperty(
        name="Pitch (mm)", default=1.25, min=0.1, soft_max=10.0,
        description="Distance between thread crests",
    )
    flank_angle_deg: FloatProperty(
        name="Flank Angle (°)", default=60.0, min=1.0, max=179.0,
        description="Included thread angle: 60° = metric/UNC, 55° = BSP, 29° = ACME",
    )
    truncation: FloatProperty(
        name="Truncation", default=0.125, min=0.0, max=0.3,
        description="Crest flat as fraction of pitch (ISO metric = 1/8). Root flat = 2×.",
    )
    height_mm: FloatProperty(
        name="Height (mm)", default=12.0, min=0.5, soft_max=200.0,
        description="Total thread length",
    )
    resolution: IntProperty(
        name="Resolution", default=32, min=8, soft_max=128,
        description="Steps per revolution",
    )
    outer_compensation_mm: FloatProperty(
        name="Outer Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
        description="FDM: shrinks major diameter. Use for External Additive (bolt). "
                    "Printed external features come out fatter than designed.",
    )
    inner_compensation_mm: FloatProperty(
        name="Inner Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
        description="FDM: expands major diameter. Use for subtractive and internal modes. "
                    "Printed holes come out tighter than designed.",
    )
    center_location: FloatVectorProperty(
        name="Location", size=3, default=(0.0, 0.0, 0.0), subtype='TRANSLATION',
    )

    def _derive(self):
        major_r = self.diameter_mm / 2.0
        minor_r, cf, fdz, depth = _thread_params(
            major_r, self.pitch_mm, self.flank_angle_deg, self.truncation,
        )
        if self.thread_type == 'EXTERNAL' and self.operation == 'ADDITIVE':
            major_r -= self.outer_compensation_mm
            minor_r  = major_r - depth
        elif (self.operation == 'SUBTRACTIVE' or
              (self.thread_type == 'INTERNAL' and self.operation == 'ADDITIVE')):
            major_r += self.inner_compensation_mm
            minor_r  = major_r - depth
        return major_r, minor_r, cf, fdz, depth

    def draw(self, context):
        layout = self.layout
        _, minor_r, _, _, depth = self._derive()

        box = layout.box()
        box.label(text="Mode")
        box.prop(self, "thread_type")
        box.prop(self, "operation")

        box = layout.box()
        box.label(text="Thread Geometry")
        box.prop(self, "diameter_mm")
        box.prop(self, "pitch_mm")
        box.prop(self, "flank_angle_deg")
        box.prop(self, "truncation")
        box.prop(self, "height_mm")
        box.prop(self, "resolution")
        box.label(text="Thread depth (derived): %.3f mm" % depth)
        box.label(text="Minor Ø (derived): %.3f mm" % (minor_r * 2.0))
        cf = self.truncation * self.pitch_mm
        rf = 2.0 * self.truncation * self.pitch_mm
        fdz = (self.pitch_mm - cf - rf) / 2.0
        if fdz <= 0:
            box.label(text="Truncation too high — no room for flanks at this pitch", icon='ERROR')
        if self.flank_angle_deg < 2.0:
            box.label(text="Flank angle near zero — thread depth will be very large", icon='ERROR')

        box = layout.box()
        box.label(text="FDM Compensation")
        box.prop(self, "outer_compensation_mm")
        box.prop(self, "inner_compensation_mm")

        box = layout.box()
        box.prop(self, "center_location")

    def execute(self, context):
        major_r, minor_r, cf, fdz, _ = self._derive()

        names = {
            ('EXTERNAL', 'ADDITIVE'):    "ExternalThread",
            ('EXTERNAL', 'SUBTRACTIVE'): "ExternalThreadCutter",
            ('INTERNAL', 'ADDITIVE'):    "InternalThread",
            ('INTERNAL', 'SUBTRACTIVE'): "TapCutter",
        }
        name = names[(self.thread_type, self.operation)]

        if (self.thread_type == 'EXTERNAL') == (self.operation == 'ADDITIVE'):
            prof = _external_profile(major_r, minor_r, cf, fdz)
        else:
            prof = _internal_profile(major_r, minor_r, cf, fdz)

        bm = bmesh.new()
        _build_helix(bm, prof, self.pitch_mm, self.height_mm, self.resolution)
        result = _link(context, bm, name, tuple(self.center_location))

        bpy.ops.object.select_all(action='DESELECT')
        result.select_set(True)
        context.view_layer.objects.active = result
        return {'FINISHED'}


classes = (OBJECT_OT_add_threaded_fastener,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
