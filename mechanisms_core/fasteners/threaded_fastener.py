"""
Threaded Fastener Generator

Outputs a helical thread solid. The user applies booleans manually:
  External + Additive    → union with a shaft cylinder to make a bolt
  External + Subtractive → difference from a cylinder to cut external threads
  Internal + Additive    → union with a tube bore to build nut thread ridges
  Internal + Subtractive → difference from a solid block to tap a threaded hole

All modes produce the same helix geometry; the distinction is which FDM
compensation is applied and how the result is named.

Thread profile is trapezoidal (4 points: root → rising flank → crest → falling flank).
Root flat faces close the gap between consecutive thread strips, making the mesh
a closed manifold. Start/end caps seal the open helix ends.
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil
from bpy.props import (
    FloatProperty, IntProperty, EnumProperty, FloatVectorProperty,
)
from . import fastener_matching


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

    def bmech_sync_target(self, context):
        fastener_matching.sync_raw_thread(self, context.window_manager.bmech_fastener_target)

    def invoke(self, context, event):
        fastener_matching.reset_target(context)
        return self.execute(context)

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
        description="FDM: use for External Additive (bolt). Printed external "
                    "features tend to shrink — added to major diameter to compensate.",
    )
    inner_compensation_mm: FloatProperty(
        name="Inner Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
        description="FDM: expands major diameter. Use for subtractive and internal modes. "
                    "Printed holes come out tighter than designed.",
    )
    def _derive(self):
        major_r = self.diameter_mm / 2.0
        minor_r, cf, fdz, depth = _thread_params(
            major_r, self.pitch_mm, self.flank_angle_deg, self.truncation,
        )
        if self.thread_type == 'EXTERNAL' and self.operation == 'ADDITIVE':
            major_r += self.outer_compensation_mm
            minor_r  = major_r - depth
        elif (self.operation == 'SUBTRACTIVE' or
              (self.thread_type == 'INTERNAL' and self.operation == 'ADDITIVE')):
            major_r += self.inner_compensation_mm
            minor_r  = major_r - depth
        return major_r, minor_r, cf, fdz, depth

    def draw(self, context):
        layout = self.layout
        _, minor_r, _, _, depth = self._derive()

        layout.prop(context.window_manager, "bmech_fastener_target", text="Match Target")
        has_target = context.window_manager.bmech_fastener_target is not None

        box = layout.box()
        box.label(text="Mode")
        # thread_type is forced to the opposite of the target's own
        # orientation (an external target needs an internal thread here,
        # and vice versa) — see fastener_matching.sync_raw_thread. operation
        # (additive/subtractive) only controls HOW that thread gets built,
        # not whether it fits, so it stays free even with a target set.
        type_row = box.column(align=True)
        type_row.enabled = not has_target
        type_row.prop(self, "thread_type")
        box.prop(self, "operation")

        box = layout.box()
        box.label(text="Thread Geometry")
        # All four dimensions freeze together whenever a target is set —
        # same reasoning as hex_bolt.py/hex_nut.py: a mating pair needs all
        # four to match simultaneously, no partial-match case.
        driven = box.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "diameter_mm")
        driven.prop(self, "pitch_mm")
        driven.prop(self, "flank_angle_deg")
        driven.prop(self, "truncation")
        box.prop(self, "height_mm")
        box.prop(self, "resolution")
        box.label(text="Thread depth (derived): %.3f mm" % depth)
        box.label(text="Minor Ø (derived): %.3f mm" % (minor_r * 2.0))
        ha = radians(self.flank_angle_deg / 2.0)
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
        result = _link(context, bm, name, tuple(context.scene.cursor.location))

        # Stamped by thread_type alone, independent of operation: an
        # "ExternalThreadCutter" (subtractive) still represents an external
        # thread once the user finishes the boolean by hand — its own
        # profile shape uses _internal_profile as the cutter tool (see
        # execute()'s profile-selection logic above), but the RESULT it
        # produces is external, which is what other parts need to match
        # against, not the cutter's own current shape.
        kind = "external_thread" if self.thread_type == 'EXTERNAL' else "internal_thread"
        fastener_matching.stamp_thread(result, kind, self.diameter_mm,
                                        self.pitch_mm, self.flank_angle_deg, self.truncation)

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
