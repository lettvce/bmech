"""
Container (Revolve) Prototype
───────────────────────────────
Run with Alt+P. Popup dialog for parameters.

Cup/jar body as a surface of revolution — a 2D (r, z) profile spun 360°
around the Z axis. This is a NEW construction technique for this project:
every prior generator either extrudes a profile along an axis or scales it
per Z-slice (bevel gear); this one rotates a fixed profile around Z.

Profile (r, z), matching the requested point list with OD/2 as the radius
(OD is treated as a true outer DIAMETER — using it directly as a radius
would double the real-world size):

  (0,        0)              floor centre, bottom
  (outer_r,  0)               outer bottom edge
  (outer_r,  height)          outer top edge (rim)
  (inner_r,  height)          inner top edge (rim) — steps in by wall_thickness_mm
  (inner_r,  wall_thickness)  inner wall bottom (top of the floor slab)
  (0,        wall_thickness)  floor centre, top of floor slab

Revolving this closes into a watertight solid with NO separate end caps:
the two r=0 points become shared "pole" vertices (like a sphere's poles),
and the loop closes angularly since theta=0 meets theta=2*pi. Floor
thickness and wall thickness are both governed by the single
wall_thickness_mm parameter, matching the profile as given.

Threading (not yet implemented — "soon"): the flat top rim (the ring2->ring3
quad band) is where a lid thread would attach — either an external thread
on the outer wall near the rim, or an internal thread cut into the inner
wall near the rim, using the same additive/subtractive thread approach as
hex_bolt.py / hex_nut.py.
"""

import bpy
import bmesh
from math import cos, sin, pi
from bpy.props import FloatProperty, IntProperty

POLE_EPS = 1e-9


# ── Generic revolve helper (reusable for future lathe-style parts) ────────────

def _add_revolve(bm, profile, n_segs):
    """
    Revolve a 2D (r, z) profile 360 deg around the Z axis into bm.
    Profile points with r ~ 0 collapse to a single shared "pole" vertex
    (avoids a degenerate zero-area ring) and are stitched to their
    neighbouring ring with a triangle fan; all other points get a full
    per-angle-step ring connected to its neighbour with quads.
    """
    angles = [2.0 * pi * i / n_segs for i in range(n_segs)]

    point_verts = []
    for r, z in profile:
        if r <= POLE_EPS:
            point_verts.append(bm.verts.new((0.0, 0.0, z)))
        else:
            point_verts.append([bm.verts.new((r * cos(a), r * sin(a), z)) for a in angles])
    bm.verts.index_update()

    for k in range(len(profile) - 1):
        a_r, _ = profile[k]
        b_r, _ = profile[k + 1]
        a_v = point_verts[k]
        b_v = point_verts[k + 1]
        a_pole = a_r <= POLE_EPS
        b_pole = b_r <= POLE_EPS

        if a_pole and b_pole:
            continue
        elif a_pole:
            for i in range(n_segs):
                ni = (i + 1) % n_segs
                bm.faces.new([a_v, b_v[i], b_v[ni]])
        elif b_pole:
            for i in range(n_segs):
                ni = (i + 1) % n_segs
                bm.faces.new([a_v[i], a_v[ni], b_v])
        else:
            for i in range(n_segs):
                ni = (i + 1) % n_segs
                bm.faces.new([a_v[i], a_v[ni], b_v[ni], b_v[i]])


def _to_obj(bm, name, context):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_container(bpy.types.Operator):
    """Parametric open-top container (cup/jar body) via surface of revolution."""
    bl_idname  = "object.container"
    bl_label   = "Container"
    bl_options = {'REGISTER', 'UNDO'}

    outer_diameter_mm: FloatProperty(name="Outer Ø (mm)",       default=40.0, min=2.0, soft_max=300.0)
    wall_thickness_mm: FloatProperty(name="Wall Thickness (mm)", default=2.0,  min=0.2, soft_max=20.0,
                                      description="Governs both the radial wall and the floor slab")
    height_mm:         FloatProperty(name="Height (mm)",        default=60.0, min=1.0, soft_max=400.0)
    n_segs:            IntProperty(  name="Segments",           default=48,   min=8,   soft_max=256)

    def _derived(self):
        outer_r = self.outer_diameter_mm / 2.0
        inner_r = outer_r - self.wall_thickness_mm
        cavity_depth = self.height_mm - self.wall_thickness_mm
        return outer_r, inner_r, cavity_depth

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        outer_r, inner_r, cavity_depth = self._derived()

        col = layout.column(align=True)
        col.prop(self, "outer_diameter_mm")
        col.prop(self, "wall_thickness_mm")
        col.prop(self, "height_mm")
        col.prop(self, "n_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Inner Ø:      %.2f mm" % (inner_r * 2.0))
        box.label(text="Cavity depth: %.2f mm" % cavity_depth)

        if inner_r <= 0:
            layout.label(text="Wall thickness >= outer radius — no cavity", icon='ERROR')
        if cavity_depth <= 0:
            layout.label(text="Height <= wall thickness — floor fills the whole body", icon='ERROR')

    def execute(self, context):
        outer_r, inner_r, cavity_depth = self._derived()

        if inner_r <= 0 or cavity_depth <= 0:
            self.report({'ERROR'}, "Invalid geometry — check wall thickness vs outer diameter and height")
            return {'CANCELLED'}

        w = self.wall_thickness_mm
        h = self.height_mm

        profile = [
            (0.0,     0.0),
            (outer_r, 0.0),
            (outer_r, h),
            (inner_r, h),
            (inner_r, w),
            (0.0,     w),
        ]

        bm = bmesh.new()
        _add_revolve(bm, profile, self.n_segs)
        obj = _to_obj(bm, "Container", context)
        obj.location = context.scene.cursor.location.copy()

        obj["bmech_outer_diameter"] = self.outer_diameter_mm
        obj["bmech_wall_thickness"] = self.wall_thickness_mm
        obj["bmech_height"]         = self.height_mm

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'},
            "Container: Ø%.1f mm outer, %.1f mm wall, %.1f mm tall"
            % (self.outer_diameter_mm, self.wall_thickness_mm, self.height_mm))
        return {'FINISHED'}


# ── Register and run ──────────────────────────────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_container)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_container)
bpy.ops.object.container('INVOKE_DEFAULT')
