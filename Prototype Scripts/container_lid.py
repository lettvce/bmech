"""
Container Lid (Friction-Fit) Prototype
─────────────────────────────────────────
Run with Alt+P. Popup dialog for parameters.

Same surface-of-revolution technique as container.py, but the profile is
the container's shape with Z flipped: a solid disc on top, and a hollow
skirt hanging down that grips over the container's outer wall via a small
radial press-fit clearance (fit_gap_mm) — no threads yet.

Profile (r, z), z=0 at the skirt's open bottom edge, z=height_mm at the
top of the disc:

  (0,          height)              disc top centre (pole)
  (lid_outer_r, height)             disc outer top edge
  (lid_outer_r, 0)                  skirt outer bottom edge (open mouth)
  (lid_inner_r, 0)                  skirt inner bottom edge — bore that
                                     slides over the container's rim
  (lid_inner_r, height-wall)        skirt inner top (meets underside of disc)
  (0,          height-wall)         disc underside centre (pole)

wall_thickness_mm governs both the skirt's radial wall and the disc
thickness, matching container.py's convention. lid_inner_r is derived so
the skirt bore equals (container outer radius + fit_gap_mm) — pick a
Match Target (a container.py output) to read its outer diameter directly
instead of retyping it.
"""

import bpy
import bmesh
from math import cos, sin, pi
from bpy.props import FloatProperty, IntProperty, PointerProperty

POLE_EPS = 1e-9


# ── Generic revolve helper (duplicated from container.py) ─────────────────────

def _add_revolve(bm, profile, n_segs):
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


def _lid_target_poll(_self, obj):
    return obj.type == 'MESH' and "bmech_outer_diameter" in obj.keys()


def _update_lid_target(self, context):
    if self.target is not None and "bmech_outer_diameter" in self.target.keys():
        self.container_outer_diameter_mm = self.target["bmech_outer_diameter"]


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_container_lid(bpy.types.Operator):
    """Friction-fit lid — skirt grips the container's outer wall via a press-fit clearance."""
    bl_idname  = "object.container_lid"
    bl_label   = "Container Lid"
    bl_options = {'REGISTER', 'UNDO'}

    target: PointerProperty(
        name="Match Target", type=bpy.types.Object,
        poll=_lid_target_poll, update=_update_lid_target,
        description="Pick a container.py output to copy its outer diameter from",
    )
    container_outer_diameter_mm: FloatProperty(
        name="Container Outer Ø (mm)", default=40.0, min=2.0, soft_max=300.0,
        description="Must match the container's own outer diameter",
    )
    fit_gap_mm:        FloatProperty(name="Fit Gap (mm)",        default=0.25, min=0.02, soft_max=1.0,
                                      description="Radial press-fit clearance, skirt bore to container outer wall")
    wall_thickness_mm: FloatProperty(name="Wall Thickness (mm)", default=2.0,  min=0.2,  soft_max=20.0,
                                      description="Governs both the skirt's radial wall and the disc thickness")
    height_mm:         FloatProperty(name="Height (mm)",         default=10.0, min=0.5,  soft_max=100.0,
                                      description="Total lid height: skirt depth + disc thickness")
    n_segs:            IntProperty(  name="Segments",            default=48,   min=8,    soft_max=256)

    def _derived(self):
        lid_inner_r  = self.container_outer_diameter_mm / 2.0 + self.fit_gap_mm
        lid_outer_r  = lid_inner_r + self.wall_thickness_mm
        skirt_depth  = self.height_mm - self.wall_thickness_mm
        return lid_inner_r, lid_outer_r, skirt_depth

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=340)

    def draw(self, context):
        layout = self.layout
        lid_inner_r, lid_outer_r, skirt_depth = self._derived()

        layout.prop(self, "target")
        col = layout.column(align=True)
        col.prop(self, "container_outer_diameter_mm")
        col.prop(self, "fit_gap_mm")
        col.prop(self, "wall_thickness_mm")
        col.prop(self, "height_mm")
        col.prop(self, "n_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Skirt bore Ø:  %.2f mm" % (lid_inner_r * 2.0))
        box.label(text="Lid outer Ø:   %.2f mm" % (lid_outer_r * 2.0))
        box.label(text="Skirt depth:   %.2f mm" % skirt_depth)

        if skirt_depth <= 0:
            layout.label(text="Height <= wall thickness — no skirt to grip the container", icon='ERROR')

    def execute(self, context):
        lid_inner_r, lid_outer_r, skirt_depth = self._derived()

        if skirt_depth <= 0:
            self.report({'ERROR'}, "Invalid geometry — height must exceed wall thickness")
            return {'CANCELLED'}

        h = self.height_mm
        w = self.wall_thickness_mm

        profile = [
            (0.0,         h),
            (lid_outer_r, h),
            (lid_outer_r, 0.0),
            (lid_inner_r, 0.0),
            (lid_inner_r, h - w),
            (0.0,         h - w),
        ]

        bm = bmesh.new()
        _add_revolve(bm, profile, self.n_segs)
        obj = _to_obj(bm, "ContainerLid", context)
        obj.location = context.scene.cursor.location.copy()

        obj["bmech_outer_diameter"] = lid_outer_r * 2.0
        obj["bmech_fits_diameter"]  = self.container_outer_diameter_mm

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'},
            "Container lid: fits Ø%.1f mm container, %.2f mm gap, %.1f mm tall"
            % (self.container_outer_diameter_mm, self.fit_gap_mm, h))
        return {'FINISHED'}


# ── Register and run ──────────────────────────────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_container_lid)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_container_lid)
bpy.ops.object.container_lid('INVOKE_DEFAULT')
