# serpentine_spring/serpentine_spring.py
# Blender 5.1 Add-on: FDM Serpentine Spring Generator
# All values in mm.
#
# Geometry model: centerline path (pad → leg → bend → … → leg → pad) offset
# by ±strip_thickness/2 to produce synchronized outer/inner quad strips.
# Solidify modifier handles out-of-plane (Z) thickness.
#
# Input modes:
#   PITCH_MODE  — user sets Module Count + Pitch; Length is computed exactly
#   LENGTH_MODE — user sets Module Count + Length; Pitch is computed exactly
# Module Count is always a direct integer input in both modes — it can't be
# derived/floored from a length+pitch combination without producing a mismatch
# between the requested and actual spring length.
#
# Termination convention:
#   Odd  module_count → U-termination (both open ends on the same Y side)
#   Even module_count → S-termination (open ends on opposite Y sides)

import bpy
import bmesh
from bpy.props import EnumProperty, FloatProperty, IntProperty
from bpy.types import Operator
import math


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _arc_pts(cx, cy, radius, start_angle, end_angle, n_segs, skip_first=False):
    pts = []
    for i in range(n_segs + 1):
        if i == 0 and skip_first:
            continue
        t = i / n_segs
        a = start_angle + t * (end_angle - start_angle)
        pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return pts


def _n_bend_segs(bend_radius):
    return max(8, round(math.pi * bend_radius / 0.2))


def build_spring_centerline(pitch, module_count, leg_length, bend_radius, n_bend, strip_thickness):
    """Centerline path: flat_end → leg → bend → … → leg → flat_end.

    Y anchors are inset by strip_thickness/2 so the outer offset lands exactly
    on spring_width after offset_centerline runs.
    """
    half_t   = strip_thickness / 2
    y_bc_bot = half_t + bend_radius
    y_bc_top = y_bc_bot + leg_length

    def x_leg(i):
        return i * pitch

    path = [(x_leg(0), y_bc_bot)]

    for i in range(module_count + 1):
        xi      = x_leg(i)
        is_even = (i % 2 == 0)
        if is_even:
            path.append((xi, y_bc_top))
            if i < module_count:
                path += _arc_pts(xi + bend_radius, y_bc_top, bend_radius,
                                  math.pi, 0.0, n_bend, skip_first=True)
        else:
            path.append((xi, y_bc_bot))
            if i < module_count:
                path += _arc_pts(xi + bend_radius, y_bc_bot, bend_radius,
                                  math.pi, 2 * math.pi, n_bend, skip_first=True)

    return path


def offset_centerline(path, half_height):
    """Offset path ±half_height along local perpendicular at each point.

    Returns (outer_verts, inner_verts), index-aligned with path.
    At sharp corners the averaged-normal offset produces a slight chamfer
    rather than a sharp miter — visible but under 1 mm at typical strip sizes.
    """
    M = len(path)
    if M < 2:
        raise ValueError("Centerline path has only %d point(s) — need at least 2." % M)

    outer, inner = [], []
    for i in range(M):
        px, py = path[i]

        if i == 0:
            dx = path[1][0] - path[0][0]
            dy = path[1][1] - path[0][1]
        elif i == M - 1:
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
        else:
            dx = path[i + 1][0] - path[i - 1][0]
            dy = path[i + 1][1] - path[i - 1][1]

        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            raise ValueError(
                "Degenerate centerline direction at path index %d (point=%s). "
                "Check for a zero-length leg or degenerate bend radius." % (i, path[i])
            )
        dx /= norm
        dy /= norm
        nx, ny = -dy, dx  # leftward perpendicular

        outer.append((px + nx * half_height, py + ny * half_height))
        inner.append((px - nx * half_height, py - ny * half_height))

    return outer, inner


def build_spring_quadstrip(spring_width, pitch, module_count, strip_thickness, leg_length, bend_radius):
    """Build the spring as a quad strip between outer/inner offset loops.

    Returns (verts_3d, faces):
      verts_3d — outer loop then inner loop, all at z=0
      faces    — quads [o_i, i_i, i_{i+1}, o_{i+1}], CCW-wound
    """
    n_bend = _n_bend_segs(bend_radius)
    path   = build_spring_centerline(pitch, module_count, leg_length, bend_radius, n_bend, strip_thickness)
    outer, inner = offset_centerline(path, strip_thickness / 2)

    M       = len(path)
    n_outer = M
    verts_3d = [(x, y, 0.0) for x, y in outer] + [(x, y, 0.0) for x, y in inner]

    faces = []
    for i in range(M - 1):
        o0, o1 = i, i + 1
        i0, i1 = n_outer + i, n_outer + i + 1
        faces.append([o0, i0, i1, o1])

    return verts_3d, faces


# ─────────────────────────────────────────────────────────────────────────────
# Operator
# ─────────────────────────────────────────────────────────────────────────────

class OBJECT_OT_add_serpentine_spring(Operator):
    """Add a flat serpentine spring mesh to the scene"""
    bl_idname  = "object.add_serpentine_spring"
    bl_label   = "Add Serpentine Spring"
    bl_options = {'REGISTER', 'UNDO'}

    input_mode: EnumProperty(
        name="Input Mode",
        items=[
            ('PITCH_MODE',  "Pitch Mode",  "Set Module Count + Pitch; Length is computed"),
            ('LENGTH_MODE', "Length Mode", "Set Module Count + Length; Pitch is computed"),
        ],
        default='PITCH_MODE',
    )
    module_count: IntProperty(
        name="Module Count",
        description="Number of U-turn modules. Always a direct input — never derived.",
        default=5, min=1,
    )
    spring_width: FloatProperty(
        name="Width (mm)",
        description="Full Y-axis extent of the spring (center-to-center of bend centers).",
        default=20.0, min=1.0,
    )
    pitch: FloatProperty(
        name="Pitch (mm)",
        description="Center-to-center distance between adjacent leg centerlines. Pitch Mode only.",
        default=4.0, min=0.8,
    )
    spring_length: FloatProperty(
        name="Length (mm)",
        description="Total spring extent along the compression (X) axis. Length Mode only.",
        default=40.0, min=1.0,
    )
    strip_height: FloatProperty(
        name="Strip Height (mm)",
        description="Out-of-plane (Z) dimension — drives the Solidify modifier.",
        default=2.0, min=0.4,
    )
    strip_thickness: FloatProperty(
        name="Strip Thickness (mm)",
        description="In-plane cross-section in X — drives the outer/inner profile offset.",
        default=0.8, min=0.0,
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "input_mode", expand=True)

        dims = layout.box()
        dims.label(text="Dimensions")
        col = dims.column(align=True)
        col.prop(self, "module_count")
        col.prop(self, "spring_width")
        col.separator()

        if self.input_mode == 'PITCH_MODE':
            col.prop(self, "pitch")
            true_length = self.module_count * self.pitch + self.strip_thickness
            col.label(text="Length: %.2f mm  (computed)" % true_length)
        else:
            col.prop(self, "spring_length")
            if self.module_count > 0:
                cp = (self.spring_length - self.strip_thickness) / self.module_count
                col.label(text="Pitch: %.3f mm  (computed)" % cp)

        strip = layout.box()
        strip.label(text="Strip")
        col2 = strip.column(align=True)
        col2.prop(self, "strip_height")
        col2.prop(self, "strip_thickness")

    def execute(self, context):
        # ── Resolve pitch / length ────────────────────────────────────────────
        if self.input_mode == 'PITCH_MODE':
            pitch         = self.pitch
            spring_length = self.module_count * pitch + self.strip_thickness
        else:
            spring_length     = self.spring_length
            centerline_length = spring_length - self.strip_thickness
            pitch = centerline_length / self.module_count if self.module_count > 0 else 0.0

        bend_radius = pitch / 2
        leg_length  = self.spring_width - self.strip_thickness - 2 * bend_radius

        # ── Validate ──────────────────────────────────────────────────────────
        if leg_length <= 0:
            self.report({'ERROR'},
                "Width is too narrow for the current pitch and strip thickness. "
                "Increase Width, decrease Pitch, or decrease Strip Thickness.")
            return {'CANCELLED'}
        if pitch < self.strip_thickness:
            self.report({'ERROR'}, "Pitch must be greater than strip thickness to avoid self-intersection.")
            return {'CANCELLED'}

        if self.module_count % 2 == 0:
            self.report({'WARNING'},
                "module_count=%d is even → S-termination (open ends on opposite sides). "
                "Use an odd module count for U-termination." % self.module_count)

        # ── Build geometry ────────────────────────────────────────────────────
        try:
            verts_3d, faces = build_spring_quadstrip(
                spring_width=self.spring_width,
                pitch=pitch,
                module_count=self.module_count,
                strip_thickness=self.strip_thickness,
                leg_length=leg_length,
                bend_radius=bend_radius,
            )
        except Exception as ex:
            self.report({'ERROR'}, "Geometry construction failed: %s" % ex)
            return {'CANCELLED'}

        if len(verts_3d) < 3:
            self.report({'ERROR'}, "Too few vertices generated — check parameters.")
            return {'CANCELLED'}

        # ── Mesh ──────────────────────────────────────────────────────────────
        mesh = bpy.data.meshes.new("SerpentineSpring")
        mesh.from_pydata(verts_3d, [], faces)
        mesh.update()

        bm = bmesh.new()
        bm.from_mesh(mesh)
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()

        if mesh.validate(verbose=False):
            print("SerpentineSpring: mesh.validate() repaired some issues.")

        # ── Object ────────────────────────────────────────────────────────────
        obj = bpy.data.objects.new("SerpentineSpring", mesh)
        context.collection.objects.link(obj)

        bpy.ops.object.select_all(action='DESELECT')
        context.view_layer.update()
        obj.select_set(True)
        context.view_layer.objects.active = obj

        mod             = obj.modifiers.new(name="SpringThickness", type='SOLIDIFY')
        mod.thickness   = self.strip_height
        mod.offset      = 0.0
        mod.use_even_offset = True
        mod.use_rim     = True
        mod.use_rim_only = False

        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY', center='BOUNDS')

        self.report({'INFO'},
            "Spring generated: %d modules, pitch=%.2f mm, bend_radius=%.2f mm"
            % (self.module_count, pitch, bend_radius))
        return {'FINISHED'}


# ─────────────────────────────────────────────────────────────────────────────
# Registration
# ─────────────────────────────────────────────────────────────────────────────

def register():
    bpy.utils.register_class(OBJECT_OT_add_serpentine_spring)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_add_serpentine_spring)


if __name__ == "__main__":
    register()
