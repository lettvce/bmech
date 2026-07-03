# hairspring/hairspring.py
# Blender 5.1 Add-on: Flat Archimedean Spiral Ribbon Mesh Generator
# Suitable for FDM 3D printing. All values in mm.

import bpy
import math
from mathutils import Vector
from bpy.props import FloatProperty, IntProperty, EnumProperty
from bpy.types import Operator


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def generate_centerline_points(r_inner_mm, gap_mm, turns, resolution):
    """Return list of (x, y, z) tuples for the spiral centerline.

    r(θ) = r_inner + (gap / 2π) × θ,  θ ∈ [0, 2π·turns]
    """
    N = max(int(round(resolution * turns)) + 1, 2)
    two_pi_turns = 2.0 * math.pi * turns
    gap_over_2pi = gap_mm / (2.0 * math.pi)
    points = []
    for i in range(N):
        t     = i / (N - 1)
        theta = t * two_pi_turns
        r     = r_inner_mm + gap_over_2pi * theta
        points.append((r * math.cos(theta), r * math.sin(theta), 0.0))
    return points


def build_manual_ribbon(points, strip_thickness_mm, strip_width_mm):
    """Build a rectangular cross-section ribbon along the centerline.

    Returns (verts, faces) ready for mesh.from_pydata().
    """
    half_t = strip_thickness_mm / 2.0
    half_w = strip_width_mm     / 2.0
    Z_hat  = Vector((0.0, 0.0, 1.0))
    N      = len(points)
    verts  = []
    faces  = []

    tangents = []
    for i in range(N):
        if i == 0:
            T = Vector(points[1])     - Vector(points[0])
        elif i == N - 1:
            T = Vector(points[N - 1]) - Vector(points[N - 2])
        else:
            T = Vector(points[i + 1]) - Vector(points[i - 1])
        length = T.length
        tangents.append(T / length if length > 1e-12 else Vector((1.0, 0.0, 0.0)))

    for i in range(N):
        P = Vector(points[i])
        B = tangents[i].cross(Z_hat)
        b_len = B.length
        B = B / b_len if b_len > 1e-12 else Vector((1.0, 0.0, 0.0))

        v0 = P + half_t * B + half_w * Z_hat
        v1 = P - half_t * B + half_w * Z_hat
        v2 = P - half_t * B - half_w * Z_hat
        v3 = P + half_t * B - half_w * Z_hat
        verts.extend([v0[:], v1[:], v2[:], v3[:]])

    for i in range(N - 1):
        a0, a1, a2, a3 = 4*i,       4*i+1,     4*i+2,     4*i+3
        b0, b1, b2, b3 = 4*(i+1),   4*(i+1)+1, 4*(i+1)+2, 4*(i+1)+3
        faces.append((a0, b0, b3, a3))  # outer
        faces.append((b1, a1, a2, b2))  # inner
        faces.append((a0, a1, b1, b0))  # top
        faces.append((b2, a2, a3, b3))  # bottom

    faces.append((0, 3, 2, 1))                                          # start cap
    base = 4 * (N - 1)
    faces.append((base, base + 1, base + 2, base + 3))                  # end cap

    return verts, faces


# ---------------------------------------------------------------------------
# Operator
# ---------------------------------------------------------------------------

class OBJECT_OT_add_hairspring(Operator):
    """Add a flat Archimedean spiral ribbon mesh (hairspring)"""
    bl_idname  = "object.add_hairspring"
    bl_label   = "Add Hairspring"
    bl_options = {'REGISTER', 'UNDO'}

    input_mode: EnumProperty(
        name="Input Mode",
        items=[
            ('MODE_A', "Inner / Turns / Gap",   "Specify inner radius, number of turns, and coil gap"),
            ('MODE_B', "Inner / Outer / Turns",  "Specify inner radius, outer radius, and number of turns"),
        ],
        default='MODE_A',
    )
    r_inner: FloatProperty(
        name="Inner Radius (mm)", default=10.0, min=0.1, soft_max=500.0,
    )
    turns: FloatProperty(
        name="Turns", default=5.0, min=0.5, soft_max=50.0,
    )
    gap: FloatProperty(
        name="Coil Gap (mm)", default=2.0, min=0.1, soft_max=50.0,
        description="Radial gap between adjacent coil passes. Mode A only.",
    )
    r_outer: FloatProperty(
        name="Outer Radius (mm)", default=20.0, min=0.2, soft_max=500.0,
        description="Radius of the outermost coil. Mode B only.",
    )
    strip_width: FloatProperty(
        name="Width (mm)", default=1.0, min=0.4, soft_max=20.0,
        description="Strip width — ribbon dimension in Z. FDM min: 0.4 mm.",
    )
    strip_thickness: FloatProperty(
        name="Thickness (mm)", default=0.4, min=0.2, soft_max=10.0,
        description="Strip thickness — ribbon dimension radially. FDM min: 0.2 mm.",
    )
    resolution: IntProperty(
        name="Resolution (pts/turn)", default=128, min=8, soft_max=512,
        description="Polyline sample points per full turn.",
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "input_mode")

        spiral = layout.box()
        spiral.label(text="Spiral", icon='CURVE_DATA')
        spiral.prop(self, "r_inner")
        spiral.prop(self, "turns")

        if self.input_mode == 'MODE_A':
            spiral.prop(self, "gap")
            spiral.label(text="Outer Radius: %.2f mm" % (self.r_inner + self.gap * self.turns))
        else:
            spiral.prop(self, "r_outer")
            derived_gap = (
                (self.r_outer - self.r_inner) / self.turns
                if self.turns > 0 and self.r_outer > self.r_inner else 0.0
            )
            spiral.label(text="Coil Gap: %.2f mm" % derived_gap)
            if self.r_outer <= self.r_inner:
                spiral.label(text="Outer radius must exceed inner radius", icon='ERROR')
            elif derived_gap < 0.1:
                spiral.label(text="Derived gap below 0.1 mm — increase outer radius or reduce turns", icon='ERROR')

        strip = layout.box()
        strip.label(text="Strip", icon='MOD_SOLIDIFY')
        strip.prop(self, "strip_width")
        strip.prop(self, "strip_thickness")

        mesh = layout.box()
        mesh.label(text="Mesh", icon='MESH_DATA')
        mesh.prop(self, "resolution")

    def execute(self, context):
        r_inner_mm = self.r_inner
        turns      = self.turns

        if self.input_mode == 'MODE_A':
            gap_mm = self.gap
        else:
            if self.r_outer <= r_inner_mm:
                return {'CANCELLED'}
            gap_mm = (self.r_outer - r_inner_mm) / turns
            gap_mm = max(gap_mm, 0.1)

        points      = generate_centerline_points(r_inner_mm, gap_mm, turns, self.resolution)
        verts, faces = build_manual_ribbon(points, self.strip_thickness, self.strip_width)

        mesh = bpy.data.meshes.new("HairspringMesh")
        obj  = bpy.data.objects.new("Hairspring", mesh)
        context.collection.objects.link(obj)
        obj.location = context.scene.cursor.location.copy()
        mesh.from_pydata(verts, [], faces)
        mesh.update()

        for o in context.selected_objects:
            o.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

def register():
    bpy.utils.register_class(OBJECT_OT_add_hairspring)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_add_hairspring)


if __name__ == "__main__":
    register()
