"""
Print-in-Place Captive Telescopic Prototype
─────────────────────────────────────────────
Run with Alt+P. Popup dialog for parameter editing.

Two concentric tubes printed already assembled. The inner tube slides in/out
but cannot be fully withdrawn — a lip on the inner tube catches in a groove on
the outer tube's inner wall.

Profile approach (same as Blender's Screw modifier):
  Build a 2D closed cross-section in the XZ plane (X = radius, Z = height),
  then revolve it 360° around Z with bmesh.ops.spin.

  The profile includes:
    • Outer wall (straight, X = outer_r)
    • Top cap edge (inner_r → outer_r at Z = length)
    • Bottom cap edge (outer_r → inner_r at Z = 0)
    • Inner wall with the groove/lip feature cut in

Captive feature geometry (angled walls for FDM self-support):
  Groove (outer tube inner wall) and lip (inner tube outer wall) are symmetric
  trapezoidal features. lip_angle controls the wall taper:
    0°   → vertical step (easiest to design, hardest to print cleanly)
    30°  → good FDM compromise
    45°  → fully self-supporting overhang

Clearance:
  The radial gap between outer tube inner wall and inner tube outer wall.
  Also applied between lip and groove walls. Tune per printer — start at 0.3mm.

Assembly position:
  Inner tube is placed with lip below groove — it can extend outward until
  the lip catches the groove's upper wall, and retract fully inward.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, tan, radians


SPIN_STEPS = 64   # revolution resolution


# ── Profile builder ───────────────────────────────────────────────────────────

def outer_tube_profile(outer_r, inner_r, length, groove_z, lip_h, lip_d, lip_ang_rad):
    """
    2D cross-section for the outer tube (XZ plane).
    The groove is a trapezoid recessed into the inner wall.
    Returns list of (x, z) CCW.
    """
    slant = lip_d * tan(lip_ang_rad)   # axial shift per unit of radial depth

    pts = []
    # Bottom cap: outer → inner at Z=0
    pts.append((outer_r, 0.0))
    pts.append((inner_r, 0.0))

    # Inner wall up to groove lower approach
    pts.append((inner_r, groove_z - lip_h / 2.0 - slant))

    # Groove lower angled wall (inward)
    pts.append((inner_r - lip_d, groove_z - lip_h / 2.0))

    # Groove bottom flat (zero width if lip_h covers it fully)
    pts.append((inner_r - lip_d, groove_z + lip_h / 2.0))

    # Groove upper angled wall (back outward)
    pts.append((inner_r, groove_z + lip_h / 2.0 + slant))

    # Inner wall up to top
    pts.append((inner_r, length))

    # Top cap: inner → outer at Z=length
    pts.append((outer_r, length))

    return pts


def inner_tube_profile(lip_outer_r, lip_inner_r, length, lip_z, lip_h, lip_d, lip_ang_rad, clearance):
    """
    2D cross-section for the inner tube (XZ plane).
    The lip is a trapezoid protruding from the outer wall.
    outer_r = lip_outer_r - clearance (printed gap from outer tube inner wall)
    """
    slant   = lip_d * tan(lip_ang_rad)
    outer_r = lip_outer_r            # outer surface of inner tube (= outer tube inner_r - clearance)
    inner_r = lip_inner_r            # inner bore of inner tube

    pts = []
    # Bottom cap: outer → inner at Z=0
    pts.append((outer_r, 0.0))
    pts.append((inner_r, 0.0))

    # Inner wall (bore) straight up
    pts.append((inner_r, length))

    # Top cap: inner → outer at Z=length
    pts.append((outer_r, length))

    # Outer wall down from top to lip upper approach
    pts.append((outer_r, lip_z + lip_h / 2.0 + slant))

    # Lip upper angled wall (outward)
    pts.append((outer_r + lip_d, lip_z + lip_h / 2.0))

    # Lip flat face
    pts.append((outer_r + lip_d, lip_z - lip_h / 2.0))

    # Lip lower angled wall (back inward)
    pts.append((outer_r, lip_z - lip_h / 2.0 - slant))

    # Outer wall down to bottom
    # (bottom cap already added at start — profile is now closed)

    return pts


def profile_to_mesh(context, pts, name, steps=SPIN_STEPS):
    """Revolve a 2D XZ profile 360° around Z to produce a solid of revolution."""
    bm = bmesh.new()

    verts = [bm.verts.new((x, 0.0, z)) for x, z in pts]
    bm.verts.index_update()

    n = len(verts)
    edges = [bm.edges.new((verts[i], verts[(i + 1) % n])) for i in range(n)]
    bm.edges.index_update()

    bmesh.ops.spin(
        bm,
        geom=edges + verts,
        axis=(0.0, 0.0, 1.0),
        cent=(0.0, 0.0, 0.0),
        angle=2.0 * pi,
        steps=steps,
        use_duplicate=False,
    )

    bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-5)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(name, me)
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_captive_telescopic(bpy.types.Operator):
    """Print-in-place captive telescopic — inner tube slides but cannot be withdrawn."""
    bl_idname  = "object.captive_telescopic"
    bl_label   = "Captive Telescopic"
    bl_options = {'REGISTER', 'UNDO'}

    outer_od:       FloatProperty(name="Outer OD (mm)",        default=30.0, min=4.0,  soft_max=200.0,
                                   description="Outside diameter of the outer tube")
    wall_thickness: FloatProperty(name="Wall Thickness (mm)",  default=2.5,  min=0.5,  soft_max=20.0,
                                   description="Wall thickness — applied to both tubes")
    clearance:      FloatProperty(name="Clearance (mm)",       default=0.3,  min=0.05, soft_max=2.0,
                                   description="Radial gap between tubes — tune per printer")
    tube_length:    FloatProperty(name="Tube Length (mm)",     default=40.0, min=5.0,  soft_max=300.0,
                                   description="Axial length of each tube section")
    lip_height:     FloatProperty(name="Lip Height (mm)",      default=2.0,  min=0.5,  soft_max=10.0,
                                   description="Axial height of the captive lip / groove")
    lip_depth:      FloatProperty(name="Lip Depth (mm)",       default=1.2,  min=0.1,  soft_max=10.0,
                                   description="Radial protrusion of lip / depth of groove")
    lip_angle_deg:  FloatProperty(name="Lip Angle (°)",        default=30.0, min=0.0,  max=60.0,
                                   description="Wall angle: 0=vertical step, 30=good FDM, 45=self-supporting")
    lip_offset:     FloatProperty(name="Lip Offset (mm)",      default=5.0,  min=1.0,  soft_max=50.0,
                                   description="Distance from open end of each tube to lip center")

    def _derived(self):
        outer_r       = self.outer_od / 2.0
        outer_inner_r = outer_r - self.wall_thickness          # inner wall of outer tube
        inner_outer_r = outer_inner_r - self.clearance         # outer wall of inner tube
        inner_inner_r = inner_outer_r - self.wall_thickness    # bore of inner tube
        lip_ang       = radians(self.lip_angle_deg)
        slant         = self.lip_depth * tan(lip_ang)

        # Groove sits near the top of the outer tube (open end)
        groove_z = self.tube_length - self.lip_offset

        # Lip on inner tube sits near the top (open end) — starts BELOW groove when assembled
        # so inner tube can push in. When pulled out, lip catches groove upper wall.
        lip_z = self.lip_offset

        max_travel = groove_z - lip_z - self.lip_height - self.clearance

        return (outer_r, outer_inner_r, inner_outer_r, inner_inner_r,
                lip_ang, slant, groove_z, lip_z, max_travel)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        (outer_r, outer_inner_r, inner_outer_r, inner_inner_r,
         lip_ang, slant, groove_z, lip_z, max_travel) = self._derived()

        col = layout.column(align=True)
        col.prop(self, "outer_od")
        col.prop(self, "wall_thickness")
        col.prop(self, "clearance")
        col.prop(self, "tube_length")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "lip_height")
        col.prop(self, "lip_depth")
        col.prop(self, "lip_angle_deg")
        col.prop(self, "lip_offset")

        layout.separator()
        box = layout.box()
        box.label(text="Outer tube inner Ø: %.2f mm" % (outer_inner_r * 2))
        box.label(text="Inner tube outer Ø: %.2f mm" % (inner_outer_r * 2))
        box.label(text="Inner tube bore Ø:  %.2f mm" % (inner_inner_r * 2))
        box.label(text="Max travel: %.2f mm"          % max(max_travel, 0.0))

        if inner_inner_r <= 0:
            layout.label(text="Wall thickness too large — inner bore ≤ 0", icon='ERROR')
        if max_travel <= 0:
            layout.label(text="Lip offset too large — no travel room", icon='ERROR')
        if self.lip_depth >= outer_inner_r - inner_outer_r + self.clearance:
            layout.label(text="Lip too deep — will collide with bore", icon='ERROR')
        if slant * 2 >= self.lip_height:
            layout.label(text="Lip angle too steep for lip height — walls cross", icon='ERROR')

    def execute(self, context):
        (outer_r, outer_inner_r, inner_outer_r, inner_inner_r,
         lip_ang, slant, groove_z, lip_z, max_travel) = self._derived()

        if inner_inner_r <= 0 or max_travel <= 0:
            return {'CANCELLED'}
        if slant * 2 >= self.lip_height:
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()

        # Outer tube
        o_pts = outer_tube_profile(
            outer_r, outer_inner_r, self.tube_length,
            groove_z, self.lip_height, self.lip_depth, lip_ang,
        )
        outer_obj = profile_to_mesh(context, o_pts, "TelescopicOuter")
        outer_obj.location = cursor

        # Inner tube — positioned so lip starts below groove (assembled, retracted)
        # Inner tube lip_z is measured from ITS OWN bottom.
        # Place inner tube so its bottom sits at cursor Z, extending upward.
        i_pts = inner_tube_profile(
            inner_outer_r, inner_inner_r, self.tube_length,
            lip_z, self.lip_height, self.lip_depth, lip_ang, self.clearance,
        )
        inner_obj = profile_to_mesh(context, i_pts, "TelescopicInner")
        inner_obj.location = cursor

        bpy.ops.object.select_all(action='DESELECT')
        outer_obj.select_set(True)
        inner_obj.select_set(True)
        context.view_layer.objects.active = inner_obj

        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_captive_telescopic)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_captive_telescopic)
bpy.ops.object.captive_telescopic('INVOKE_DEFAULT')
