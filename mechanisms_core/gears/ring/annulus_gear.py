"""
Annulus (Internal / Ring) Gear Generator

An annulus gear has involute teeth on its INNER surface. A matching external
pinion runs inside the ring. Both gears must share the same module and
pressure angle.

Geometry:
  pitch_r      = module × tooth_count / 2
  tip_r        = pitch_r − module          (tooth tips point inward)
  root_r_inner = pitch_r + 1.25 × module   (tooth base meets ring body)
  outer_r      = root_r_inner + ring_wall_mm

Meshing rules:
  • Same module, same pressure angle as the mating pinion
  • Annulus tooth count must be > pinion tooth count (typical ratio 3:1 – 5:1)
  • Avoid N_annulus − N_pinion < 12 — interference risk with standard proportions

Build method (no Solidify — boolean only, epsilon = 0.001 mm):
  1. Solid outer cylinder at outer_r, height = width_mm
  2. Boolean DIFFERENCE with tooth-space cutter (extruded from the cutter profile)
  The cutter profile is identical to _build_gear_profile but with
  ADDENDUM_COEFF and DEDENDUM_COEFF swapped, so the "teeth" of the cutter
  fill the tooth SPACES of the annulus, plus the inner bore below tip_r.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, sqrt, radians, atan2
from .. import gear_matching

INVOLUTE_POINTS = 15
ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
BOOL_EPSILON    = 0.001


def _involute_pt(base_r, t):
    return (base_r * (cos(t) + t * sin(t)),
            base_r * (sin(t) - t * cos(t)))


def _involute_t_at_r(base_r, r):
    ratio = r / base_r
    return 0.0 if ratio < 1.0 else sqrt(ratio * ratio - 1.0)


def _rot(x, y, a):
    c, s = cos(a), sin(a)
    return (x * c - y * s, x * s + y * c)


def _build_annulus_cutter_profile(module, tooth_count, pa_deg):
    """
    2D profile of the void inside an annulus gear bore.

    Identical to the external gear profile builder but with add/ded swapped:
      ded_r → pitch_r − ADDENDUM × m   (inner bore, at annulus tooth tips)
      add_r → pitch_r + DEDENDUM × m   (outer reach, at annulus tooth roots)

    The profile traces a gear-shaped star (CCW from +Z). Boolean-subtracting
    its extruded solid from an outer cylinder produces the annulus ring body.
    """
    pa_rad  = radians(pa_deg)
    pitch_r = module * tooth_count / 2.0
    base_r  = pitch_r * cos(pa_rad)
    ded_r   = pitch_r - ADDENDUM_COEFF * module   # annulus tip_r  (inner bore)
    add_r   = pitch_r + DEDENDUM_COEFF * module   # annulus root_r (outer reach)
    half_tooth_ang = pi / (2.0 * tooth_count)
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)

    raw = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
           for i in range(INVOLUTE_POINTS)]

    px, py = _involute_pt(base_r, t_pitch)
    rot    = half_tooth_ang + atan2(py, px)
    right  = [_rot(x, -y, rot) for x, y in raw]

    if base_r > ded_r:
        ra    = atan2(right[0][1], right[0][0])
        right = [(ded_r * cos(ra), ded_r * sin(ra))] + right

    left      = [(x, -y) for x, y in right]
    tooth_pts = left + [(add_r, 0.0)] + right[::-1]

    root_pt      = _involute_pt(base_r, t_start)
    root_rot     = _rot(root_pt[0], -root_pt[1], rot)
    root_r_local = atan2(root_rot[1], root_rot[0])

    SPACE_PTS = 4
    profile   = []
    for i in range(tooth_count):
        ta = i * pitch_arc
        for x, y in tooth_pts:
            profile.append(_rot(x, y, ta))
        a0 = ta + root_r_local
        a1 = ta + pitch_arc - root_r_local
        for j in range(1, SPACE_PTS + 1):
            a = a0 + (a1 - a0) * j / (SPACE_PTS + 1)
            profile.append((ded_r * cos(a), ded_r * sin(a)))

    return profile


def _make_solid_from_profile(context, profile_pts, z_bot, z_top, mesh_name, obj_name):
    """Extrude a 2D closed polygon into a capped prism solid."""
    bm = bmesh.new()
    n  = len(profile_pts)

    bot   = [bm.verts.new((x, y, z_bot)) for x, y in profile_pts]
    top   = [bm.verts.new((x, y, z_top)) for x, y in profile_pts]
    c_bot = bm.verts.new((0.0, 0.0, z_bot))
    c_top = bm.verts.new((0.0, 0.0, z_top))
    bm.verts.index_update()

    # Side walls
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

    # Caps: center-fan triangulation — flat, non-overlapping tris guaranteed for any
    # star-shaped profile; avoids CGAL EXACT solver rejecting self-intersecting tris
    # that triangle_fill can produce on deeply concave star polygons.
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_bot, bot[ni], bot[i]])    # bottom, normal −Z
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_top, top[i], top[ni]])    # top, normal +Z

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(obj_name, me)
    context.collection.objects.link(obj)
    return obj


def _make_cylinder(context, r, z_bot, z_top, n_segs, mesh_name, obj_name):
    """Solid cylinder."""
    bm     = bmesh.new()
    angles = [2.0 * pi * i / n_segs for i in range(n_segs)]

    bot = [bm.verts.new((r * cos(a), r * sin(a), z_bot)) for a in angles]
    top = [bm.verts.new((r * cos(a), r * sin(a), z_top)) for a in angles]
    bm.verts.index_update()

    bm.faces.new(list(reversed(bot)))
    bm.faces.new(top)
    for i in range(n_segs):
        ni = (i + 1) % n_segs
        bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(obj_name, me)
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_annulus_gear(bpy.types.Operator):
    """Annulus (internal / ring) gear — involute teeth on the inner bore."""
    bl_idname  = "object.annulus_gear"
    bl_label   = "Annulus Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_module_pa(self, context.window_manager.bmech_gear_target)

    tooth_count:        IntProperty(  name="Tooth Count",          default=40,   min=8,    soft_max=200,
                                      description="Internal gear tooth count; must exceed the mating pinion count")
    module:             FloatProperty(name="Module (mm)",           default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Transverse module — must match the mating pinion")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)",   default=20.0, min=10.0, max=45.0)
    width_mm:           FloatProperty(name="Width (mm)",            default=10.0, min=1.0,  soft_max=80.0)
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",        default=5.0,  min=0.5,  soft_max=30.0,
                                      description="Radial wall thickness beyond the tooth root")
    outer_segs:         IntProperty(  name="Outer Segments",       default=64,   min=16,   soft_max=256,
                                      description="Facets on the outer cylindrical surface")

    def _derived(self):
        pitch_r      = self.module * self.tooth_count / 2.0
        tip_r        = pitch_r - ADDENDUM_COEFF * self.module
        root_r_inner = pitch_r + DEDENDUM_COEFF * self.module
        outer_r      = root_r_inner + self.ring_wall_mm

        # Pointed-tooth / self-intersection limit: past a tooth-count-dependent
        # pressure angle, the cutter's tooth flanks cross before reaching the
        # tip, producing a self-intersecting profile the boolean solver can't
        # process. execute() clamps pressure_angle_deg to this automatically.
        pa_max = gear_matching.max_pressure_angle_deg(self.tooth_count, DEDENDUM_COEFF)

        return pitch_r, tip_r, root_r_inner, outer_r, pa_max

    def draw(self, context):
        layout = self.layout
        pitch_r, tip_r, root_r_inner, outer_r, pa_max = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "width_mm")
        col.prop(self, "ring_wall_mm")
        col.prop(self, "outer_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Pitch Ø:     %.2f mm" % (pitch_r * 2))
        box.label(text="Tip Ø:       %.2f mm" % (tip_r * 2))
        box.label(text="Root Ø:      %.2f mm" % (root_r_inner * 2))
        box.label(text="Outer Ø:     %.2f mm" % (outer_r * 2))
        box.label(text="Ring wall:   %.2f mm" % self.ring_wall_mm)
        box.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))

        if tip_r <= 0:
            layout.label(text="Module too large — tip radius ≤ 0", icon='ERROR')

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, DEDENDUM_COEFF))
        pitch_r, tip_r, root_r_inner, outer_r, pa_max = self._derived()

        if tip_r <= 0:
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()

        # 1. Solid outer cylinder (the ring body blank)
        body = _make_cylinder(
            context, outer_r,
            0.0, self.width_mm,
            self.outer_segs,
            "AnnulusGearMesh", "AnnulusGear"
        )
        body.location = cursor

        # 2. Tooth-space cutter — extruded annulus void profile
        #    Slightly taller than body to guarantee clean boolean faces
        cutter_pts = _build_annulus_cutter_profile(
            self.module, self.tooth_count, self.pressure_angle_deg
        )
        cutter = _make_solid_from_profile(
            context, cutter_pts,
            -BOOL_EPSILON, self.width_mm + BOOL_EPSILON,
            "__AnnulusCutMesh", "__AnnulusCut"
        )
        cutter.location = cursor

        # 3. Boolean DIFFERENCE
        bpy.ops.object.select_all(action='DESELECT')
        body.select_set(True)
        context.view_layer.objects.active = body

        mod           = body.modifiers.new("ToothBore", 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object    = cutter
        mod.solver    = 'EXACT'

        bpy.ops.object.modifier_apply(modifier="ToothBore")

        bpy.data.objects.remove(cutter, do_unlink=True)

        body.select_set(True)
        context.view_layer.objects.active = body

        gear_matching.stamp_gear(body, "annulus", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count)

        self.report({'INFO'},
            "Annulus: %d teeth, module %.1f, pitch Ø %.1f mm, outer Ø %.1f mm"
            % (self.tooth_count, self.module, pitch_r * 2, outer_r * 2))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_annulus_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
