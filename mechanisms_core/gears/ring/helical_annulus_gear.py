"""
Helical Annulus (Internal Ring) Gear Generator

An annulus gear has involute teeth on its inner bore. A helical annulus adds
a twist along the Z axis — the same math as a helical pinion but inverted.

Geometry:
  pitch_r      = module × tooth_count / 2
  tip_r        = pitch_r − module          (tooth tips, innermost surface)
  root_r_inner = pitch_r + 1.25 × module   (tooth base, meets ring body)
  outer_r      = root_r_inner + ring_wall_mm
  twist rate   = tan(helix_angle) / pitch_r  rad/mm

Hand convention (internal gear pair):
  Helical annulus + helical pinion mesh with the SAME hand — opposite of the
  external-external rule. Right annulus meshes with a right pinion.

Build method (no Solidify — boolean only, epsilon = 0.001 mm):
  1. Solid outer cylinder at outer_r
  2. Helical cutter: the annulus void profile (bore + N tooth spaces) extruded
     slice-by-slice with increasing twist — identical to how helical_gear.py
     builds its body, but using the swapped add/ded cutter profile.
  3. Boolean DIFFERENCE: outer cylinder − helical cutter → helical annulus.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, EnumProperty
from math import pi, cos, sin, tan, sqrt, radians, degrees, atan2
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
    ADDENDUM_COEFF and DEDENDUM_COEFF are swapped vs the external gear builder:
      ded_r → pitch_r − ADDENDUM × m   (inner bore at annulus tooth tips)
      add_r → pitch_r + DEDENDUM × m   (outer reach at annulus tooth roots)
    Extruding this profile (with helical twist) and subtracting it from an
    outer cylinder leaves a ring with involute teeth on its inner bore.
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


def _make_helical_cutter(context, cutter_profile, width_mm, hand_sign, ha_rad,
                          pitch_r, n_slices, mesh_name, obj_name):
    """
    Extrude the annulus cutter profile with helical twist into a solid.
    The cutter is slightly taller than the gear body (±BOOL_EPSILON) to
    guarantee clean boolean faces with no coplanar coincidence.
    """
    n         = len(cutter_profile)
    z_bot     = -BOOL_EPSILON
    z_top     = width_mm + BOOL_EPSILON
    total_h   = z_top - z_bot

    bm = bmesh.new()

    slices = []
    for k in range(n_slices):
        z        = z_bot + total_h * k / (n_slices - 1)
        # Twist relative to z=0 so the gear body and cutter align at the bottom face
        twist    = hand_sign * (z - z_bot) * tan(ha_rad) / pitch_r
        c, s     = cos(twist), sin(twist)
        verts    = [bm.verts.new((x * c - y * s, x * s + y * c, z))
                    for x, y in cutter_profile]
        slices.append(verts)

    bm.verts.index_update()

    # Boundary ring edges for the two capped ends, created explicitly and
    # BEFORE the side walls so triangle_fill (below) has them to work with;
    # the side-wall faces.new() calls further down reuse these same edges
    # automatically rather than creating duplicates.
    bot_edges = [bm.edges.new([slices[0][i], slices[0][(i + 1) % n]]) for i in range(n)]
    top_edges = [bm.edges.new([slices[-1][i], slices[-1][(i + 1) % n]]) for i in range(n)]

    # Caps: bmesh's constrained triangle_fill on the boundary loop, not a
    # fan to a center vertex. _build_annulus_cutter_profile inserts a
    # dedendum-circle point at the SAME angle as its neighbor wherever
    # base_r > ded_r (the straight radial segment representing an
    # undercut flank below the base circle) — real, correct tooth geometry,
    # not a mistake, but a fan triangle from that segment to the center
    # degenerates to exactly zero area since all three points are collinear
    # (a shared twist rotation doesn't change that). A zero-area triangle
    # looks harmless but isn't: its short edge is shared with a side-wall
    # face, so naively dropping the triangle turns that edge into a
    # non-manifold boundary edge instead. triangle_fill avoids the
    # degenerate case entirely by triangulating the actual polygon rather
    # than blindly fanning to an arbitrary point — verified directly (0
    # zero-area faces, 0 non-manifold edges across tooth counts from 8 to
    # 100, including cases that previously produced hundreds of
    # non-manifold edges).
    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=bot_edges)
    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=top_edges)

    # Side walls (reuse the boundary edges created above for k=0 and k=n_slices-2)
    for k in range(n_slices - 1):
        bot = slices[k]
        top = slices[k + 1]
        for i in range(n):
            ni = (i + 1) % n
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

class OBJECT_OT_helical_annulus_gear(bpy.types.Operator):
    """Helical annulus (internal ring) gear — twisted involute teeth on the inner bore."""
    bl_idname  = "object.helical_annulus_gear"
    bl_label   = "Helical Annulus Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_helical_same(self, context.window_manager.bmech_gear_target)

    tooth_count:        IntProperty(  name="Tooth Count",          default=40,   min=8,    soft_max=200,
                                      description="Internal gear tooth count; must exceed the mating pinion count")
    module:             FloatProperty(name="Module (mm)",           default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Transverse module — must match the mating pinion")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)",   default=20.0, min=10.0, max=45.0)
    helix_angle_deg:    FloatProperty(name="Helix Angle (°)",      default=20.0, min=1.0,  max=45.0,
                                      description="Tooth twist — 15–30° typical for FDM")
    hand: EnumProperty(
        name="Hand",
        items=[('RIGHT', "Right-hand", "Teeth twist CW looking from top"),
               ('LEFT',  "Left-hand",  "Teeth twist CCW looking from top")],
        default='RIGHT',
        description="Internal gear pairs use the SAME hand — right annulus + right pinion",
    )
    width_mm:           FloatProperty(name="Width (mm)",            default=10.0, min=1.0,  soft_max=80.0)
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",        default=5.0,  min=0.5,  soft_max=30.0,
                                      description="Radial wall thickness beyond the tooth root")
    n_slices:           IntProperty(  name="Slices",                default=16,   min=2,    soft_max=64,
                                      description="Z divisions for the helical cutter — more = smoother helix")
    outer_segs:         IntProperty(  name="Outer Segments",        default=64,   min=16,   soft_max=256,
                                      description="Facets on the outer cylindrical surface")

    def _derived(self):
        ha_rad       = radians(self.helix_angle_deg)
        pitch_r      = self.module * self.tooth_count / 2.0
        tip_r        = pitch_r - ADDENDUM_COEFF * self.module
        root_r_inner = pitch_r + DEDENDUM_COEFF * self.module
        outer_r      = root_r_inner + self.ring_wall_mm
        total_twist  = self.width_mm * tan(ha_rad) / pitch_r
        normal_module = self.module * cos(ha_rad)
        pa_max        = gear_matching.max_pressure_angle_deg(self.tooth_count, DEDENDUM_COEFF)
        return ha_rad, pitch_r, tip_r, root_r_inner, outer_r, total_twist, normal_module, pa_max

    def draw(self, context):
        layout = self.layout
        ha_rad, pitch_r, tip_r, root_r_inner, outer_r, total_twist, normal_module, pa_max = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")
        col.prop(self, "helix_angle_deg")
        col.prop(self, "hand")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "width_mm")
        col.prop(self, "ring_wall_mm")
        col.prop(self, "n_slices")
        col.prop(self, "outer_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Pitch Ø:       %.2f mm" % (pitch_r * 2))
        box.label(text="Tip Ø:         %.2f mm" % (tip_r * 2))
        box.label(text="Root Ø:        %.2f mm" % (root_r_inner * 2))
        box.label(text="Outer Ø:       %.2f mm" % (outer_r * 2))
        box.label(text="Normal module: %.3f"    % normal_module)
        box.label(text="Total twist:   %.1f °"  % degrees(total_twist))

        if tip_r <= 0:
            layout.label(text="Module too large — tip radius ≤ 0", icon='ERROR')
        layout.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, DEDENDUM_COEFF))
        ha_rad, pitch_r, tip_r, root_r_inner, outer_r, total_twist, _, pa_max = self._derived()

        if tip_r <= 0:
            return {'CANCELLED'}

        hand_sign    = 1.0 if self.hand == 'RIGHT' else -1.0
        cursor       = context.scene.cursor.location.copy()

        # 1. Solid outer cylinder (the ring body blank)
        body = _make_cylinder(
            context, outer_r,
            0.0, self.width_mm,
            self.outer_segs,
            "HelicalAnnulusGearMesh", "HelicalAnnulusGear"
        )
        body.location = cursor

        # 2. Helical cutter — twisted annulus void profile
        cutter_pts = _build_annulus_cutter_profile(
            self.module, self.tooth_count, self.pressure_angle_deg
        )
        cutter = _make_helical_cutter(
            context, cutter_pts,
            self.width_mm, hand_sign, ha_rad, pitch_r, self.n_slices,
            "__HelAnnCutMesh", "__HelAnnCut"
        )
        cutter.location = cursor

        # 3. Boolean DIFFERENCE
        bpy.ops.object.select_all(action='DESELECT')
        body.select_set(True)
        context.view_layer.objects.active = body

        mod           = body.modifiers.new("HelixBore", 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object    = cutter
        mod.solver    = 'EXACT'

        bpy.ops.object.modifier_apply(modifier="HelixBore")

        bpy.data.objects.remove(cutter, do_unlink=True)

        body.select_set(True)
        context.view_layer.objects.active = body

        gear_matching.stamp_gear(body, "helical_annulus", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count,
                                  helix_angle_deg=self.helix_angle_deg, hand=self.hand)

        self.report({'INFO'},
            "Helical annulus: %d teeth, %.1f° helix, module %.1f, outer Ø %.1f mm"
            % (self.tooth_count, self.helix_angle_deg, self.module, outer_r * 2))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_helical_annulus_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
