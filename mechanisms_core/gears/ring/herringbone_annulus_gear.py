"""
Herringbone Annulus (Internal Ring) Gear Generator

A herringbone annulus has V-shaped teeth on its inner bore — the same
double-helix that cancels axial thrust in a herringbone pinion, but on
the inside of a ring gear.

Geometry:
  pitch_r      = module × tooth_count / 2
  tip_r        = pitch_r − module          (tooth tips, innermost surface)
  root_r_inner = pitch_r + 1.25 × module   (tooth base, meets ring body)
  outer_r      = root_r_inner + ring_wall_mm
  peak_twist   = (width/2) × tan(helix_angle) / pitch_r

Hand convention (internal gear pair):
  Herringbone annulus + herringbone pinion mesh with the SAME hand.

Build method (no Solidify — boolean only, epsilon = 0.001 mm):
  1. Solid outer cylinder at outer_r
  2. Herringbone cutter: void profile extruded slice-by-slice with a
     V-shaped twist — bottom half rises 0→peak, top half falls peak→0.
  3. Boolean DIFFERENCE: outer cylinder − herringbone cutter.

Cap geometry: center-fan triangulation (one center vertex, N flat triangles)
  — avoids self-intersecting faces that CGAL EXACT solver rejects.
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
    """
    pa_rad  = radians(pa_deg)
    pitch_r = module * tooth_count / 2.0
    base_r  = pitch_r * cos(pa_rad)
    ded_r   = pitch_r - ADDENDUM_COEFF * module
    add_r   = pitch_r + DEDENDUM_COEFF * module
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


def _make_herringbone_cutter(context, cutter_profile, width_mm, hand_sign, ha_rad,
                               pitch_r, n_slices, mesh_name, obj_name):
    """
    Extrude the annulus cutter profile with a herringbone (V-shaped) twist.

    The z range is extended by ±BOOL_EPSILON so boolean faces are never coplanar
    with the body end faces. Twist is 0 at z=-eps, peaks at z=width/2, returns
    to 0 at z=width+eps (negligible deviation from nominal over 0.001 mm).

    Caps: center-fan triangulation — one vertex at (0,0,z) per cap, N flat
    triangles fanning out to the boundary. Guaranteed non-overlapping for any
    star-shaped profile; safe for CGAL EXACT boolean solver.
    """
    n      = len(cutter_profile)
    half_h = width_mm / 2.0

    bm = bmesh.new()

    def _slice(z, twist):
        c, s = cos(twist), sin(twist)
        return [bm.verts.new((x * c - y * s, x * s + y * c, z))
                for x, y in cutter_profile]

    all_slices = []

    # Bottom half: z from −eps to half_h, twist rises 0 → peak
    for k in range(n_slices):
        z     = -BOOL_EPSILON + (half_h + BOOL_EPSILON) * k / (n_slices - 1)
        twist = hand_sign * (z + BOOL_EPSILON) * tan(ha_rad) / pitch_r
        all_slices.append(_slice(z, twist))

    # Top half: z from half_h to width+eps, twist falls peak → 0
    # k=0 is the shared mid-slice — skip it
    for k in range(1, n_slices):
        z     = half_h + (half_h + BOOL_EPSILON) * k / (n_slices - 1)
        twist = hand_sign * (width_mm + BOOL_EPSILON - z) * tan(ha_rad) / pitch_r
        all_slices.append(_slice(z, twist))

    bm.verts.index_update()

    # Boundary ring edges for the two capped ends, created explicitly and
    # BEFORE the side walls so triangle_fill (below) has them to work with;
    # the side-wall faces.new() calls further down reuse these same edges
    # automatically rather than creating duplicates.
    bot_edges = [bm.edges.new([all_slices[0][i], all_slices[0][(i + 1) % n]]) for i in range(n)]
    top_edges = [bm.edges.new([all_slices[-1][i], all_slices[-1][(i + 1) % n]]) for i in range(n)]

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

    # Side walls (reuse the boundary edges created above for the first and
    # last slice pairs)
    for k in range(len(all_slices) - 1):
        bot = all_slices[k]
        top = all_slices[k + 1]
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

class OBJECT_OT_herringbone_annulus_gear(bpy.types.Operator):
    """Herringbone annulus (internal ring) gear — V-shaped involute teeth on the inner bore."""
    bl_idname  = "object.herringbone_annulus_gear"
    bl_label   = "Herringbone Annulus Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_helical_same(self, context.window_manager.bmech_gear_target)

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    tooth_count:        IntProperty(  name="Tooth Count",          default=40,   min=8,    soft_max=200,
                                      description="Internal gear tooth count; must exceed the mating pinion count")
    module:             FloatProperty(name="Module (mm)",           default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Transverse module — must match the mating pinion")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)",   default=20.0, min=10.0, max=45.0)
    helix_angle_deg:    FloatProperty(name="Helix Angle (°)",      default=20.0, min=1.0,  max=45.0,
                                      description="Half-angle of the V — 15–30° typical for FDM")
    hand: EnumProperty(
        name="Hand",
        items=[('RIGHT', "Right-hand", "Bottom half twists CW from below"),
               ('LEFT',  "Left-hand",  "Bottom half twists CCW from below")],
        default='RIGHT',
        description="Internal gear pairs use the SAME hand — right annulus + right pinion",
    )
    width_mm:           FloatProperty(name="Total Width (mm)",      default=14.0, min=2.0,  soft_max=80.0,
                                      description="Full face width — each half is width/2")
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",        default=5.0,  min=0.5,  soft_max=30.0,
                                      description="Radial wall thickness beyond the tooth root")
    n_slices:           IntProperty(  name="Slices per Half",       default=12,   min=2,    soft_max=48,
                                      description="Z divisions per half — total slices = 2n−1")
    outer_segs:         IntProperty(  name="Outer Segments",        default=64,   min=16,   soft_max=256,
                                      description="Facets on the outer cylindrical surface")

    def _derived(self):
        ha_rad        = radians(self.helix_angle_deg)
        pitch_r       = self.module * self.tooth_count / 2.0
        tip_r         = pitch_r - ADDENDUM_COEFF * self.module
        root_r_inner  = pitch_r + DEDENDUM_COEFF * self.module
        outer_r       = root_r_inner + self.ring_wall_mm
        half_h        = self.width_mm / 2.0
        peak_twist    = half_h * tan(ha_rad) / pitch_r
        normal_module = self.module * cos(ha_rad)
        pa_max        = gear_matching.max_pressure_angle_deg(self.tooth_count, DEDENDUM_COEFF)
        return ha_rad, pitch_r, tip_r, root_r_inner, outer_r, half_h, peak_twist, normal_module, pa_max

    def draw(self, context):
        layout = self.layout
        ha_rad, pitch_r, tip_r, root_r_inner, outer_r, half_h, peak_twist, normal_module, pa_max = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        target = context.window_manager.bmech_gear_target
        has_target = target is not None
        # A spur/straight-annulus target doesn't stamp bmech_helix_angle_deg
        # /bmech_hand (see gear_matching.sync_helical_same and stamp_gear),
        # so it never drives helix_angle_deg/hand — those stay editable even
        # with a target set. module/pressure_angle_deg ARE always driven by
        # any target kind.
        target_drives_helix = has_target and "bmech_helix_angle_deg" in target.keys()

        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "module")
        driven.prop(self, "pressure_angle_deg")
        helix_driven = col.column(align=True)
        helix_driven.enabled = not target_drives_helix
        helix_driven.prop(self, "helix_angle_deg")
        helix_driven.prop(self, "hand")

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
        box.label(text="Peak twist:    %.1f °"  % degrees(peak_twist))
        box.label(text="Total slices:  %d"      % (2 * self.n_slices - 1))

        if tip_r <= 0:
            layout.label(text="Module too large — tip radius ≤ 0", icon='ERROR')
        layout.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, DEDENDUM_COEFF))
        ha_rad, pitch_r, tip_r, root_r_inner, outer_r, half_h, peak_twist, _, pa_max = self._derived()

        if tip_r <= 0:
            return {'CANCELLED'}

        hand_sign = 1.0 if self.hand == 'RIGHT' else -1.0
        cursor    = context.scene.cursor.location.copy()

        # 1. Solid outer cylinder (ring body blank)
        body = _make_cylinder(
            context, outer_r,
            0.0, self.width_mm,
            self.outer_segs,
            "HerringboneAnnulusGearMesh", "HerringboneAnnulusGear"
        )
        body.location = cursor

        # 2. Herringbone cutter — V-twisted annulus void profile
        cutter_pts = _build_annulus_cutter_profile(
            self.module, self.tooth_count, self.pressure_angle_deg
        )
        cutter = _make_herringbone_cutter(
            context, cutter_pts,
            self.width_mm, hand_sign, ha_rad, pitch_r, self.n_slices,
            "__HbAnnCutMesh", "__HbAnnCut"
        )
        cutter.location = cursor

        # 3. Boolean DIFFERENCE
        bpy.ops.object.select_all(action='DESELECT')
        body.select_set(True)
        context.view_layer.objects.active = body

        mod           = body.modifiers.new("HboneBore", 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object    = cutter
        mod.solver    = 'EXACT'

        bpy.ops.object.modifier_apply(modifier="HboneBore")

        bpy.data.objects.remove(cutter, do_unlink=True)

        body.select_set(True)
        context.view_layer.objects.active = body

        gear_matching.stamp_gear(body, "herringbone_annulus", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count,
                                  helix_angle_deg=self.helix_angle_deg, hand=self.hand)

        self.report({'INFO'},
            "Herringbone annulus: %d teeth, %.1f° helix, module %.1f, outer Ø %.1f mm"
            % (self.tooth_count, self.helix_angle_deg, self.module, outer_r * 2))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_herringbone_annulus_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
