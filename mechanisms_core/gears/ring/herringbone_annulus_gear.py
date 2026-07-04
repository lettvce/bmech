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

Build method — direct bmesh construction, NO boolean at all:
  1. Build 2*n_slices-1 V-twisted Z-layers of the inner tooth profile
     (bore + N tooth spaces) — bottom half rises 0->peak, top half falls
     peak->0, meeting at the shared mid-slice.
  2. Build a PLAIN, UNTWISTED outer ring — only 2 Z-layers (bottom, top),
     its own independently-spaced circle (outer_segs points). The outer
     surface doesn't twist; only the inner teeth do.
  3. Cap top and bottom with bmesh.ops.triangle_fill fed BOTH the inner
     profile's boundary loop and the outer ring's boundary loop together —
     same technique as annulus_gear.py, see that file for why a naive
     index-matched bridge between the loops doesn't work.
  4. Side walls: the V-twisted inner (toothed) wall between consecutive
     Z-slices, and the plain outer (cylindrical) wall between its own
     bottom/top rings.

This used to be a solid outer cylinder minus a boolean-DIFFERENCE
herringbone cutter (EXACT solver) — see annulus_gear.py for the fuller
writeup of the same rewrite applied to the straight annulus gear first,
and helical_annulus_gear.py for the single-twist (non-V) version of this
same extension.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, EnumProperty
from math import pi, cos, sin, tan, sqrt, radians, degrees, atan2
from .. import gear_matching

INVOLUTE_POINTS = 15
ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
PA_TRIANGLE_FILL_MARGIN_DEG = 0.2


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

    This same loop of points becomes the inner (toothed) boundary that
    _build_herringbone_annulus_solid sweeps through 2*n_slices-1 V-twisted
    Z-layers and bridges to a matching outer ring — no boolean involved.
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


def _build_herringbone_annulus_solid(context, inner_profile, width_mm, outer_r, outer_segs,
                                      hand_sign, ha_rad, pitch_r, n_slices,
                                      mesh_name, obj_name):
    """
    Build the complete herringbone annulus ring solid directly — V-twisted
    inner toothed wall, plain outer cylindrical wall, and two end caps —
    with no boolean step. See annulus_gear.py's _build_annulus_solid for
    the fuller rationale (this is the V-twist extension of the same
    technique) and helical_annulus_gear.py for the single-twist version.

    - The outer ring stays a PLAIN, UNTWISTED, independently-spaced circle
      (only 2 Z-layers) — the outer surface doesn't twist, only the inner
      teeth do, and giving the outer ring one point per inner-profile
      point at a matching angle would reintroduce zero-area triangles at
      the tooth profile's collinear dedendum-circle point insertion.
    - Each end is capped with bmesh.ops.triangle_fill fed the boundary
      edges of BOTH the (twisted, at that Z) inner profile ring and the
      (untwisted) outer ring together.

    Twist is relative to z=0/width_mm (not padded past the real Z bounds
    the way the old boolean cutter needed to be, since there's no boolean
    here to dodge a coincident-face issue with) — bottom half rises
    0->peak over [0, width_mm/2], top half falls peak->0 over
    [width_mm/2, width_mm], meeting at the shared mid-slice.
    """
    n      = len(inner_profile)
    half_h = width_mm / 2.0

    bm = bmesh.new()

    def _slice(z, twist):
        c, s = cos(twist), sin(twist)
        return [bm.verts.new((x * c - y * s, x * s + y * c, z)) for x, y in inner_profile]

    inner_slices = []
    for k in range(n_slices):
        z     = half_h * k / (n_slices - 1)
        twist = hand_sign * z * tan(ha_rad) / pitch_r
        inner_slices.append(_slice(z, twist))
    for k in range(1, n_slices):
        z     = half_h + half_h * k / (n_slices - 1)
        twist = hand_sign * (width_mm - z) * tan(ha_rad) / pitch_r
        inner_slices.append(_slice(z, twist))

    outer_angles = [2.0 * pi * i / outer_segs for i in range(outer_segs)]
    outer_bot = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), 0.0)) for a in outer_angles]
    outer_top = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), width_mm)) for a in outer_angles]
    bm.verts.index_update()

    # Boundary ring edges FIRST (both loops, both ends), so triangle_fill
    # has them; the side-wall faces.new() calls further down reuse these
    # same edges automatically rather than creating duplicates.
    bot_edges = [bm.edges.new([inner_slices[0][i], inner_slices[0][(i + 1) % n]]) for i in range(n)]
    bot_edges += [bm.edges.new([outer_bot[i], outer_bot[(i + 1) % outer_segs]]) for i in range(outer_segs)]
    top_edges = [bm.edges.new([inner_slices[-1][i], inner_slices[-1][(i + 1) % n]]) for i in range(n)]
    top_edges += [bm.edges.new([outer_top[i], outer_top[(i + 1) % outer_segs]]) for i in range(outer_segs)]

    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=bot_edges)
    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=top_edges)

    # Inner (toothed) V-twisted wall
    for k in range(len(inner_slices) - 1):
        bot, top = inner_slices[k], inner_slices[k + 1]
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

    # Outer (cylindrical) wall
    for i in range(outer_segs):
        ni = (i + 1) % outer_segs
        bm.faces.new([outer_bot[ni], outer_bot[i], outer_top[i], outer_top[ni]])

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
        # PA_TRIANGLE_FILL_MARGIN_DEG below the theoretical self-intersection
        # limit, not the limit itself — see annulus_gear.py's _derived() for
        # why: bmesh.ops.triangle_fill's constrained triangulation (used for
        # this generator's caps since the no-boolean rewrite) is numerically
        # fragile right at that limit in a way the old EXACT-solver boolean
        # wasn't.
        pa_max = gear_matching.max_pressure_angle_deg(self.tooth_count, DEDENDUM_COEFF) \
            - PA_TRIANGLE_FILL_MARGIN_DEG
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
        # hand specifically stays editable even when driven if the target
        # is plain helical (this gear is herringbone) — a plain helical
        # gear only meshes one half of THIS gear's V at a time, and which
        # hand is correct depends on which half, something the sync can't
        # know. See gear_matching.hand_target_ambiguous.
        hand_ambiguous = gear_matching.hand_target_ambiguous(True, target)

        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "module")
        driven.prop(self, "pressure_angle_deg")
        helix_driven = col.column(align=True)
        helix_driven.enabled = not target_drives_helix
        helix_driven.prop(self, "helix_angle_deg")
        hand_driven = col.column(align=True)
        hand_driven.enabled = not target_drives_helix or hand_ambiguous
        hand_driven.prop(self, "hand")

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
        # Not gear_matching.clamp_pressure_angle() — see _derived()'s
        # PA_TRIANGLE_FILL_MARGIN_DEG comment; _derived()'s own pa_max
        # already has the margin built in.
        _, _, _, _, _, _, _, _, pa_max_safe = self._derived()
        if self.pressure_angle_deg > pa_max_safe:
            self.pressure_angle_deg = pa_max_safe
        ha_rad, pitch_r, tip_r, root_r_inner, outer_r, half_h, peak_twist, _, pa_max = self._derived()

        if tip_r <= 0:
            return {'CANCELLED'}

        hand_sign = 1.0 if self.hand == 'RIGHT' else -1.0
        cursor    = context.scene.cursor.location.copy()

        inner_profile = _build_annulus_cutter_profile(
            self.module, self.tooth_count, self.pressure_angle_deg
        )
        body = _build_herringbone_annulus_solid(
            context, inner_profile, self.width_mm, outer_r, self.outer_segs,
            hand_sign, ha_rad, pitch_r, self.n_slices,
            "HerringboneAnnulusGearMesh", "HerringboneAnnulusGear"
        )
        body.location = cursor

        bpy.ops.object.select_all(action='DESELECT')
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
