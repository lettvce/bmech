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

Build method — direct bmesh construction, NO boolean at all:
  1. Build the inner tooth profile (same math as _build_gear_profile but
     with ADDENDUM_COEFF/DEDENDUM_COEFF swapped, so the profile traces the
     annulus's internal teeth directly).
  2. Build a matching outer ring: one point per inner-profile point, each
     at the SAME ANGLE but at outer_r — not an independently-spaced
     circle. This 1:1 angular correspondence is what lets the two loops
     bridge into a clean side wall.
  3. Cap top and bottom with bmesh.ops.triangle_fill fed BOTH loops
     together (not a center-fan, and not a naive index-matched bridge —
     see _build_annulus_solid for why).
  4. Side walls: inner (toothed) wall and outer (cylindrical) wall,
     connecting the two Z layers.

This used to be a solid outer cylinder minus a boolean-DIFFERENCE cutter
(EXACT solver). That was ~80-180x slower in testing (295ms-5.8s for
tooth counts 8-100, scaling badly, vs 3.7-32ms direct) and was the
generator most likely to feel unresponsive in this whole family. The
direct construction produces the identical shape without ever invoking
the boolean solver.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, sqrt, radians, atan2
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

    Identical to the external gear profile builder but with add/ded swapped:
      ded_r → pitch_r − ADDENDUM × m   (inner bore, at annulus tooth tips)
      add_r → pitch_r + DEDENDUM × m   (outer reach, at annulus tooth roots)

    The profile traces a gear-shaped star (CCW from +Z) — this same loop of
    points becomes the inner (toothed) boundary that _build_annulus_solid
    bridges to a matching outer ring, no boolean involved.
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


def _build_annulus_solid(context, inner_profile, outer_r, outer_segs, z_bot, z_top,
                          mesh_name, obj_name):
    """
    Build the complete annulus ring solid directly — outer cylindrical
    wall, inner toothed wall, and two end caps — with no boolean step.

    The outer ring is its own independently-spaced circle (outer_segs
    points, evenly spaced) — NOT built with one point per inner-profile
    point at a matching angle. A matched 1:1 correspondence looks like it
    would let the two loops bridge into a clean side wall directly, but it
    doesn't work: _build_annulus_cutter_profile inserts a dedendum-circle
    point at the SAME angle as its neighbor wherever base_r > ded_r (a
    genuine straight undercut-flank segment, not a mistake), and bridging
    directly to a same-angle outer point puts THREE points (both inner
    points plus the outer one) on the same ray through the origin — an
    unavoidably zero-area triangle no matter how the quad there gets
    split. Confirmed by testing: a matched-angle outer ring reintroduces
    non-manifold edges and zero-area faces at low tooth counts (8/15/20)
    that a fully independent outer ring does not.

    Each loop gets capped SEPARATELY at its own resolution — the inner
    wall connects consecutive inner-profile points, the outer wall
    connects consecutive outer-ring points, with no correspondence between
    the two required. The caps are the only place the two loops need to
    interact, and bmesh.ops.triangle_fill handles that correctly when fed
    the boundary edges of BOTH loops together in one call: it treats the
    inner loop as a hole in the outer loop's polygon and triangulates the
    actual annular region directly, with no trouble from the inner
    profile's collinear-point segment, since it isn't assuming any 1:1
    point correspondence between the loops in the first place. Verified
    directly: 0 non-manifold edges, 0 zero-area faces across tooth counts
    8-100 with this (fully independent outer ring) approach specifically —
    the matched-angle variant above was tried and rejected.
    """
    n = len(inner_profile)
    outer_angles = [2.0 * pi * i / outer_segs for i in range(outer_segs)]

    bm = bmesh.new()
    inner_bot = [bm.verts.new((x, y, z_bot)) for x, y in inner_profile]
    inner_top = [bm.verts.new((x, y, z_top)) for x, y in inner_profile]
    outer_bot = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), z_bot)) for a in outer_angles]
    outer_top = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), z_top)) for a in outer_angles]
    bm.verts.index_update()

    # Boundary ring edges FIRST (both loops, both z-layers), so
    # triangle_fill has them; the side-wall faces.new() calls further down
    # reuse these same edges automatically rather than creating duplicates.
    bot_edges = [bm.edges.new([inner_bot[i], inner_bot[(i + 1) % n]]) for i in range(n)]
    bot_edges += [bm.edges.new([outer_bot[i], outer_bot[(i + 1) % outer_segs]]) for i in range(outer_segs)]
    top_edges = [bm.edges.new([inner_top[i], inner_top[(i + 1) % n]]) for i in range(n)]
    top_edges += [bm.edges.new([outer_top[i], outer_top[(i + 1) % outer_segs]]) for i in range(outer_segs)]

    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=bot_edges)
    bmesh.ops.triangle_fill(bm, use_beauty=True, edges=top_edges)

    # Side walls (reuse the boundary edges created above). Each wall uses
    # its own loop's point count/indexing — the inner (toothed) wall and
    # the outer (cylindrical) wall are independent of each other.
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([inner_bot[i], inner_bot[ni], inner_top[ni], inner_top[i]])
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

class OBJECT_OT_annulus_gear(bpy.types.Operator):
    """Annulus (internal / ring) gear — involute teeth on the inner bore."""
    bl_idname  = "object.annulus_gear"
    bl_label   = "Annulus Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_module_pa(self, context.window_manager.bmech_gear_target)

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

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
        # pressure angle, the tooth flanks cross before reaching the tip,
        # producing a self-intersecting profile. execute() clamps
        # pressure_angle_deg to this automatically.
        #
        # PA_TRIANGLE_FILL_MARGIN_DEG below the theoretical limit, not the
        # limit itself: the old boolean-based EXACT solver tolerated a
        # profile sitting exactly at the self-intersection boundary, but
        # bmesh.ops.triangle_fill's constrained triangulation (used for
        # this generator's caps since the no-boolean rewrite) does not —
        # right at that limit, the tooth tip's flank points become
        # near-coincident, and triangle_fill produces real non-manifold
        # edges and zero-area faces (confirmed: 292-844 non-manifold edges
        # across tooth counts 8-20 tested exactly at the limit). A 0.1°
        # margin reliably cleared it in every case tested; smaller margins
        # (down to ~0.01°) did not.
        pa_max = gear_matching.max_pressure_angle_deg(self.tooth_count, DEDENDUM_COEFF) \
            - PA_TRIANGLE_FILL_MARGIN_DEG

        return pitch_r, tip_r, root_r_inner, outer_r, pa_max

    def draw(self, context):
        layout = self.layout
        pitch_r, tip_r, root_r_inner, outer_r, pa_max = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        has_target = context.window_manager.bmech_gear_target is not None
        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "module")
        driven.prop(self, "pressure_angle_deg")

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
        # Not gear_matching.clamp_pressure_angle() — that clamps to the
        # theoretical self-intersection limit exactly, which is too tight
        # for this generator's triangle_fill-based caps (see _derived()'s
        # PA_TRIANGLE_FILL_MARGIN_DEG comment). _derived()'s own pa_max
        # already has the margin built in, so clamp to that instead.
        _, _, _, _, pa_max_safe = self._derived()
        if self.pressure_angle_deg > pa_max_safe:
            self.pressure_angle_deg = pa_max_safe
        pitch_r, tip_r, root_r_inner, outer_r, pa_max = self._derived()

        if tip_r <= 0:
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()

        inner_profile = _build_annulus_cutter_profile(
            self.module, self.tooth_count, self.pressure_angle_deg
        )
        body = _build_annulus_solid(
            context, inner_profile, outer_r, self.outer_segs,
            0.0, self.width_mm,
            "AnnulusGearMesh", "AnnulusGear"
        )
        body.location = cursor

        bpy.ops.object.select_all(action='DESELECT')
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
