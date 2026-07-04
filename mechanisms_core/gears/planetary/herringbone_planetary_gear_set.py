"""
Herringbone Planetary Gear Set Generator

Same topology as helical_planetary_gear_set.py but all three members have
herringbone (V-shaped) teeth.

Hand convention (same as helical):
  Sun ↔ Planet : OPPOSITE hands  (external-external)
  Planet ↔ Ring: SAME hand       (external-internal)
  ∴ Ring = opposite of Sun = same as Planet.

Ring phase, assembly condition, and planet roll formula are identical to the
spur and helical planetary sets — only the mesh builders change.

Sun / Planet build: two mirrored helical halves sharing a mid-slice (2n−1 total).
Ring build       : solid cylinder + boolean DIFFERENCE with herringbone cutter.
                   Cutter uses center-fan caps (safe for CGAL EXACT solver).
Planet mesh is shared across all N linked copies.
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
SPACE_PTS       = 4


def _involute_pt(base_r, t):
    return (base_r * (cos(t) + t * sin(t)),
            base_r * (sin(t) - t * cos(t)))


def _involute_t_at_r(base_r, r):
    ratio = r / base_r
    return 0.0 if ratio < 1.0 else sqrt(ratio * ratio - 1.0)


def _rot(x, y, a):
    c, s = cos(a), sin(a)
    return (x * c - y * s, x * s + y * c)


def _build_gear_profile(module, tooth_count, pa_deg, pip_gap=0.0):
    """External involute gear cross-section profile (2D, CCW)."""
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count) - pip_gap / pitch_r
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
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

    profile = []
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


def _build_annulus_cutter_profile(module, tooth_count, pa_deg, pip_gap=0.0):
    """Annulus bore void profile (add/ded swapped; for ring gear boolean cutter)."""
    pa_rad  = radians(pa_deg)
    pitch_r = module * tooth_count / 2.0
    base_r  = pitch_r * cos(pa_rad)
    ded_r   = pitch_r - ADDENDUM_COEFF * module
    add_r   = pitch_r + DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count) + pip_gap / pitch_r
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
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

    profile = []
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


def _make_herringbone_mesh(module, tooth_count, pa_deg, width_mm,
                            hand_sign, ha_rad, n_slices, mesh_name, pip_gap=0.0):
    """
    Herringbone external gear mesh data (no object created yet).
    Bottom half (k=0..n-1): twist 0 → peak at half_h.
    Top half    (k=1..n-1): twist peak → 0 at width_mm.  Total: 2n−1 slices.
    """
    profile = _build_gear_profile(module, tooth_count, pa_deg, pip_gap)
    pitch_r = module * tooth_count / 2.0
    n       = len(profile)
    half_h  = width_mm / 2.0

    bm = bmesh.new()

    def _slice(z, twist):
        c, s = cos(twist), sin(twist)
        return [bm.verts.new((x * c - y * s, x * s + y * c, z))
                for x, y in profile]

    all_slices = []

    for k in range(n_slices):
        z     = half_h * k / (n_slices - 1)
        twist = hand_sign * z * tan(ha_rad) / pitch_r
        all_slices.append(_slice(z, twist))

    for k in range(1, n_slices):
        z     = half_h + half_h * k / (n_slices - 1)
        twist = hand_sign * (width_mm - z) * tan(ha_rad) / pitch_r
        all_slices.append(_slice(z, twist))

    bm.verts.index_update()

    # Cap with a single n-gon over the whole profile ring, not a triangle
    # fan to a center vertex. _build_gear_profile inserts a dedendum-circle
    # point at the SAME angle as the adjacent involute departure point
    # wherever base_r > ded_r, so two adjacent ring points can be exactly
    # collinear with the origin — a fan triangle to the center then has
    # exactly zero area (confirmed: 28/12 zero-area cap faces on the
    # default sun/planet gears here before this fix). A single n-gon face
    # has no center vertex and no per-point triangles, so this can't occur.
    # Unlike _make_herringbone_cutter_obj below (the ring/annulus cutter),
    # this is an EXTERNAL gear profile — not deeply concave enough to risk
    # the self-intersecting-triangulation problem that function's fan-cap
    # is deliberately avoiding, so switching this one to an n-gon is safe.
    bm.faces.new(all_slices[0])
    bm.faces.new(list(reversed(all_slices[-1])))

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
    return me


def _make_cylinder_obj(context, r, z_bot, z_top, n_segs, mesh_name, obj_name):
    """Solid cylinder object."""
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


def _make_herringbone_cutter_obj(context, cutter_profile, width_mm,
                                  hand_sign, ha_rad, pitch_r,
                                  n_slices, mesh_name, obj_name):
    """
    Herringbone cutter prism for ring boolean.
    Z range ±BOOL_EPSILON beyond body; center-fan caps (safe for CGAL EXACT).
    Twist: 0 at z=−eps, peaks at half_h, 0 again at width+eps.
    """
    n      = len(cutter_profile)
    half_h = width_mm / 2.0

    bm = bmesh.new()

    def _slice(z, twist):
        c, s = cos(twist), sin(twist)
        return [bm.verts.new((x * c - y * s, x * s + y * c, z))
                for x, y in cutter_profile]

    all_slices = []

    for k in range(n_slices):
        z     = -BOOL_EPSILON + (half_h + BOOL_EPSILON) * k / (n_slices - 1)
        twist = hand_sign * (z + BOOL_EPSILON) * tan(ha_rad) / pitch_r
        all_slices.append(_slice(z, twist))

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
    # fan to a center vertex. cutter_profile inserts a dedendum-circle
    # point at the SAME angle as its neighbor wherever base_r > ded_r (the
    # straight radial segment representing an undercut flank below the
    # base circle) — real, correct tooth geometry, not a mistake, but a fan
    # triangle from that segment to the center degenerates to exactly zero
    # area since all three points are collinear (a shared twist rotation
    # doesn't change that). A zero-area triangle looks harmless but isn't:
    # its short edge is shared with a side-wall face, so naively dropping
    # the triangle (an earlier attempt at this fix) turns that edge into a
    # non-manifold boundary edge instead. triangle_fill avoids the
    # degenerate case entirely by triangulating the actual polygon rather
    # than blindly fanning to an arbitrary point.
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

class OBJECT_OT_herringbone_planetary_gear_set(bpy.types.Operator):
    """Herringbone planetary gear set — sun, planets, and ring with V-shaped teeth."""
    bl_idname  = "object.herringbone_planetary_gear_set"
    bl_label   = "Herringbone Planetary Gear Set"
    bl_options = {'REGISTER', 'UNDO'}

    sun_teeth:          IntProperty(  name="Sun Teeth",          default=12,   min=4,    soft_max=100)
    planet_teeth:       IntProperty(  name="Planet Teeth",       default=18,   min=4,    soft_max=100,
                                      description="Ring = Sun + 2 × Planet (auto-derived)")
    planet_count:       IntProperty(  name="Planet Count",       default=3,    min=2,    max=8,
                                      description="(Sun + Ring) must be divisible by this")
    module:             FloatProperty(name="Module (mm)",         default=2.0,  min=0.1,  soft_max=20.0)
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    helix_angle_deg:    FloatProperty(name="Helix Angle (°)",    default=20.0, min=1.0,  max=45.0,
                                      description="Half-angle of the V for all three members")
    hand: EnumProperty(
        name="Sun Hand",
        items=[('RIGHT', "Right (sun)", "Sun RH → planets LH → ring LH"),
               ('LEFT',  "Left (sun)",  "Sun LH → planets RH → ring RH")],
        default='RIGHT',
        description="Sun gear hand — planets and ring are derived automatically",
    )
    width_mm:           FloatProperty(name="Total Width (mm)",   default=14.0, min=2.0,  soft_max=80.0,
                                      description="Full face width — each half is width/2")
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",     default=5.0,  min=0.5,  soft_max=30.0)
    pip_gap:            FloatProperty(name="PiP Gap (mm)",       default=0.2,  min=0.0,  soft_max=2.0,
                                      description="Radial clearance at tooth tips for print-in-place")
    n_slices:           IntProperty(  name="Slices per Half",    default=12,   min=2,    soft_max=48,
                                      description="Z divisions per half — total slices = 2n−1")
    outer_segs:         IntProperty(  name="Outer Segments",     default=64,   min=16,   soft_max=256)

    def _derived(self):
        ring_teeth    = self.sun_teeth + 2 * self.planet_teeth
        ha_rad        = radians(self.helix_angle_deg)
        r_sun         = self.module * self.sun_teeth    / 2.0
        r_planet      = self.module * self.planet_teeth / 2.0
        r_ring        = self.module * ring_teeth        / 2.0
        center_dist   = r_sun + r_planet
        outer_r       = r_ring + DEDENDUM_COEFF * self.module + self.ring_wall_mm
        normal_module = self.module * cos(ha_rad)
        peak_twist    = (self.width_mm / 2.0) * tan(ha_rad) / r_sun
        assembly_ok   = (self.sun_teeth + ring_teeth) % self.planet_count == 0
        pa_max = min(
            gear_matching.max_pressure_angle_deg(self.sun_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(self.planet_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(ring_teeth, DEDENDUM_COEFF),
        )
        return (ring_teeth, ha_rad, r_sun, r_planet, r_ring,
                center_dist, outer_r, normal_module, peak_twist, assembly_ok, pa_max)

    def draw(self, context):
        layout = self.layout
        (ring_teeth, ha_rad, r_sun, r_planet, r_ring,
         center_dist, outer_r, normal_module, peak_twist, assembly_ok, pa_max) = self._derived()

        col = layout.column(align=True)
        col.prop(self, "sun_teeth")
        col.prop(self, "planet_teeth")
        col.prop(self, "planet_count")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "helix_angle_deg")
        col.prop(self, "hand")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "width_mm")
        col.prop(self, "ring_wall_mm")
        col.prop(self, "pip_gap")
        col.prop(self, "n_slices")
        col.prop(self, "outer_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Ring teeth:       %d"           % ring_teeth)
        box.label(text="Sun pitch Ø:      %.2f mm"      % (r_sun    * 2))
        box.label(text="Planet pitch Ø:   %.2f mm"      % (r_planet * 2))
        box.label(text="Ring pitch Ø:     %.2f mm"      % (r_ring   * 2))
        box.label(text="Center dist:      %.2f mm"      % center_dist)
        box.label(text="Outer Ø:          %.2f mm"      % (outer_r  * 2))
        box.label(text="Normal module:    %.3f"         % normal_module)
        box.label(text="Sun peak twist:   %.1f °"       % degrees(peak_twist))
        box.label(text="Total slices:     %d"           % (2 * self.n_slices - 1))
        box.label(text="Ratio (sun/ring): 1 : %.2f"    % (ring_teeth / self.sun_teeth))
        hand_str = "R→L→L" if self.hand == 'RIGHT' else "L→R→R"
        box.label(text="Hands (sun/planet/ring): %s"   % hand_str)

        if not assembly_ok:
            layout.label(
                text="(%d + %d) / %d not integer — planets won't space evenly"
                     % (self.sun_teeth, ring_teeth, self.planet_count),
                icon='ERROR')
        layout.label(text="Max pressure angle for these teeth: %.1f°" % pa_max)

    def execute(self, context):
        gear_matching.clamp_pressure_angle(
            self,
            (self.sun_teeth, ADDENDUM_COEFF),
            (self.planet_teeth, ADDENDUM_COEFF),
            (self.sun_teeth + 2 * self.planet_teeth, DEDENDUM_COEFF),
        )
        (ring_teeth, ha_rad, r_sun, r_planet, r_ring,
         center_dist, outer_r, normal_module, peak_twist, assembly_ok, pa_max) = self._derived()

        # No popup here on purpose — the assembly-condition failure is already
        # flagged inline in draw()'s redo panel (bottom-left "ERROR" label),
        # which stays visible without an intrusive banner notification.

        cursor      = context.scene.cursor.location.copy()
        m           = self.module
        pa          = self.pressure_angle_deg
        w           = self.width_mm
        sun_sign    =  1.0 if self.hand == 'RIGHT' else -1.0
        planet_sign = -sun_sign
        ring_sign   =  planet_sign

        created = []

        # ── Ring (herringbone annulus) gear ────────────────────────────────────
        body = _make_cylinder_obj(
            context, outer_r, 0.0, w, self.outer_segs,
            "HbPlanetaryRingMesh", "HbPlanetaryRing"
        )
        body.location = cursor

        cutter_pts = _build_annulus_cutter_profile(m, ring_teeth, pa, self.pip_gap)
        cutter = _make_herringbone_cutter_obj(
            context, cutter_pts, w,
            ring_sign, ha_rad, r_ring, self.n_slices,
            "__HbPGSRingCutMesh", "__HbPGSRingCut"
        )
        cutter.location = cursor

        bpy.ops.object.select_all(action='DESELECT')
        body.select_set(True)
        context.view_layer.objects.active = body

        mod           = body.modifiers.new("RingBore", 'BOOLEAN')
        mod.operation = 'DIFFERENCE'
        mod.object    = cutter
        mod.solver    = 'EXACT'
        bpy.ops.object.modifier_apply(modifier="RingBore")
        bpy.data.objects.remove(cutter, do_unlink=True)

        # Cutter lobes at k·2π/N_ring → slots there, teeth at (2k+1)·π/N_ring.
        # −π/N_ring aligns a ring tooth with each planet's valley.
        body.rotation_euler.z = -pi / ring_teeth
        created.append(body)

        # ── Sun gear (herringbone) ─────────────────────────────────────────────
        sun_me  = _make_herringbone_mesh(m, self.sun_teeth, pa, w,
                                          sun_sign, ha_rad, self.n_slices,
                                          "HbPlanetarySunMesh", self.pip_gap)
        sun_obj = bpy.data.objects.new("HbPlanetarySun", sun_me)
        sun_obj.location = cursor
        # An odd planet tooth count puts the planet-roll formula's sun contact
        # exactly a half-tooth-pitch out of phase (an even count cancels this
        # by symmetry). Rotating the sun by pi/N_sun restores a valid mesh
        # without touching the planet/ring formulas at all.
        if self.planet_teeth % 2 == 1:
            sun_obj.rotation_euler.z = pi / self.sun_teeth
        context.collection.objects.link(sun_obj)
        created.append(sun_obj)

        # ── Planet gears (herringbone, shared mesh, N linked copies) ──────────
        planet_me  = _make_herringbone_mesh(m, self.planet_teeth, pa, w,
                                             planet_sign, ha_rad, self.n_slices,
                                             "HbPlanetaryPlanetMesh", self.pip_gap)
        angle_step = 2.0 * pi / self.planet_count

        for i in range(self.planet_count):
            theta      = i * angle_step
            planet_obj = bpy.data.objects.new(
                "HbPlanetaryPlanet.%03d" % (i + 1), planet_me)
            planet_obj.location = (
                cursor.x + center_dist * cos(theta),
                cursor.y + center_dist * sin(theta),
                cursor.z,
            )
            planet_obj.rotation_euler.z = (
                -theta * self.sun_teeth / self.planet_teeth + pi / self.planet_teeth
            )
            context.collection.objects.link(planet_obj)
            created.append(planet_obj)

        bpy.ops.object.select_all(action='DESELECT')
        for obj in created:
            obj.select_set(True)
        context.view_layer.objects.active = created[0]

        self.report({'INFO'},
            "Herringbone planetary: %d/%d/%d teeth (sun/planet/ring), %.1f° helix, %d planets"
            % (self.sun_teeth, self.planet_teeth, ring_teeth,
               self.helix_angle_deg, self.planet_count))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_herringbone_planetary_gear_set,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
