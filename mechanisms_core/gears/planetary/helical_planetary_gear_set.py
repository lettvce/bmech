"""
Helical Planetary Gear Set Generator

Same topology as planetary_gear_set.py but all three members have helical teeth.

Hand convention:
  Sun (external) ↔ Planet (external) : OPPOSITE hands  (external-external rule)
  Planet (external) ↔ Ring (internal): SAME hand       (external-internal rule)
  ∴ Ring = opposite of Sun, same as Planet.

  "Hand" parameter selects the SUN. Planets and ring are derived automatically.

Tooth count rule (same as spur):
  N_ring = N_sun + 2 × N_planet
  Assembly condition: (N_sun + N_ring) % N_planets == 0

Planet placement (same roll formula as spur — helix doesn't change z=0 phase):
  φ_i = −θ_i × (N_sun / N_planet) + π/N_planet

Ring phase correction (same derivation as spur):
  ring.rotation_euler.z = −π / N_ring

Build method:
  Sun / Planet : direct bmesh slice-by-slice twist (no boolean needed)
  Ring         : solid cylinder + boolean DIFFERENCE with helical cutter
  Planet mesh is shared across all N planet objects.
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


def _make_helical_mesh(module, tooth_count, pa_deg, width_mm,
                        hand_sign, ha_rad, n_slices, mesh_name, pip_gap=0.0):
    """Helical external gear mesh data (no object created yet)."""
    profile = _build_gear_profile(module, tooth_count, pa_deg, pip_gap)
    pitch_r = module * tooth_count / 2.0
    n       = len(profile)

    bm     = bmesh.new()
    slices = []
    for k in range(n_slices):
        z     = width_mm * k / (n_slices - 1)
        twist = hand_sign * z * tan(ha_rad) / pitch_r
        c, s  = cos(twist), sin(twist)
        slices.append([bm.verts.new((x * c - y * s, x * s + y * c, z))
                       for x, y in profile])

    bm.verts.index_update()
    bm.faces.new(list(reversed(slices[0])))
    bm.faces.new(slices[-1])
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


def _make_helical_cutter_obj(context, cutter_profile, width_mm,
                               hand_sign, ha_rad, pitch_r,
                               n_slices, mesh_name, obj_name):
    """Helical cutter prism for ring boolean (z extended by ±eps, center-fan caps)."""
    n       = len(cutter_profile)
    z_bot   = -BOOL_EPSILON
    z_top   = width_mm + BOOL_EPSILON
    total_h = z_top - z_bot

    bm     = bmesh.new()
    slices = []
    for k in range(n_slices):
        z     = z_bot + total_h * k / (n_slices - 1)
        twist = hand_sign * (z - z_bot) * tan(ha_rad) / pitch_r
        c, s  = cos(twist), sin(twist)
        slices.append([bm.verts.new((x * c - y * s, x * s + y * c, z))
                       for x, y in cutter_profile])

    c_bot = bm.verts.new((0.0, 0.0, z_bot))
    c_top = bm.verts.new((0.0, 0.0, z_top))
    bm.verts.index_update()

    for k in range(n_slices - 1):
        bot = slices[k]
        top = slices[k + 1]
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_bot, slices[0][ni], slices[0][i]])
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_top, slices[-1][i], slices[-1][ni]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(obj_name, me)
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_helical_planetary_gear_set(bpy.types.Operator):
    """Helical planetary gear set — sun, planets, and ring with twisted teeth."""
    bl_idname  = "object.helical_planetary_gear_set"
    bl_label   = "Helical Planetary Gear Set"
    bl_options = {'REGISTER', 'UNDO'}

    sun_teeth:          IntProperty(  name="Sun Teeth",          default=12,   min=4,    soft_max=100)
    planet_teeth:       IntProperty(  name="Planet Teeth",       default=18,   min=4,    soft_max=100,
                                      description="Ring = Sun + 2 × Planet (auto-derived)")
    planet_count:       IntProperty(  name="Planet Count",       default=3,    min=2,    max=8,
                                      description="(Sun + Ring) must be divisible by this")
    module:             FloatProperty(name="Module (mm)",         default=2.0,  min=0.1,  soft_max=20.0)
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    helix_angle_deg:    FloatProperty(name="Helix Angle (°)",    default=20.0, min=1.0,  max=45.0,
                                      description="Helix angle for all three members")
    hand: EnumProperty(
        name="Sun Hand",
        items=[('RIGHT', "Right (sun)", "Sun RH → planets LH → ring LH"),
               ('LEFT',  "Left (sun)",  "Sun LH → planets RH → ring RH")],
        default='RIGHT',
        description="Sun gear hand — planets and ring are derived automatically",
    )
    width_mm:           FloatProperty(name="Width (mm)",          default=10.0, min=1.0,  soft_max=80.0)
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",      default=5.0,  min=0.5,  soft_max=30.0)
    pip_gap:            FloatProperty(name="PiP Gap (mm)",        default=0.2,  min=0.0,  soft_max=2.0,
                                      description="Radial clearance at tooth tips for print-in-place")
    n_slices:           IntProperty(  name="Slices",              default=16,   min=2,    soft_max=64,
                                      description="Z divisions for helical twist — applies to all members")
    outer_segs:         IntProperty(  name="Outer Segments",      default=64,   min=16,   soft_max=256)

    def _derived(self):
        ring_teeth    = self.sun_teeth + 2 * self.planet_teeth
        ha_rad        = radians(self.helix_angle_deg)
        r_sun         = self.module * self.sun_teeth    / 2.0
        r_planet      = self.module * self.planet_teeth / 2.0
        r_ring        = self.module * ring_teeth        / 2.0
        center_dist   = r_sun + r_planet
        outer_r       = r_ring + DEDENDUM_COEFF * self.module + self.ring_wall_mm
        normal_module = self.module * cos(ha_rad)
        assembly_ok   = (self.sun_teeth + ring_teeth) % self.planet_count == 0
        sun_twist_deg = degrees(self.width_mm * tan(ha_rad) / r_sun)
        pa_max = min(
            gear_matching.max_pressure_angle_deg(self.sun_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(self.planet_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(ring_teeth, DEDENDUM_COEFF),
        )
        return (ring_teeth, ha_rad, r_sun, r_planet, r_ring,
                center_dist, outer_r, normal_module, assembly_ok, sun_twist_deg, pa_max)

    def draw(self, context):
        layout = self.layout
        (ring_teeth, ha_rad, r_sun, r_planet, r_ring,
         center_dist, outer_r, normal_module, assembly_ok, sun_twist_deg, pa_max) = self._derived()

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
        box.label(text="Sun total twist:  %.1f °"       % sun_twist_deg)
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
         center_dist, outer_r, normal_module, assembly_ok, _, pa_max) = self._derived()

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

        # ── Ring (helical annulus) gear ───────────────────────────────────────
        body = _make_cylinder_obj(
            context, outer_r, 0.0, w, self.outer_segs,
            "HelPlanetaryRingMesh", "HelPlanetaryRing"
        )
        body.location = cursor

        cutter_pts = _build_annulus_cutter_profile(m, ring_teeth, pa, self.pip_gap)
        cutter = _make_helical_cutter_obj(
            context, cutter_pts, w,
            ring_sign, ha_rad, r_ring, self.n_slices,
            "__HPGSRingCutMesh", "__HPGSRingCut"
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

        # Slot centres at k·2π/N_ring → teeth at (2k+1)·π/N_ring.
        # −π/N_ring aligns a ring tooth with each planet's valley.
        body.rotation_euler.z = -pi / ring_teeth
        created.append(body)

        # ── Sun gear (helical) ────────────────────────────────────────────────
        sun_me  = _make_helical_mesh(m, self.sun_teeth, pa, w,
                                      sun_sign, ha_rad, self.n_slices,
                                      "HelPlanetarySunMesh", self.pip_gap)
        sun_obj = bpy.data.objects.new("HelPlanetarySun", sun_me)
        sun_obj.location = cursor
        # An odd planet tooth count puts the planet-roll formula's sun contact
        # exactly a half-tooth-pitch out of phase (an even count cancels this
        # by symmetry). Rotating the sun by pi/N_sun restores a valid mesh
        # without touching the planet/ring formulas at all.
        if self.planet_teeth % 2 == 1:
            sun_obj.rotation_euler.z = pi / self.sun_teeth
        context.collection.objects.link(sun_obj)
        created.append(sun_obj)

        # ── Planet gears (helical, shared mesh, N linked copies) ──────────────
        planet_me  = _make_helical_mesh(m, self.planet_teeth, pa, w,
                                         planet_sign, ha_rad, self.n_slices,
                                         "HelPlanetaryPlanetMesh", self.pip_gap)
        angle_step = 2.0 * pi / self.planet_count

        for i in range(self.planet_count):
            theta      = i * angle_step
            planet_obj = bpy.data.objects.new("HelPlanetaryPlanet.%03d" % (i + 1), planet_me)
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
            "Helical planetary: %d/%d/%d teeth (sun/planet/ring), %.1f° helix, %d planets"
            % (self.sun_teeth, self.planet_teeth, ring_teeth,
               self.helix_angle_deg, self.planet_count))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_helical_planetary_gear_set,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
