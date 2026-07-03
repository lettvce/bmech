"""
Planetary Gear Set Generator

A planetary (epicyclic) gear set has three coaxial members:
  Sun gear    — central external gear
  Planet gears — N external gears that orbit between sun and ring
  Ring gear   — internal (annulus) gear surrounding the planets

Tooth count rule (must hold exactly):
  N_ring = N_sun + 2 × N_planet

Equal-spacing assembly condition (planets must fit symmetrically):
  (N_sun + N_ring) % N_planets == 0

Planet mesh rotation (external–external with sun, external–internal with ring):
  φ_i = −θ_i × (N_sun / N_planet) + π/N_planet

Ring phase correction:
  ring.rotation_euler.z = −π / N_ring

Build method:
  Sun and planet bodies: direct bmesh extrusion (no boolean needed).
  Ring body: solid cylinder + boolean DIFFERENCE with cutter profile.
  Planet gears share one mesh data-block (linked copies).
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
    """External involute gear profile (2D, CCW, for sun/planet bodies)."""
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    # pip_gap thins the tooth at the pitch circle → angular backlash at each mesh interface
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
    # pip_gap widens the cutter slot → thins the ring tooth → angular backlash
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


def _make_spur_mesh(module, tooth_count, pa_deg, width_mm, mesh_name, pip_gap=0.0):
    """External spur gear mesh data (no object created yet)."""
    profile = _build_gear_profile(module, tooth_count, pa_deg, pip_gap)
    n       = len(profile)

    bm  = bmesh.new()
    bot = [bm.verts.new((x, y, 0.0))      for x, y in profile]
    top = [bm.verts.new((x, y, width_mm)) for x, y in profile]
    bm.verts.index_update()

    bm.faces.new(list(reversed(bot)))
    bm.faces.new(top)
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


def _make_cutter_obj(context, profile_pts, z_bot, z_top, mesh_name, obj_name):
    """Prism cutter solid with center-fan caps (safe for EXACT boolean solver)."""
    bm = bmesh.new()
    n  = len(profile_pts)

    bot   = [bm.verts.new((x, y, z_bot)) for x, y in profile_pts]
    top   = [bm.verts.new((x, y, z_top)) for x, y in profile_pts]
    c_bot = bm.verts.new((0.0, 0.0, z_bot))
    c_top = bm.verts.new((0.0, 0.0, z_top))
    bm.verts.index_update()

    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([bot[i], bot[ni], top[ni], top[i]])
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_bot, bot[ni], bot[i]])
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([c_top, top[i], top[ni]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(mesh_name)
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj = bpy.data.objects.new(obj_name, me)
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_planetary_gear_set(bpy.types.Operator):
    """Planetary gear set — sun, planets, and ring gear, correctly meshed."""
    bl_idname  = "object.planetary_gear_set"
    bl_label   = "Planetary Gear Set"
    bl_options = {'REGISTER', 'UNDO'}

    sun_teeth:          IntProperty(  name="Sun Teeth",          default=12,   min=4,    soft_max=100,
                                      description="Tooth count of the central sun gear")
    planet_teeth:       IntProperty(  name="Planet Teeth",       default=18,   min=4,    soft_max=100,
                                      description="Tooth count of each planet gear (ring = sun + 2×planet)")
    planet_count:       IntProperty(  name="Planet Count",       default=3,    min=2,    max=8,
                                      description="Number of planet gears — (sun+ring) must be divisible by this")
    module:             FloatProperty(name="Module (mm)",         default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Common module for all gears")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    width_mm:           FloatProperty(name="Width (mm)",          default=10.0, min=1.0,  soft_max=80.0,
                                      description="Face width — same for all gears")
    ring_wall_mm:       FloatProperty(name="Ring Wall (mm)",      default=5.0,  min=0.5,  soft_max=30.0,
                                      description="Radial wall of ring gear beyond the tooth root")
    pip_gap:            FloatProperty(name="PiP Gap (mm)",        default=0.2,  min=0.0,  soft_max=2.0,
                                      description="Radial clearance at tooth tips for print-in-place")
    outer_segs:         IntProperty(  name="Outer Segments",     default=64,   min=16,   soft_max=256,
                                      description="Facets on the ring gear's outer cylindrical surface")

    def _derived(self):
        ring_teeth  = self.sun_teeth + 2 * self.planet_teeth
        r_sun       = self.module * self.sun_teeth    / 2.0
        r_planet    = self.module * self.planet_teeth / 2.0
        r_ring      = self.module * ring_teeth        / 2.0
        center_dist = r_sun + r_planet
        outer_r     = r_ring + DEDENDUM_COEFF * self.module + self.ring_wall_mm
        assembly_ok = (self.sun_teeth + ring_teeth) % self.planet_count == 0
        pa_max = min(
            gear_matching.max_pressure_angle_deg(self.sun_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(self.planet_teeth, ADDENDUM_COEFF),
            gear_matching.max_pressure_angle_deg(ring_teeth, DEDENDUM_COEFF),
        )
        return ring_teeth, r_sun, r_planet, r_ring, center_dist, outer_r, assembly_ok, pa_max

    def draw(self, context):
        layout = self.layout
        ring_teeth, r_sun, r_planet, r_ring, center_dist, outer_r, assembly_ok, pa_max = self._derived()

        col = layout.column(align=True)
        col.prop(self, "sun_teeth")
        col.prop(self, "planet_teeth")
        col.prop(self, "planet_count")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "width_mm")
        col.prop(self, "ring_wall_mm")
        col.prop(self, "pip_gap")
        col.prop(self, "outer_segs")

        layout.separator()
        box = layout.box()
        box.label(text="Ring teeth:       %d"           % ring_teeth)
        box.label(text="Sun pitch Ø:      %.2f mm"      % (r_sun    * 2))
        box.label(text="Planet pitch Ø:   %.2f mm"      % (r_planet * 2))
        box.label(text="Ring pitch Ø:     %.2f mm"      % (r_ring   * 2))
        box.label(text="Center dist:      %.2f mm"      % center_dist)
        box.label(text="Outer Ø:          %.2f mm"      % (outer_r  * 2))
        box.label(text="Ratio (sun/ring): 1 : %.2f"    % (ring_teeth / self.sun_teeth))

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
        ring_teeth, r_sun, r_planet, r_ring, center_dist, outer_r, assembly_ok, pa_max = self._derived()

        # No popup here on purpose — the assembly-condition failure is already
        # flagged inline in draw()'s redo panel (bottom-left "ERROR" label),
        # which stays visible without an intrusive banner notification.

        cursor = context.scene.cursor.location.copy()
        m      = self.module
        pa     = self.pressure_angle_deg
        w      = self.width_mm

        created = []

        # ── Ring gear ──────────────────────────────────────────────────────────
        body = _make_cylinder_obj(
            context, outer_r, 0.0, w, self.outer_segs,
            "PlanetaryRingMesh", "PlanetaryRing"
        )
        body.location = cursor

        cutter_pts = _build_annulus_cutter_profile(m, ring_teeth, pa, self.pip_gap)
        cutter = _make_cutter_obj(
            context, cutter_pts, -BOOL_EPSILON, w + BOOL_EPSILON,
            "__PGSRingCutMesh", "__PGSRingCut"
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

        # Cutter lobes sit at k·2π/N_ring → ring slots there, teeth at (2k+1)·π/N_ring.
        # Planet roll formula puts a valley at each ring-contact; −π/N_ring aligns a tooth.
        body.rotation_euler.z = -pi / ring_teeth
        created.append(body)

        # ── Sun gear ───────────────────────────────────────────────────────────
        sun_me  = _make_spur_mesh(m, self.sun_teeth, pa, w, "PlanetarySunMesh", self.pip_gap)
        sun_obj = bpy.data.objects.new("PlanetarySun", sun_me)
        sun_obj.location = cursor
        # An odd planet tooth count puts the planet-roll formula's sun contact
        # exactly a half-tooth-pitch out of phase (an even count cancels this
        # by symmetry). Rotating the sun by pi/N_sun restores a valid mesh
        # without touching the planet/ring formulas at all.
        if self.planet_teeth % 2 == 1:
            sun_obj.rotation_euler.z = pi / self.sun_teeth
        context.collection.objects.link(sun_obj)
        created.append(sun_obj)

        # ── Planet gears (shared mesh, N linked copies) ────────────────────────
        planet_me  = _make_spur_mesh(m, self.planet_teeth, pa, w, "PlanetaryPlanetMesh", self.pip_gap)
        angle_step = 2.0 * pi / self.planet_count

        for i in range(self.planet_count):
            theta      = i * angle_step
            planet_obj = bpy.data.objects.new("PlanetaryPlanet.%03d" % (i + 1), planet_me)
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
            "Planetary: %d/%d/%d teeth (sun/planet/ring), module %.1f, %d planets"
            % (self.sun_teeth, self.planet_teeth, ring_teeth, m, self.planet_count))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_planetary_gear_set,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
