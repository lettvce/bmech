"""
Bevel Gear Generator

Straight-tooth bevel gear using Tredgold's approximation:
  The tooth cross-section at each z-slice is the standard involute profile
  scaled by (cone_dist − slant_pos) / cone_dist.  Two gears whose mate_teeth
  values are swapped mesh at 90° with their apices coincident.

Geometry (90° shaft angle):
  cone_angle δ = atan(N / N_mate)
  pitch_r      = m × N / 2              (at large end / back face)
  cone_dist L  = pitch_r / sin(δ)       (apex-to-back-face slant distance)
  z_apex       = L × cos(δ)            = pitch_r × N_mate / N
  z_top        = face_width × cos(δ)   (axial height of small end)
  scale_top    = (L − face_width) / L  (uniform scale at small end)

Build method:
  n_slices axial cross-sections from z=0 (large end) to z_top (small end).
  Each slice scales the full involute profile by (z_apex − z) / z_apex.
  Both end faces closed with center-fan triangulation.
  No boolean needed — the gear body under the root circle is covered by the
  center-fan fill of the back and front faces.

Note: face_width ≤ L/3 is the standard engineering limit.
To make the mating gear: swap tooth_count ↔ mate_teeth (δ₂ = 90° − δ₁).
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, sqrt, radians, degrees, atan2, atan
from .. import gear_matching

INVOLUTE_POINTS = 15
ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
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


def _build_gear_profile(module, tooth_count, pa_deg):
    """Full-size involute gear cross-section (2D, CCW) at the large end."""
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count)
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


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_bevel_gear(bpy.types.Operator):
    """Straight-tooth bevel gear — toothed frustum via Tredgold's approximation."""
    bl_idname  = "object.bevel_gear"
    bl_label   = "Bevel Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_bevel(self, context.window_manager.bmech_gear_target)

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    tooth_count:        IntProperty(  name="Tooth Count",        default=16,   min=8,    soft_max=120,
                                      description="Tooth count of this gear")
    mate_teeth:         IntProperty(  name="Mate Teeth",         default=16,   min=8,    soft_max=120,
                                      description="Tooth count of the mating gear — sets the pitch cone angle")
    module:             FloatProperty(name="Module (mm)",         default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Module at the large end (back face)")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    face_width_mm:      FloatProperty(name="Face Width (mm)",    default=8.0,  min=1.0,  soft_max=60.0,
                                      description="Tooth length along the cone slant — keep ≤ cone distance / 3")
    n_slices:           IntProperty(  name="Slices",              default=12,   min=2,    soft_max=64,
                                      description="Axial divisions — more gives a smoother taper")

    def _derived(self):
        N   = self.tooth_count
        Nm  = self.mate_teeth
        m   = self.module
        b   = self.face_width_mm
        delta     = atan(N / Nm)
        pitch_r   = m * N / 2.0
        cone_dist = pitch_r / sin(delta)
        z_apex    = cone_dist * cos(delta)
        z_top     = b * cos(delta)
        scale_top = (cone_dist - b) / cone_dist
        add_r     = pitch_r + ADDENDUM_COEFF * m
        ded_r     = pitch_r - DEDENDUM_COEFF * m
        pa_max    = gear_matching.max_pressure_angle_deg(N, ADDENDUM_COEFF)
        return (delta, pitch_r, cone_dist, z_apex, z_top,
                scale_top, add_r, ded_r,
                b > cone_dist / 3.0,
                b >= cone_dist,
                pa_max)

    def draw(self, context):
        layout = self.layout
        (delta, pitch_r, cone_dist, z_apex, z_top,
         scale_top, add_r, ded_r, over_limit, too_long, pa_max) = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        col = layout.column(align=True)
        col.prop(self, "tooth_count")
        col.prop(self, "mate_teeth")
        col.prop(self, "module")
        col.prop(self, "pressure_angle_deg")
        col.prop(self, "face_width_mm")
        col.prop(self, "n_slices")

        layout.separator()
        box = layout.box()
        box.label(text="Cone angle:       %.2f °"  % degrees(delta))
        box.label(text="Mate cone angle:  %.2f °"  % (90.0 - degrees(delta)))
        box.label(text="Pitch Ø (large):  %.2f mm" % (pitch_r  * 2.0))
        box.label(text="Tip Ø   (large):  %.2f mm" % (add_r    * 2.0))
        box.label(text="Root Ø  (large):  %.2f mm" % (ded_r    * 2.0))
        box.label(text="Cone distance:    %.2f mm" % cone_dist)
        box.label(text="Height (z_top):   %.2f mm" % z_top)
        box.label(text="Scale at tip:     %.3f"    % max(scale_top, 0.0))
        box.label(text="Pitch Ø (small):  %.2f mm" % (pitch_r * max(scale_top, 0.0) * 2.0))
        box.label(text="Ratio:            %d : %d" % (self.tooth_count, self.mate_teeth))
        box.label(text="Mate: swap tooth_count ↔ mate_teeth")

        if too_long:
            layout.label(text="Face width ≥ cone distance — gear vanishes", icon='ERROR')
        elif over_limit:
            layout.label(text="Face width > L/3 — small-end teeth very small", icon='ERROR')
        layout.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, ADDENDUM_COEFF))
        (delta, pitch_r, cone_dist, z_apex, z_top,
         scale_top, add_r, ded_r, over_limit, too_long, pa_max) = self._derived()

        if too_long:
            self.report({'ERROR'}, "Face width ≥ cone distance — nothing to build")
            return {'CANCELLED'}
        if over_limit:
            self.report({'WARNING'}, "Face width > cone distance / 3")

        profile = _build_gear_profile(self.module, self.tooth_count, self.pressure_angle_deg)
        n  = len(profile)
        ns = self.n_slices

        bm = bmesh.new()

        all_slices = []
        for k in range(ns):
            t     = k / (ns - 1)
            z     = t * z_top
            scale = (z_apex - z) / z_apex
            all_slices.append(
                [bm.verts.new((x * scale, y * scale, z)) for x, y in profile]
            )

        bm.verts.index_update()

        for k in range(ns - 1):
            bot = all_slices[k]
            top = all_slices[k + 1]
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

        c_bot = bm.verts.new((0.0, 0.0, 0.0))
        bm.verts.index_update()
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([c_bot, all_slices[0][i], all_slices[0][ni]])

        c_top = bm.verts.new((0.0, 0.0, z_top))
        bm.verts.index_update()
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([c_top, all_slices[-1][ni], all_slices[-1][i]])

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        me = bpy.data.meshes.new("BevelGearMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        cursor = context.scene.cursor.location.copy()
        obj    = bpy.data.objects.new("BevelGear", me)
        obj.location = cursor
        context.collection.objects.link(obj)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        gear_matching.stamp_gear(obj, "bevel", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count)

        self.report({'INFO'},
            "Bevel gear: %d/%d teeth, δ=%.1f°, module %.1f, face %.1f mm"
            % (self.tooth_count, self.mate_teeth,
               degrees(delta), self.module, self.face_width_mm))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_bevel_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
