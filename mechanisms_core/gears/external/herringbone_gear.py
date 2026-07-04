"""
Herringbone Gear Generator
──────────────────────────
A herringbone gear is two helical halves mirrored at mid-height. The V-shaped
tooth cancels axial thrust, so no thrust bearings are needed. The helix angle
applies to each half independently.

Tooth geometry:
  Bottom half  Z=0 → Z=width/2  : twist rises  0 → peak
  Top half     Z=width/2 → Z=width: twist falls peak → 0
  The peak twist = (width/2) × tan(helix_angle) / pitch_radius

Meshing rules (same as helical):
  • Same module, same pressure angle
  • Opposite hand — right meshes with left
  • Helix angle 15–30° typical for FDM

Hand note:
  For herringbone there is no net axial force regardless of hand, but hand
  still determines which way the V opens and must be matched to the mating gear.

Normal module:
  mn = module × cos(helix_angle)
  The module here is the TRANSVERSE module.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, BoolProperty, EnumProperty
from math import pi, cos, sin, tan, sqrt, radians, degrees, atan2
from .. import gear_matching

# ── Involute math ─────────────────────────────────────────────────────────────

INVOLUTE_POINTS = 15
ADDENDUM_COEFF  = 1.0
DEDENDUM_COEFF  = 1.25
BOOL_EPSILON    = 0.001
BORE_SEGS       = 32


def _involute_pt(base_r, t):
    return (base_r * (cos(t) + t * sin(t)),
            base_r * (sin(t) - t * cos(t)))


def _involute_t_at_r(base_r, r):
    ratio = r / base_r
    return 0.0 if ratio < 1.0 else sqrt(ratio * ratio - 1.0)


def _rot(x, y, a):
    c, s = cos(a), sin(a)
    return (x * c - y * s, x * s + y * c)


def _build_tooth_profile(module, tooth_count, pa_deg):
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    add_r          = pitch_r + ADDENDUM_COEFF * module
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count)

    t_start = _involute_t_at_r(base_r, max(ded_r, base_r))
    t_tip   = _involute_t_at_r(base_r, add_r)
    raw     = [_involute_pt(base_r, t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1))
               for i in range(INVOLUTE_POINTS)]

    t_pitch = _involute_t_at_r(base_r, pitch_r)
    px, py  = _involute_pt(base_r, t_pitch)
    rot     = half_tooth_ang + atan2(py, px)
    right   = [_rot(x, -y, rot) for x, y in raw]

    if base_r > ded_r:
        ra    = atan2(right[0][1], right[0][0])
        right = [(ded_r * cos(ra), ded_r * sin(ra))] + right

    left = [(x, -y) for x, y in right]
    return left + [(add_r, 0.0)] + right[::-1]


def _build_gear_profile(module, tooth_count, pa_deg):
    pa_rad         = radians(pa_deg)
    pitch_r        = module * tooth_count / 2.0
    base_r         = pitch_r * cos(pa_rad)
    ded_r          = pitch_r - DEDENDUM_COEFF * module
    half_tooth_ang = pi / (2.0 * tooth_count)
    pitch_arc      = 2.0 * pi / tooth_count

    t_pitch      = _involute_t_at_r(base_r, pitch_r)
    t_start      = _involute_t_at_r(base_r, max(ded_r, base_r))
    px, py       = _involute_pt(base_r, t_pitch)
    rot          = half_tooth_ang + atan2(py, px)
    root_pt      = _involute_pt(base_r, t_start)
    root_rot     = _rot(root_pt[0], -root_pt[1], rot)
    root_r_local = atan2(root_rot[1], root_rot[0])

    tooth_pts = _build_tooth_profile(module, tooth_count, pa_deg)
    SPACE_PTS = 4

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


def _apply_bore(context, obj, bore_r, total_width):
    bm     = bmesh.new()
    angles = [2.0 * pi * i / BORE_SEGS for i in range(BORE_SEGS)]
    z0, z1 = -BOOL_EPSILON, total_width + BOOL_EPSILON

    vb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angles]
    vt = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angles]
    bm.verts.index_update()

    for i in range(BORE_SEGS):
        ni = (i + 1) % BORE_SEGS
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])
    bm.faces.new(vb)
    bm.faces.new(list(reversed(vt)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me_cut = bpy.data.meshes.new("__HBBoreMesh")
    bm.to_mesh(me_cut)
    bm.free()
    me_cut.update()

    cutter = bpy.data.objects.new("__HBBore", me_cut)
    cutter.location = obj.location.copy()
    context.collection.objects.link(cutter)

    mod           = obj.modifiers.new("Bore", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object    = cutter
    mod.solver    = 'EXACT'

    # modifier_apply's poll requires the active object to also be the ONLY
    # selected one — temp_override(active_object=obj) alone doesn't select
    # it, so if a previously-created gear is still selected from an earlier
    # call, the poll silently fails (returns CANCELLED, not an exception)
    # and the bore modifier is left un-applied, then the cutter gets
    # deleted below anyway, leaving the modifier pointing at nothing (no
    # visible bore at all). Deselect everything and select obj explicitly
    # first, same pattern as hex_bolt.py/hex_nut.py's _bool_diff.
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    context.view_layer.objects.active = obj
    with context.temp_override(active_object=obj):
        bpy.ops.object.modifier_apply(modifier="Bore")

    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_herringbone_gear(bpy.types.Operator):
    """Herringbone involute gear — V-shaped teeth cancel axial thrust."""
    bl_idname  = "object.herringbone_gear"
    bl_label   = "Herringbone Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_helical_opposite(self, context.window_manager.bmech_gear_target)

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    tooth_count:        IntProperty(  name="Tooth Count",        default=20,   min=5,    soft_max=200)
    module:             FloatProperty(name="Module (mm)",         default=2.0,  min=0.1,  soft_max=20.0,
                                      description="Transverse module — sets pitch circle size")
    pressure_angle_deg: FloatProperty(name="Pressure Angle (°)", default=20.0, min=10.0, max=45.0)
    helix_angle_deg:    FloatProperty(name="Helix Angle (°)",    default=20.0, min=1.0,  max=45.0,
                                      description="Half-angle of the V — 15–30° typical for FDM")
    hand: EnumProperty(
        name="Hand",
        items=[('RIGHT', "Right-hand", "Bottom half twists CW from below"),
               ('LEFT',  "Left-hand",  "Bottom half twists CCW from below")],
        default='RIGHT',
        description="Mating herringbone gears must have opposite hands",
    )
    width_mm:          FloatProperty(name="Total Width (mm)",    default=14.0, min=2.0,  soft_max=80.0,
                                      description="Full face width — each half is width/2")
    bore_enable:       BoolProperty( name="Bore Hole",           default=True)
    bore_diameter:     FloatProperty(name="Bore Ø (mm)",         default=5.0,  min=0.1,  soft_max=50.0)
    bore_compensation: FloatProperty(name="Compensation (mm)",   default=0.2,  min=0.0,  soft_max=1.0,
                                      description="FDM holes print tight — added to bore radius")
    n_slices:          IntProperty(  name="Slices per Half",     default=12,   min=2,    soft_max=48,
                                      description="Z divisions per half — total slices = 2n−1")

    def _derived(self):
        ha_rad        = radians(self.helix_angle_deg)
        pitch_r       = self.module * self.tooth_count / 2.0
        ded_r         = pitch_r - DEDENDUM_COEFF * self.module
        add_r         = pitch_r + ADDENDUM_COEFF * self.module
        half_h        = self.width_mm / 2.0
        peak_twist    = half_h * tan(ha_rad) / pitch_r
        normal_module = self.module * cos(ha_rad)
        bore_r        = (self.bore_diameter / 2.0 + self.bore_compensation) if self.bore_enable else 0.0
        pa_max        = gear_matching.max_pressure_angle_deg(self.tooth_count, ADDENDUM_COEFF)
        return ha_rad, pitch_r, ded_r, add_r, half_h, peak_twist, normal_module, bore_r, pa_max

    def draw(self, context):
        layout = self.layout
        ha_rad, pitch_r, ded_r, add_r, half_h, peak_twist, normal_module, bore_r, pa_max = self._derived()

        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        target = context.window_manager.bmech_gear_target
        has_target = target is not None
        # A spur-gear target doesn't stamp bmech_helix_angle_deg/bmech_hand
        # (see gear_matching.sync_helical_opposite and stamp_gear), so it
        # never drives helix_angle_deg/hand — those stay editable even with
        # a target set, since the match there is only "shares a pitch
        # circle" (compound-gear hub), not "these teeth mesh directly".
        # module/pressure_angle_deg ARE always driven by any target kind.
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
        col.prop(self, "n_slices")
        layout.prop(self, "bore_enable")
        if self.bore_enable:
            sub = layout.column(align=True)
            sub.prop(self, "bore_diameter")
            sub.prop(self, "bore_compensation")

        layout.separator()
        box = layout.box()
        box.label(text="Pitch Ø:       %.2f mm"  % (pitch_r * 2))
        box.label(text="Outer Ø:       %.2f mm"  % (add_r * 2))
        box.label(text="Half width:    %.2f mm"  % half_h)
        box.label(text="Normal module: %.3f"     % normal_module)
        box.label(text="Peak twist:    %.1f °"   % degrees(peak_twist))
        box.label(text="Total slices:  %d"       % (2 * self.n_slices - 1))

        if ded_r <= 0:
            layout.label(text="Module too large — dedendum radius ≤ 0", icon='ERROR')
        if bore_r > 0 and bore_r >= ded_r:
            layout.label(text="Bore larger than dedendum radius", icon='ERROR')
        layout.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, ADDENDUM_COEFF))
        ha_rad, pitch_r, ded_r, add_r, half_h, peak_twist, _, bore_r, pa_max = self._derived()

        if ded_r <= 0:
            return {'CANCELLED'}

        base_profile = _build_gear_profile(self.module, self.tooth_count, self.pressure_angle_deg)
        hand_sign    = 1.0 if self.hand == 'RIGHT' else -1.0
        n            = len(base_profile)

        bm = bmesh.new()

        def _make_slice(z, twist):
            c, s = cos(twist), sin(twist)
            return [bm.verts.new((x * c - y * s, x * s + y * c, z))
                    for x, y in base_profile]

        all_slices = []
        for k in range(self.n_slices):
            z     = half_h * k / (self.n_slices - 1)
            twist = hand_sign * z * tan(ha_rad) / pitch_r
            all_slices.append(_make_slice(z, twist))

        for k in range(1, self.n_slices):
            z     = half_h + half_h * k / (self.n_slices - 1)
            twist = hand_sign * (self.width_mm - z) * tan(ha_rad) / pitch_r
            all_slices.append(_make_slice(z, twist))

        bm.verts.index_update()

        # Cap the top/bottom with a single n-gon over the whole profile ring,
        # not a triangle fan to a center vertex. The tooth profile inserts a
        # dedendum-circle point at the SAME angle as the adjacent involute
        # departure point wherever base_r > ded_r (see _build_tooth_profile),
        # so two adjacent ring points can be exactly collinear with the
        # origin — a fan triangle to the center then has exactly zero area.
        # A single n-gon face has no center vertex and no per-point
        # triangles, so this class of degenerate face can't occur; this
        # matches how involute_gear_rack.py's profile_to_mesh_object and
        # helical_gear.py cap their own profiles.
        bm.faces.new(all_slices[0])
        bm.faces.new(list(reversed(all_slices[-1])))

        for k in range(len(all_slices) - 1):
            bot = all_slices[k]
            top = all_slices[k + 1]
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        me = bpy.data.meshes.new("HerringboneGearMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj          = bpy.data.objects.new("HerringboneGear", me)
        obj.location = context.scene.cursor.location.copy()
        context.collection.objects.link(obj)

        if bore_r > 0 and bore_r < ded_r:
            _apply_bore(context, obj, bore_r, self.width_mm)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        gear_matching.stamp_gear(obj, "herringbone", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count,
                                  helix_angle_deg=self.helix_angle_deg, hand=self.hand)

        self.report({'INFO'},
            "Herringbone: %d teeth, %.1f° helix, %.1f mm wide"
            % (self.tooth_count, self.helix_angle_deg, self.width_mm))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_herringbone_gear,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
