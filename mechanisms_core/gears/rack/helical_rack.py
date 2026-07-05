"""
Helical Rack Generator
───────────────────────
A helical rack is a straight rack whose teeth are sheared along X as a
linear function of Z, mirroring how a helical gear's teeth twist along its
axis — the rack is the gear-of-infinite-radius limit of that twist.

For a helical gear, twist_angle(z) = z * tan(helix_angle) / pitch_radius,
and the tooth-profile displacement at radius r is r * twist_angle(z). As
pitch_radius → ∞ (the rack limit), the arc-length displacement of any
profile point converges to a RADIUS-INDEPENDENT constant:
    shear(z) = z * tan(helix_angle)
every point of the profile shifts by the same amount in X at a given Z —
a pure shear, not a rotation.

[NOTE] Because shear(z) is LINEAR in z, the transform (x, y, z) ->
(x + shear(z), y, z) is an AFFINE map of 3D space. A straight edge in 3D
between the bottom-layer vertex (x, y, 0) and the top-layer vertex
(x + shear(width), y, width) passes exactly through (x + shear(z), y, z)
for every z in between (parametrize by t = z/width and shear(z) = t *
shear(width) — both sides agree). Unlike helical_gear.py/
herringbone_gear.py, which twist by ROTATING the profile — a nonlinear
function of z that needs many Z slices to approximate the curved helical
surface — this shear needs exactly TWO layers (bottom and top) to
represent the surface exactly. There is deliberately no n_slices property
here; adding one would suggest a faceting approximation this generator
doesn't have.

Meshing rules — NOT the same as two external helical gears:
  - Same module, same pressure angle, SAME hand
  - Helix angle 15–30° typical for FDM

[HAND FIX] A first pass here reused helical_gear.py's external-external rule
(opposite hand) by analogy, on the assumption that a rack is just another
external gear. It isn't, in the one respect that matters for this rule: two
side-by-side external gears contact at a point that is theta=180° apart in
their two LOCAL frames (gear B's contact point faces gear A, i.e. sits at
gear B's own -X side when gear A is to its left), and that 180° offset is
exactly what flips the sign in the derivation (dx/dz at theta0=180° carries
a minus sign relative to theta0=0° that theta0=90°/-90° does not). A rack
sitting directly BELOW its pinion contacts at the pinion's theta=-90° point
— a 90°, not 180°, relative offset — and the flank-slope derivation at that
angle does NOT pick up the same sign flip. Verified empirically, not just
symbolically: building a helix=30° gear with a SAME-hand rack gave 0 mm³
mesh interpenetration (boolean EXACT intersect); an OPPOSITE-hand rack of
the same parameters gave 376 mm³ of gross overlap. Confirmed via numeric
extraction of actual mesh vertices too — a RIGHT-hand gear's tooth flank at
its bottom (rack-facing) contact point shifts by measured dx/dz ≈
+tan(helix_angle) per mm of Z, matching this file's shear(z) formula
directly (no sign flip needed there) — the fix is entirely in which hand
value gets copied from the target, not in the shear math itself.
"""

import bpy
import bmesh
from math import (
    cos, sin, tan, pi, radians, degrees
)
from bpy.props import (
    FloatProperty, IntProperty, EnumProperty
)
from .. import gear_matching

# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
DEFAULT_PRESSURE_ANGLE_DEG = 20.0
ADDENDUM_COEFF       = 1.0
DEDENDUM_COEFF       = 1.25
ROOT_FILLET_COEFF    = 0.38


# ═════════════════════════════════════════════
# MATH LAYER (duplicated from straight_rack.py — see gear_matching.py's
# module docstring for why mesh-building helpers aren't centralized)
# ═════════════════════════════════════════════

def build_rack_tooth_profile(module, pressure_angle_deg):
    """One rack tooth in local coords, centered at X=0. See straight_rack.py
    for the full derivation — identical math, duplicated per convention."""
    pa_rad       = radians(pressure_angle_deg)
    tooth_pitch  = pi * module
    half_pitch   = tooth_pitch / 2.0
    addendum     = ADDENDUM_COEFF   * module
    dedendum     = DEDENDUM_COEFF   * module
    fillet_r     = ROOT_FILLET_COEFF * module

    y_root       = -dedendum
    y_tip        = addendum
    y_fillet_top = y_root + fillet_r

    half_tooth_at_pitch = (pi * module / 2.0) / 2.0
    flank_slope = tan(pa_rad)

    x_flank_right_at_pitch =  half_tooth_at_pitch
    x_flank_left_at_pitch  = -half_tooth_at_pitch

    x_flank_right_fillet_top = x_flank_right_at_pitch + (dedendum - fillet_r) * flank_slope
    x_flank_left_fillet_top  = x_flank_left_at_pitch  - (dedendum - fillet_r) * flank_slope

    x_flank_right_tip = x_flank_right_at_pitch - addendum * flank_slope
    x_flank_left_tip  = x_flank_left_at_pitch  + addendum * flank_slope

    x_flank_right_root = x_flank_right_fillet_top + fillet_r
    x_flank_left_root  = x_flank_left_fillet_top  - fillet_r

    x_root_right = half_pitch
    x_root_left  = -half_pitch

    def fillet_arc(x_center, side, n=3):
        if fillet_r <= 0:
            return []
        pts = []
        for j in range(1, n + 1):
            phi = (pi / 2.0) * j / (n + 1)
            x = x_center + side * fillet_r * sin(phi)
            y = y_root + fillet_r * (1.0 - cos(phi))
            pts.append((x, y))
        return pts

    fillet_left  = fillet_arc(x_flank_left_root,  +1)
    fillet_right = fillet_arc(x_flank_right_root, -1)

    profile = []
    profile.append((x_root_left,  y_root))
    profile.extend(fillet_left)
    profile.append((x_flank_left_fillet_top, y_fillet_top))
    profile.append((x_flank_left_tip,  y_tip))
    profile.append((x_flank_right_tip, y_tip))
    profile.append((x_flank_right_fillet_top, y_fillet_top))
    profile.extend(fillet_right[::-1])
    profile.append((x_root_right, y_root))

    return profile


def build_rack_profile(module, pressure_angle_deg, tooth_count_rack):
    """Full rack profile, teeth tiled along X plus a rectangular base.
    See straight_rack.py for the dedup-pass rationale — identical math."""
    tooth_pitch = pi * module
    dedendum    = DEDENDUM_COEFF * module

    all_pts = []
    for i in range(tooth_count_rack):
        tooth = build_rack_tooth_profile(module, pressure_angle_deg)
        offset_x = i * tooth_pitch
        for x, y in tooth:
            all_pts.append((x + offset_x, y))

    half_pitch = tooth_pitch / 2.0
    x_right = (tooth_count_rack - 1) * tooth_pitch + half_pitch
    x_left  = -half_pitch
    y_base  = -(dedendum + module)

    all_pts.append((x_right, y_base))
    all_pts.append((x_left,  y_base))

    deduped = []
    for pt in all_pts:
        if not deduped or abs(pt[0] - deduped[-1][0]) > 1e-9 or abs(pt[1] - deduped[-1][1]) > 1e-9:
            deduped.append(pt)
    if len(deduped) > 1 and abs(deduped[0][0] - deduped[-1][0]) < 1e-9 and abs(deduped[0][1] - deduped[-1][1]) < 1e-9:
        deduped.pop()

    return deduped


def unique_name(base_name):
    if base_name not in bpy.data.objects:
        return base_name
    i = 1
    while f"{base_name}.{i:03d}" in bpy.data.objects:
        i += 1
    return f"{base_name}.{i:03d}"


# ═════════════════════════════════════════════
# OPERATOR: Add Helical Rack
# ═════════════════════════════════════════════

class OBJECT_OT_helical_rack(bpy.types.Operator):
    """Rack with teeth sheared along X — meshes with a helical pinion"""
    bl_idname  = "object.helical_rack"
    bl_label   = "Helical Rack"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        target = context.window_manager.bmech_gear_target
        gear_matching.sync_helical_same(self, target)
        if target is not None and "bmech_tooth_count" in target.keys():
            self.tooth_count_rack = target["bmech_tooth_count"]

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    module: FloatProperty(
        name="Module (mm)", default=1.0, min=0.1, max=50.0,
        description="Rack module — must match the mating pinion's module",
    )
    pressure_angle_deg: FloatProperty(
        name="Pressure Angle (deg)", default=DEFAULT_PRESSURE_ANGLE_DEG, min=10.0, max=45.0,
    )
    helix_angle_deg: FloatProperty(
        name="Helix Angle (°)", default=20.0, min=1.0, max=45.0,
        description="Tooth shear — 15–30° typical for FDM",
    )
    hand: EnumProperty(
        name="Hand",
        items=[('RIGHT', "Right-hand", "Teeth shear toward +X as Z increases"),
               ('LEFT',  "Left-hand",  "Teeth shear toward -X as Z increases")],
        default='RIGHT',
        description="Mating helical pinion must have the SAME hand — unlike two "
                     "external gears, a rack sits at a different relative angular "
                     "position (below, not beside), which flips the usual rule",
    )
    length_mode: EnumProperty(
        name="Length Mode",
        items=[
            ('TOOTH_COUNT', "Tooth Count",              "Specify number of rack teeth manually"),
            ('MATCH_GEAR',  "Match Gear Circumference", "Span one full gear pitch circumference"),
        ],
        default='TOOTH_COUNT',
    )
    tooth_count_rack: IntProperty(
        name="Tooth Count", default=10, min=2, max=1000,
    )
    width_mm: FloatProperty(
        name="Width (mm)", default=10.0, min=1.0, soft_max=100.0,
        description="Rack thickness along the twist (Z) axis",
    )

    def _derived(self):
        ha_rad = radians(self.helix_angle_deg)
        shear  = self.width_mm * tan(ha_rad)
        return ha_rad, shear

    def draw(self, context):
        layout = self.layout
        target = context.window_manager.bmech_gear_target
        has_target = target is not None
        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")

        target_drives_helix = has_target and "bmech_helix_angle_deg" in target.keys()
        # A plain helical rack meshes one hand or the other of a herringbone
        # pinion's V — see gear_matching.hand_target_ambiguous, same logic
        # helical_gear.py uses for the equivalent gear-gear case.
        hand_ambiguous = gear_matching.hand_target_ambiguous(False, target)

        driven = layout.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "module")
        driven.prop(self, "pressure_angle_deg")

        helix_driven = layout.column(align=True)
        helix_driven.enabled = not target_drives_helix
        helix_driven.prop(self, "helix_angle_deg")
        hand_driven = layout.column(align=True)
        hand_driven.enabled = not target_drives_helix or hand_ambiguous
        hand_driven.prop(self, "hand")

        layout.prop(self, "length_mode")
        if self.length_mode == 'TOOTH_COUNT':
            layout.prop(self, "tooth_count_rack")
        else:
            if target is not None and "bmech_tooth_count" in target.keys():
                layout.label(text="Teeth from target: %d" % target["bmech_tooth_count"])
            else:
                layout.label(text="No target gear set — using tooth count below", icon='INFO')
                layout.prop(self, "tooth_count_rack")
        layout.prop(self, "width_mm")

        _, shear = self._derived()
        layout.label(text="Total shear: %.2f mm" % shear)

    def execute(self, context):
        target = context.window_manager.bmech_gear_target
        target_has_teeth = target is not None and "bmech_tooth_count" in target.keys()

        if self.length_mode == 'MATCH_GEAR' and target_has_teeth:
            tooth_count_rack = target["bmech_tooth_count"]
        else:
            tooth_count_rack = self.tooth_count_rack

        ha_rad, shear = self._derived()
        hand_sign = 1.0 if self.hand == 'RIGHT' else -1.0

        try:
            profile = build_rack_profile(self.module, self.pressure_angle_deg, tooth_count_rack)
        except Exception:
            return {'CANCELLED'}

        n = len(profile)
        bm = bmesh.new()

        bot = [bm.verts.new((x, y, 0.0)) for x, y in profile]
        top = [bm.verts.new((x + hand_sign * shear, y, self.width_mm)) for x, y in profile]
        bm.verts.index_update()

        bm.faces.new(list(reversed(bot)))
        bm.faces.new(top)
        for i in range(n):
            ni = (i + 1) % n
            bm.faces.new([bot[i], bot[ni], top[ni], top[i]])

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        mesh = bpy.data.meshes.new(unique_name("HelicalRack"))
        bm.to_mesh(mesh)
        bm.free()
        mesh.update()

        obj = bpy.data.objects.new(mesh.name, mesh)
        context.collection.objects.link(obj)
        obj.location = context.scene.cursor.location.copy()

        if target is not None and "bmech_module" in target.keys():
            pitch_radius     = target["bmech_module"] * target.get("bmech_tooth_count", tooth_count_rack) / 2.0
            tooth_pitch      = pi * self.module
            half_rack_length = (tooth_count_rack * tooth_pitch) / 2.0
            obj.location     = target.location.copy()
            obj.location.y  -= pitch_radius
            obj.location.x  -= half_rack_length - tooth_pitch / 2.0
            obj.location.x  += gear_matching.rack_phase_align_x(target, tooth_count_rack)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        gear_matching.stamp_gear(obj, "helical_rack", self.module, self.pressure_angle_deg,
                                  helix_angle_deg=self.helix_angle_deg, hand=self.hand)
        return {'FINISHED'}


# ═════════════════════════════════════════════
# REGISTRATION
# ═════════════════════════════════════════════

_classes = [
    OBJECT_OT_helical_rack,
]


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
