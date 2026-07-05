"""
Herringbone Rack Generator
──────────────────────────
A herringbone rack is a straight rack sheared into a V along X: the bottom
half shears one way from Z=0 to Z=width/2, the top half shears back the
other way from Z=width/2 to Z=width — the gear-of-infinite-radius limit of
herringbone_gear.py's V-twist, the same way helical_rack.py is the limit of
helical_gear.py's twist. See helical_rack.py's module docstring for the
gear→rack shear derivation.

Tooth geometry:
  Bottom half  Z=0 → Z=width/2   : shear rises  0 → peak
  Top half     Z=width/2 → Z=width: shear falls peak → 0
  peak_shear = (width/2) × tan(helix_angle)

[NOTE] Each half is its own LINEAR (affine) shear of Z within its own
range — see helical_rack.py's docstring for why an affine shear needs no
intermediate Z slices to be exact. A herringbone rack therefore needs
exactly THREE layers (Z=0, Z=width/2, Z=width), not the many slices per
half that herringbone_gear.py needs to approximate its curved twist.

Meshing rules — NOT the same as two external herringbone gears:
  - Same module, same pressure angle, SAME hand
  - Helix angle 15–30° typical for FDM

[HAND FIX] See helical_rack.py's module docstring for the full derivation
and empirical verification of why a rack needs the SAME hand as its mating
pinion, not the opposite hand the external-external gear-gear rule would
suggest — a rack sits at a 90°-offset relative angular position (directly
below its pinion) rather than the 180°-offset position two side-by-side
external gears sit at, and that difference is exactly what flips the sign.
The same derivation applies unchanged to the herringbone case (each half
is its own linear shear, so the initial-slope argument at Z=0 carries over
identically).
"""

import bpy
import bmesh
from math import (
    cos, sin, tan, pi, radians
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
# OPERATOR: Add Herringbone Rack
# ═════════════════════════════════════════════

class OBJECT_OT_herringbone_rack(bpy.types.Operator):
    """Rack with V-sheared teeth — meshes with a herringbone pinion"""
    bl_idname  = "object.herringbone_rack"
    bl_label   = "Herringbone Rack"
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
        description="Half-angle of the V — 15–30° typical for FDM",
    )
    hand: EnumProperty(
        name="Hand",
        items=[('RIGHT', "Right-hand", "Bottom half shears toward +X as Z increases"),
               ('LEFT',  "Left-hand",  "Bottom half shears toward -X as Z increases")],
        default='RIGHT',
        description="Mating herringbone pinion must have the SAME hand — unlike "
                     "two external gears, a rack sits at a different relative "
                     "angular position (below, not beside), which flips the "
                     "usual rule",
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
        name="Total Width (mm)", default=14.0, min=2.0, soft_max=100.0,
        description="Full length along the V axis — each half is width/2",
    )

    def _derived(self):
        ha_rad     = radians(self.helix_angle_deg)
        half_w     = self.width_mm / 2.0
        peak_shear = half_w * tan(ha_rad)
        return ha_rad, half_w, peak_shear

    def draw(self, context):
        layout = self.layout
        target = context.window_manager.bmech_gear_target
        has_target = target is not None
        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")

        target_drives_helix = has_target and "bmech_helix_angle_deg" in target.keys()
        # A herringbone rack meshes a plain helical pinion on only one half
        # of its V — see gear_matching.hand_target_ambiguous, same logic
        # herringbone_gear.py uses for the equivalent gear-gear case.
        hand_ambiguous = gear_matching.hand_target_ambiguous(True, target)

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

        _, half_w, peak_shear = self._derived()
        layout.label(text="Half width: %.2f mm" % half_w)
        layout.label(text="Peak shear: %.2f mm" % peak_shear)

    def execute(self, context):
        target = context.window_manager.bmech_gear_target
        target_has_teeth = target is not None and "bmech_tooth_count" in target.keys()

        if self.length_mode == 'MATCH_GEAR' and target_has_teeth:
            tooth_count_rack = target["bmech_tooth_count"]
        else:
            tooth_count_rack = self.tooth_count_rack

        ha_rad, half_w, peak_shear = self._derived()
        hand_sign = 1.0 if self.hand == 'RIGHT' else -1.0

        try:
            profile = build_rack_profile(self.module, self.pressure_angle_deg, tooth_count_rack)
        except Exception:
            return {'CANCELLED'}

        n = len(profile)
        bm = bmesh.new()

        def _make_layer(z, shear):
            return [bm.verts.new((x + hand_sign * shear, y, z)) for x, y in profile]

        bottom = _make_layer(0.0, 0.0)
        middle = _make_layer(half_w, peak_shear)
        top    = _make_layer(self.width_mm, 0.0)
        bm.verts.index_update()

        bm.faces.new(list(reversed(bottom)))
        bm.faces.new(top)

        for lower, upper in ((bottom, middle), (middle, top)):
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([lower[i], lower[ni], upper[ni], upper[i]])

        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

        mesh = bpy.data.meshes.new(unique_name("HerringboneRack"))
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

        gear_matching.stamp_gear(obj, "herringbone_rack", self.module, self.pressure_angle_deg,
                                  helix_angle_deg=self.helix_angle_deg, hand=self.hand)
        return {'FINISHED'}


# ═════════════════════════════════════════════
# REGISTRATION
# ═════════════════════════════════════════════

_classes = [
    OBJECT_OT_herringbone_rack,
]


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
