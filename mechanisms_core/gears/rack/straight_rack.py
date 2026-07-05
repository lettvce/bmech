"""
Straight Involute Rack Generator for Blender

A rack is a gear of infinite radius: straight flanks at pressure_angle_deg
from vertical instead of curved involute flanks, but otherwise it follows
the same meshing rules as its mating spur/helical pinion (same module, same
pressure angle).

This file used to live combined with the spur gear as
gears/external/involute_gear_rack.py — split out so every rack variant
(straight/helical/herringbone) lives together in its own family, matching
docs/CONVENTIONS.md's family-per-folder shape. profile_to_mesh_object and
unique_name are duplicated here rather than imported, following this
package's convention that low-level mesh-building helpers are private to
each generator file (see gear_matching.py's module docstring).
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

bl_info = {
    "name": "Involute Rack Generator",
    "author": "",
    "version": (0, 1),
    "blender": (5, 1, 0),
    "location": "View3D > Sidebar > Gear & Rack",
    "description": "Generates involute racks for 3D printing",
    "category": "Add Mesh",
}

# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
DEFAULT_PRESSURE_ANGLE_DEG = 20.0
ADDENDUM_COEFF       = 1.0
DEDENDUM_COEFF       = 1.25
ROOT_FILLET_COEFF    = 0.38


def build_rack_tooth_profile(module, pressure_angle_deg):
    """
    One rack tooth in local coords, centered at X=0.
    Rack flanks are straight lines at pressure_angle_deg from vertical.
    Profile goes left→right:
      root-left → fillet-left (root land curving up to the flank) → flank-left → tip-left
      → tip-right → flank-right → fillet-right (flank curving down to the root land) → root-right

    Returns list of (x, y).

    [FIX] The old version had BOTH root-land endpoints (x_root_left/right) AND the
    flank-base endpoints (x_flank_left/right_root) sitting at the exact same
    Y = -dedendum, then ran a "fillet" between two same-height points using a
    parabola that bulges *upward*. Two of those bulges per gap, separated by a
    duplicate corner point, is exactly the "~3 sharp spikes poking up out of the
    gap floor" artifact — there was nothing for a fillet to actually round.

    Now the flank stops `fillet_r` short of the root land (at y_fillet_top =
    y_root + fillet_r), and a quarter-circle fillet carries it the rest of the
    way down to the flat root land at y_root. Both x and y change monotonically
    along each fillet — no rise-then-fall, no spikes above the floor.
    """
    pa_rad       = radians(pressure_angle_deg)
    tooth_pitch  = pi * module
    half_pitch   = tooth_pitch / 2.0
    addendum     = ADDENDUM_COEFF   * module
    dedendum     = DEDENDUM_COEFF   * module
    fillet_r     = ROOT_FILLET_COEFF * module

    # Y coordinates (pitch line at Y=0 by convention, addendum goes positive)
    y_root       = -dedendum            # flat root land — true floor of the gap
    y_pitch      = 0.0
    y_tip        = addendum             # top of tooth
    y_fillet_top = y_root + fillet_r    # where each flank meets its root fillet

    # X coordinate of flank at pitch line: tooth is π*m wide at pitch, half on each side
    # Standard rack tooth thickness at pitch line = π*m/2
    half_tooth_at_pitch = (pi * module / 2.0) / 2.0  # = π*m/4

    # Flank slope: dx/dy = tan(pressure_angle) — flank leans away from tooth centerline
    flank_slope = tan(pa_rad)

    x_flank_right_at_pitch =  half_tooth_at_pitch
    x_flank_left_at_pitch  = -half_tooth_at_pitch

    # Flank X where it meets its fillet, i.e. at Y = y_fillet_top — a drop of
    # (dedendum - fillet_r) from the pitch line. The last fillet_r of vertical
    # drop is covered by the fillet arc, not the straight flank.
    x_flank_right_fillet_top = x_flank_right_at_pitch + (dedendum - fillet_r) * flank_slope
    x_flank_left_fillet_top  = x_flank_left_at_pitch  - (dedendum - fillet_r) * flank_slope

    # Flank X at tip level
    x_flank_right_tip = x_flank_right_at_pitch - addendum * flank_slope
    x_flank_left_tip  = x_flank_left_at_pitch  + addendum * flank_slope

    # Where each fillet meets the flat root land (Y = y_root) — one fillet_r
    # further from the tooth centerline than the flank's fillet-top point.
    x_flank_right_root = x_flank_right_fillet_top + fillet_r
    x_flank_left_root  = x_flank_left_fillet_top  - fillet_r

    # Root boundaries — midpoint of the flat gap floor, shared with the neighboring tooth
    x_root_right = half_pitch
    x_root_left  = -half_pitch

    # ── Root fillet ─────────────────────────────────────────────────────
    # Quarter-circle of radius fillet_r, rounding the corner where the flat
    # root land meets the flank. Returns `n` points strictly between the
    # root-land end (phi -> 0) and the flank end (phi -> 90°), in that order —
    # both x and y increase monotonically with phi (side flips which way x goes).
    def fillet_arc(x_center, side, n=3):
        if fillet_r <= 0:
            return []  # sharp corner, no fillet — handled fine by the caller
        pts = []
        for j in range(1, n + 1):
            phi = (pi / 2.0) * j / (n + 1)   # strictly 0 < phi < 90°, increasing
            x = x_center + side * fillet_r * sin(phi)
            y = y_root + fillet_r * (1.0 - cos(phi))
            pts.append((x, y))
        return pts

    # fillet_left:  root-land end -> approaching the flank, x increasing
    fillet_left  = fillet_arc(x_flank_left_root,  +1)
    # fillet_right: root-land end -> approaching the flank, x decreasing.
    # Used reversed below (flank -> root-land) to match the profile's winding order.
    fillet_right = fillet_arc(x_flank_right_root, -1)

    profile = []
    profile.append((x_root_left,  y_root))                        # root-left
    profile.extend(fillet_left)                                    # curve up to the flank
    profile.append((x_flank_left_fillet_top, y_fillet_top))       # flank-left base
    profile.append((x_flank_left_tip,  y_tip))                    # flank-left tip
    profile.append((x_flank_right_tip, y_tip))                    # tip land right
    profile.append((x_flank_right_fillet_top, y_fillet_top))      # flank-right base
    profile.extend(fillet_right[::-1])                             # curve down to root land
    profile.append((x_root_right, y_root))                        # root-right

    return profile


def build_rack_profile(module, pressure_angle_deg, tooth_count_rack):
    """
    Full rack profile: tooth_count_rack teeth tiled along X,
    closed with a rectangular base below the dedendum line.
    Returns list of (x, y).

    [FIX] Each tooth's own x_root_right end point is mathematically
    identical to the next tooth's x_root_left start point (both equal
    offset_i + half_pitch, by construction) — tiling teeth by simple
    concatenation put a literal duplicate vertex at every tooth-to-tooth
    boundary. The old closing logic then added an explicit "back up to
    start" point that ALSO duplicated tooth 0's own first point — doubly
    unnecessary, since bm.faces.new() (see profile_to_mesh_object)
    auto-closes the polygon loop from its last vertex back to its first;
    no explicit closing point was ever needed. Two adjacent polygon
    vertices at the identical XY position produce a genuine zero-width
    side-wall face once Solidify extrudes the profile — confirmed via the
    evaluated (post-Solidify) mesh: exactly `tooth_count_rack` zero-area
    faces (one per tooth-to-tooth junction plus the wraparound), not
    visible from the raw pre-modifier mesh alone. Deduplicating adjacent
    (including wraparound) points below fixes it generally, without
    needing to special-case which of the two causes produced a given
    duplicate.
    """
    tooth_pitch = pi * module
    dedendum    = DEDENDUM_COEFF * module

    all_pts = []
    for i in range(tooth_count_rack):
        tooth = build_rack_tooth_profile(module, pressure_angle_deg)
        offset_x = i * tooth_pitch
        for x, y in tooth:
            all_pts.append((x + offset_x, y))

    # Close the profile with a rectangular base
    # Rightmost X of last tooth + half_pitch, leftmost = 0 - half_pitch
    half_pitch = tooth_pitch / 2.0
    x_right = (tooth_count_rack - 1) * tooth_pitch + half_pitch
    x_left  = -half_pitch
    y_base  = -(dedendum + module)  # extra clearance below root, makes a solid base

    # Append base rect. The last tooth's profile already ends at
    # (x_right, -dedendum) — that's this rect's top-right corner.
    all_pts.append((x_right, y_base))
    all_pts.append((x_left,  y_base))

    # Deduplicate adjacent points (including the wraparound from the last
    # point back to the first, since bm.faces.new() closes the loop there
    # too) — see the docstring above for why duplicates occur here.
    deduped = []
    for pt in all_pts:
        if not deduped or abs(pt[0] - deduped[-1][0]) > 1e-9 or abs(pt[1] - deduped[-1][1]) > 1e-9:
            deduped.append(pt)
    if len(deduped) > 1 and abs(deduped[0][0] - deduped[-1][0]) < 1e-9 and abs(deduped[0][1] - deduped[-1][1]) < 1e-9:
        deduped.pop()

    return deduped


# ═════════════════════════════════════════════
# MESH + SOLIDIFY PIPELINE
# ═════════════════════════════════════════════

def profile_to_mesh_object(profile_points, name, width_mm):
    """Fill a closed 2D profile as a single n-gon face and attach a Solidify modifier.

    The profile is placed at local z = width_mm / 2 so the centered (offset=0)
    Solidify below spans local [0, width_mm] — matching the bottom-at-z=0
    convention used by the other gear generators.
    """
    bm    = bmesh.new()
    verts = [bm.verts.new((x, y, width_mm / 2.0)) for x, y in profile_points]
    bm.verts.index_update()
    bm.faces.new(verts)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    mesh.update()

    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)

    mod           = obj.modifiers.new("Thickness", 'SOLIDIFY')
    mod.thickness = width_mm
    mod.offset    = 0.0

    return obj


def unique_name(base_name):
    """
    If 'Rack' already exists, return 'Rack.001', etc.
    Blender does this automatically on object creation but we need the name
    ahead of time for scene storage. So we pre-compute it.
    """
    if base_name not in bpy.data.objects:
        return base_name
    i = 1
    while f"{base_name}.{i:03d}" in bpy.data.objects:
        i += 1
    return f"{base_name}.{i:03d}"


# ═════════════════════════════════════════════
# OPERATOR: Add Rack
# ═════════════════════════════════════════════

class OBJECT_OT_add_rack(bpy.types.Operator):
    """Generate a filled involute rack mesh"""
    bl_idname  = "object.add_rack"
    bl_label   = "Add Gear Rack"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        target = context.window_manager.bmech_gear_target
        gear_matching.sync_module_pa(self, target)
        if target is not None and "bmech_tooth_count" in target.keys():
            self.tooth_count_rack = target["bmech_tooth_count"]

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    module: FloatProperty(
        name="Module (mm)", default=1.0, min=0.1, max=50.0,
        description="Rack module — must match gear module to mesh correctly",
    )
    pressure_angle_deg: FloatProperty(
        name="Pressure Angle (deg)", default=DEFAULT_PRESSURE_ANGLE_DEG, min=10.0, max=45.0,
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
        name="Width (mm)", default=6.0, min=0.1, soft_max=100.0,
        description="Rack thickness — Solidify modifier depth",
    )

    def draw(self, context):
        layout = self.layout
        target = context.window_manager.bmech_gear_target
        has_target = target is not None
        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        driven = layout.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "module")
        driven.prop(self, "pressure_angle_deg")
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

    def execute(self, context):
        target = context.window_manager.bmech_gear_target
        target_has_teeth = target is not None and "bmech_tooth_count" in target.keys()

        if self.length_mode == 'MATCH_GEAR' and target_has_teeth:
            tooth_count_rack = target["bmech_tooth_count"]
        else:
            tooth_count_rack = self.tooth_count_rack

        try:
            profile = build_rack_profile(self.module, self.pressure_angle_deg, tooth_count_rack)
        except Exception as e:
            return {'CANCELLED'}

        obj = profile_to_mesh_object(profile, unique_name("Rack"), self.width_mm)
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

        gear_matching.stamp_gear(obj, "rack", self.module, self.pressure_angle_deg)
        return {'FINISHED'}


# ═════════════════════════════════════════════
# REGISTRATION
# ═════════════════════════════════════════════

_classes = [
    OBJECT_OT_add_rack,
]


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    # Running directly in Text Editor — handy for testing
    register()
