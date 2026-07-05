"""
Involute Spur Gear Generator for Blender

Math is correct, I checked it twice. If your gear looks wrong, blame your pressure angle.

[FIXED] Turns out it wasn't the pressure angle. See build_tooth_profile / build_gear_profile.

This file used to also contain the rack generator (as involute_gear_rack.py) —
split out into gears/rack/ so racks get their own family, matching the
docs/CONVENTIONS.md family-per-folder shape. build_tooth_profile/
build_gear_profile/compute_involute_point/involute_angle_at_radius/
rotate_point stay here since cluster_gear.py and compound_gear.py both
import this module directly for them.
"""

import bpy
import bmesh
from math import (
    cos, sin, sqrt, pi, radians, atan2
)
from bpy.props import (
    FloatProperty, IntProperty, BoolProperty
)
from .. import gear_matching

bl_info = {
    "name": "Involute Spur Gear Generator",
    "author": "",
    "version": (0, 1),
    "blender": (5, 1, 0),
    "location": "View3D > Sidebar > Gear & Rack",
    "description": "Generates involute spur gears for 3D printing",
    "category": "Add Mesh",
}

# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
INVOLUTE_POINTS      = 15
DEFAULT_PRESSURE_ANGLE_DEG = 20.0
ADDENDUM_COEFF       = 1.0
DEDENDUM_COEFF       = 1.25
MIN_TOOTH_COUNT      = 5
BOOL_EPSILON         = 0.001
BORE_SEGS            = 32


# ═════════════════════════════════════════════
# MATH LAYER
# ═════════════════════════════════════════════

def compute_involute_point(base_radius, t):
    """
    Standard parametric involute of a circle.
    t=0 → point of departure from base circle.
    Returns (x, y), Z is always 0 (caller's problem).
    """
    x = base_radius * (cos(t) + t * sin(t))
    y = base_radius * (sin(t) - t * cos(t))
    return (x, y)


def involute_angle_at_radius(base_radius, r):
    """
    Invert involute: given radius r, return parameter t.
    Derived from r² = base_radius²(1 + t²).
    """
    ratio = r / base_radius
    if ratio < 1.0:
        # We're inside the base circle — t doesn't exist, clamp to 0
        return 0.0
    return sqrt(ratio * ratio - 1.0)


def rotate_point(x, y, angle):
    """Rotate 2D point by angle (radians). Simple, not magic."""
    c, s = cos(angle), sin(angle)
    return (x * c - y * s, x * s + y * c)


def build_tooth_profile(module, tooth_count, pressure_angle_deg):
    """
    Build one tooth in local coords — tooth centerline on +X axis.
    Coordinate model:
      - Tooth tip points toward +X
      - Right flank has positive Y, left flank has negative Y
      - "Right" and "left" are as seen looking outward from gear center

    Profile winding order (CCW, Blender's preferred):
      left_root → left flank (root→tip) → tip land → right flank (tip→root) → right_root

    This is the order build_gear_profile needs: tooth N's right_root connects
    straight into a dedendum-circle arc that ends at tooth N+1's left_root,
    with no jumps and no overlap.

    [FIX] The involute as produced by compute_involute_point has its polar
    angle = inv(alpha) = tan(alpha) - alpha, which INCREASES monotonically
    with t (i.e. with radius). A real gear tooth does the OPPOSITE — it gets
    NARROWER going outward:
        half_angle(r) = half_tooth_angle + inv(alpha_pitch) - inv(alpha_r)
    A plain rotation can't turn "+inv(alpha_r)" into "-inv(alpha_r)" — that
    needs a MIRROR (negate y) as well. The old code rotated without mirroring,
    which gave half_angle(r) = half_tooth_angle - inv(alpha_pitch) + inv(alpha_r)
    — the tooth got WIDER going outward instead of narrower, and the whole
    gear profile self-intersected into a pinwheel. Mirror first, then rotate
    by (half_tooth_angle + angle_at_pitch), and it comes out the right way round.
    """
    pa_rad           = radians(pressure_angle_deg)
    pitch_radius     = module * tooth_count / 2.0
    base_radius      = pitch_radius * cos(pa_rad)
    addendum_radius  = pitch_radius + ADDENDUM_COEFF * module
    dedendum_radius  = pitch_radius - DEDENDUM_COEFF * module

    # Standard tooth thickness at pitch circle = π*m/2 (arc length)
    # That arc spans this half-angle on each side of the tooth centerline:
    half_tooth_angle = pi / (2.0 * tooth_count)   # = (π*m/2) / (2 * pitch_radius), simplified

    # ── Sample the raw involute ──────────────────────────────────────────
    # t=0 is where the involute departs the base circle.
    # compute_involute_point at t=0 gives (base_radius, 0) — on +X.
    # As t increases the curve sweeps CCW (polar angle increases with t).
    t_start = involute_angle_at_radius(base_radius, max(dedendum_radius, base_radius))
    t_tip   = involute_angle_at_radius(base_radius, addendum_radius)

    raw_flank = []
    for i in range(INVOLUTE_POINTS):
        t = t_start + (t_tip - t_start) * i / (INVOLUTE_POINTS - 1)
        raw_flank.append(compute_involute_point(base_radius, t))

    # ── Mirror + rotate flank into position ──────────────────────────────
    # The involute point at the pitch circle — find its polar angle:
    t_pitch = involute_angle_at_radius(base_radius, pitch_radius)
    px, py  = compute_involute_point(base_radius, t_pitch)
    angle_at_pitch = atan2(py, px)   # = inv(pressure_angle_at_pitch), grows with radius

    # [FIX] Mirror (negate y) THEN rotate by +(half_tooth_angle + angle_at_pitch).
    # Mirroring flips "angle grows with radius" into "angle shrinks with radius",
    # which is what a real involute flank does relative to the tooth centerline.
    # The rotation puts the pitch-circle crossing at exactly +half_tooth_angle.
    rotation = half_tooth_angle + angle_at_pitch

    right_flank = [rotate_point(x, -y, rotation) for x, y in raw_flank]
    # right_flank[0]  = root end (lowest radius, positive Y side, LARGEST angle)
    # right_flank[-1] = tip end  (highest radius, positive Y side, SMALLEST angle)

    # When base_radius > dedendum_radius (standard case for most tooth counts), the involute
    # departs the base circle ABOVE the dedendum circle, leaving a gap at the root.
    # Extend each flank downward with a short radial line to the dedendum circle.
    # This makes the tooth endpoints land exactly on the dedendum circle, so the
    # space arc in build_gear_profile can connect them cleanly with no radius jumps.
    if base_radius > dedendum_radius:
        root_r_angle = atan2(right_flank[0][1], right_flank[0][0])
        right_flank = [(dedendum_radius * cos(root_r_angle), dedendum_radius * sin(root_r_angle))] + right_flank

    # ── Mirror to get left flank ─────────────────────────────────────────
    # Tooth is symmetric about the +X axis (tooth centerline).
    # Mirror across X axis = negate Y.
    # right_flank goes root→tip with angle DEcreasing (e.g. ~+5.35° → ~+1.81° for 20T/20°PA, m=1)
    # mirror:  left_flank  goes root→tip with angle INcreasing (~-5.35° → ~-1.81°)
    left_flank = [(x, -y) for x, y in right_flank]   # root→tip, negative Y, root at -root_r_local

    # ── Tip land ─────────────────────────────────────────────────────────
    # One point at the peak of the tooth: (addendum_radius, 0) in local tooth coords.
    # This sits between left_flank tip (~-1.81°) and right_flank tip (~+1.81°).
    tip_land = [(addendum_radius, 0.0)]

    # ── Assemble ──────────────────────────────────────────────────────────
    # CCW winding for the full gear profile:
    # left_root(-root_r_local) → left_flank ascending (root→tip, angle rising toward 0) →
    # tip_land(0°) → right_flank descending (tip→root, angle rising away from 0) → right_root(+root_r_local)
    #
    # The whole tooth profile has a MONOTONICALLY INCREASING polar angle from
    # -root_r_local to +root_r_local, narrowing toward the tip (root wider than
    # pitch, pitch wider than tip — the normal shape). build_gear_profile then
    # inserts a dedendum-circle arc from this right_root to the NEXT tooth's
    # left_root, continuing the increasing-angle sweep around the gear. ✓
    profile = []
    profile.extend(left_flank)        # root→tip, negative Y side
    profile.extend(tip_land)          # tip peak
    profile.extend(right_flank[::-1]) # tip→root, positive Y side (reversed)
    # ends at right_root (+root_r_local) on dedendum circle

    return profile


def build_gear_profile(module, tooth_count, pressure_angle_deg):
    """
    Full closed gear profile: tooth profile rotated tooth_count times,
    with tooth-space arcs (dedendum circle segments) inserted between teeth.

    Each tooth profile (build_tooth_profile) sweeps from its own left_root,
    through the tip, to its right_root — a monotonically increasing polar-angle
    run of width 2*root_r_local. Between tooth i's right_root and tooth i+1's
    left_root sits the tooth SPACE (the gap); we cover it with a dedendum-circle
    arc through the gap's center, continuing the increasing-angle sequence.

    Without this arc, the profile would jump straight from one tooth's root
    to the next tooth's root across the gap — Blender's curve fill can usually
    cope with that, but the arc keeps the root circle properly rounded.
    """
    pa_rad            = radians(pressure_angle_deg)
    pitch_radius      = module * tooth_count / 2.0
    base_radius       = pitch_radius * cos(pa_rad)
    dedendum_radius   = pitch_radius - DEDENDUM_COEFF * module
    half_tooth_angle  = pi / (2.0 * tooth_count)
    tooth_pitch_angle = 2.0 * pi / tooth_count

    # Determine the local polar angle of the right_root (same for left_root by symmetry, negated)
    t_pitch = involute_angle_at_radius(base_radius, pitch_radius)
    t_start = involute_angle_at_radius(base_radius, max(dedendum_radius, base_radius))
    px, py  = compute_involute_point(base_radius, t_pitch)
    angle_at_pitch  = atan2(py, px)

    # [FIX] Same mirror-then-rotate convention as build_tooth_profile — see the
    # comment block there for why it's "+" and a mirror, not "-" and a plain rotate.
    rotation        = half_tooth_angle + angle_at_pitch

    # The root point of the right flank in local tooth coords, after the same
    # mirror (negate y) + rotation that build_tooth_profile applies to raw_flank.
    root_pt_local   = compute_involute_point(base_radius, t_start)
    root_pt_rotated = rotate_point(root_pt_local[0], -root_pt_local[1], rotation)
    root_r_local    = atan2(root_pt_rotated[1], root_pt_rotated[0])  # small positive angle
    root_l_local    = -root_r_local                                    # symmetric

    tooth_pts = build_tooth_profile(module, tooth_count, pressure_angle_deg)
    SPACE_PTS = 4  # intermediate points along the tooth-space dedendum arc

    profile = []
    for i in range(tooth_count):
        tooth_angle = i * tooth_pitch_angle

        # Rotate tooth profile into position
        for x, y in tooth_pts:
            rx, ry = rotate_point(x, y, tooth_angle)
            profile.append((rx, ry))

        # Space arc along dedendum circle from this tooth's right_root to next tooth's left_root.
        # Tooth profile ends at right_root (+root_r_local in local coords).
        # Next tooth's left_root is at (tooth_angle + tooth_pitch_angle) + root_l_local
        #                             = tooth_angle + tooth_pitch_angle - root_r_local.
        # Span = tooth_pitch_angle - 2*root_r_local (~7.29° for 20 teeth at 20°PA, m=1).
        # This arc goes through the gap center and does NOT cross the tooth body. ✓
        a_start = tooth_angle + root_r_local                              # this tooth's right_root
        a_end   = tooth_angle + tooth_pitch_angle + root_l_local          # next tooth's left_root
        for j in range(1, SPACE_PTS + 1):
            a = a_start + (a_end - a_start) * j / (SPACE_PTS + 1)
            profile.append((dedendum_radius * cos(a), dedendum_radius * sin(a)))

    return profile


# ═════════════════════════════════════════════
# MESH + SOLIDIFY PIPELINE
# ═════════════════════════════════════════════

def profile_to_mesh_object(profile_points, name, width_mm):
    """Fill a closed 2D profile as a single n-gon face and attach a Solidify modifier.

    The profile is placed at local z = width_mm / 2 so the centered (offset=0)
    Solidify below spans local [0, width_mm] — matching the bottom-at-z=0
    convention used by the other gear generators, and what _apply_bore's
    cutter assumes.
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


def _apply_bore(context, obj, bore_r, width_mm):
    """Boolean-difference a bore cylinder through obj. Cutter deleted after."""
    bm     = bmesh.new()
    angles = [2.0 * pi * i / BORE_SEGS for i in range(BORE_SEGS)]
    z0, z1 = -BOOL_EPSILON, width_mm + BOOL_EPSILON

    vb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angles]
    vt = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angles]
    bm.verts.index_update()

    for i in range(BORE_SEGS):
        ni = (i + 1) % BORE_SEGS
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])
    bm.faces.new(vb)
    bm.faces.new(list(reversed(vt)))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me_cut = bpy.data.meshes.new("__SGBoreMesh")
    bm.to_mesh(me_cut)
    bm.free()
    me_cut.update()

    cutter = bpy.data.objects.new("__SGBore", me_cut)
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
    # and this modifier is left un-applied. Deselect everything and select
    # obj explicitly first, same pattern as hex_bolt.py/hex_nut.py's
    # _bool_diff.
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    context.view_layer.objects.active = obj
    with context.temp_override(active_object=obj):
        bpy.ops.object.modifier_apply(modifier="Bore")

    bpy.data.objects.remove(cutter, do_unlink=True)


def unique_name(base_name):
    """
    If 'Gear' already exists, return 'Gear.001', etc.
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
# OPERATOR: Add Spur Gear
# ═════════════════════════════════════════════

class OBJECT_OT_add_spur_gear(bpy.types.Operator):
    """Generate a filled involute spur gear mesh"""
    bl_idname  = "object.add_spur_gear"
    bl_label   = "Add Spur Gear"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        gear_matching.sync_module_pa(self, context.window_manager.bmech_gear_target)

    def invoke(self, context, event):
        gear_matching.reset_target(context)
        return self.execute(context)

    module: FloatProperty(
        name="Module (mm)", default=1.0, min=0.1, max=50.0,
        description="Gear module — controls tooth size",
    )
    tooth_count: IntProperty(
        name="Tooth Count", default=20, min=MIN_TOOTH_COUNT, max=500,
    )
    pressure_angle_deg: FloatProperty(
        name="Pressure Angle (deg)", default=DEFAULT_PRESSURE_ANGLE_DEG, min=10.0, max=45.0,
    )
    width_mm: FloatProperty(
        name="Width (mm)", default=6.0, min=0.1, soft_max=100.0,
        description="Gear thickness — Solidify modifier depth",
    )
    bore_enable: BoolProperty(name="Bore Hole", default=True)
    bore_diameter: FloatProperty(
        name="Bore Ø (mm)", default=5.0, min=0.1, soft_max=50.0,
    )
    bore_compensation: FloatProperty(
        name="Compensation (mm)", default=0.2, min=0.0, soft_max=1.0,
        description="FDM holes print tight — added to bore radius",
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(context.window_manager, "bmech_gear_target", text="Match Target")
        has_target = context.window_manager.bmech_gear_target is not None
        module_row = layout.row()
        module_row.enabled = not has_target
        module_row.prop(self, "module")
        layout.prop(self, "tooth_count")
        layout.prop(self, "pressure_angle_deg")
        layout.prop(self, "width_mm")
        layout.separator()
        layout.prop(self, "bore_enable")
        if self.bore_enable:
            col = layout.column(align=True)
            col.prop(self, "bore_diameter")
            col.prop(self, "bore_compensation")
        pitch_d = self.module * self.tooth_count
        layout.label(text="Pitch Ø: %.2f mm" % pitch_d)
        pa_rad  = radians(self.pressure_angle_deg)
        pitch_r = self.module * self.tooth_count / 2.0
        ded_r   = pitch_r - DEDENDUM_COEFF * self.module
        if (pitch_r * cos(pa_rad)) > ded_r:
            layout.label(text="Undercut likely", icon='ERROR')
        if ded_r <= 0:
            layout.label(text="Module too large — dedendum radius is zero or negative", icon='ERROR')
        pa_max = gear_matching.max_pressure_angle_deg(self.tooth_count, ADDENDUM_COEFF)
        layout.label(text="Max pressure angle for %d teeth: %.1f°" % (self.tooth_count, pa_max))
        if self.bore_enable:
            bore_r = self.bore_diameter / 2.0 + self.bore_compensation
            if bore_r >= ded_r:
                layout.label(text="Bore too large for dedendum radius", icon='ERROR')

    def execute(self, context):
        gear_matching.clamp_pressure_angle(self, (self.tooth_count, ADDENDUM_COEFF))
        pa_rad          = radians(self.pressure_angle_deg)
        pitch_radius    = self.module * self.tooth_count / 2.0
        dedendum_radius = pitch_radius - DEDENDUM_COEFF * self.module

        if dedendum_radius <= 0:
            return {'CANCELLED'}

        try:
            profile = build_gear_profile(self.module, self.tooth_count, self.pressure_angle_deg)
        except Exception as e:
            return {'CANCELLED'}

        obj = profile_to_mesh_object(profile, unique_name("Gear"), self.width_mm)
        obj.location = context.scene.cursor.location.copy()

        if self.bore_enable:
            bore_r = self.bore_diameter / 2.0 + self.bore_compensation
            if bore_r > 0 and bore_r < dedendum_radius:
                # modifier_apply's poll requires the active object to also
                # be the ONLY selected one — a previously-created gear left
                # selected from an earlier call makes this silently fail
                # (CANCELLED, not an exception) without the explicit
                # deselect/select/activate below.
                bpy.ops.object.select_all(action='DESELECT')
                obj.select_set(True)
                context.view_layer.objects.active = obj
                with context.temp_override(active_object=obj):
                    bpy.ops.object.modifier_apply(modifier="Thickness")
                _apply_bore(context, obj, bore_r, self.width_mm)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        gear_matching.stamp_gear(obj, "spur", self.module, self.pressure_angle_deg,
                                  tooth_count=self.tooth_count)
        return {'FINISHED'}


# ═════════════════════════════════════════════
# REGISTRATION
# ═════════════════════════════════════════════

_classes = [
    OBJECT_OT_add_spur_gear,
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
