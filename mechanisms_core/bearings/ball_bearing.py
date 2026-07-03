"""
Ball Bearing Generator
======================
Generates an inner race, outer race, and ball_count truncated-sphere balls.

Pitch diameter is driven by ball packing:
    ball_center_r = (2·ball_r + gap_mm) / (2·sin(π / ball_count))

Bore ID and outer OD are user-specified.  Inner and outer wall thicknesses are
derived and validated — errors if either falls below 0.8 mm.

Race grooves are semicircular arcs of radius groove_r = ball_r + clearance/2,
spanning only ±z_cut (the ball truncation height).  Ball flat caps sit flush with
race faces; the ball zone is open at top and bottom for FDM assembly.
"""

import bpy
import bmesh
import math
from bpy.props import (
    FloatProperty, IntProperty, EnumProperty, BoolProperty, FloatVectorProperty,
)

RACE_SEGMENTS = 64
N_ARC         = 16
BALL_LON      = 32
BALL_LAT      = 16
MIN_WALL_MM   = 0.8


# ── Geometry math ──────────────────────────────────────────────────────────────

def resolve_ball(sizing_mode, ball_radius_mm, ball_height_mm, overhang_rad):
    """Return (ball_r, h, z_cut, r_flat)."""
    if sizing_mode == 'RADIUS':
        ball_r = ball_radius_mm
        h      = 2.0 * ball_r * math.cos(overhang_rad)
    else:
        h      = ball_height_mm
        ball_r = h / (2.0 * math.cos(overhang_rad))
    z_cut  = ball_r * math.cos(overhang_rad)
    r_flat = ball_r * math.sin(overhang_rad)
    return ball_r, h, z_cut, r_flat


def derive_bearing(ball_r, ball_count, gap_mm, bore_r, outer_r, clearance_mm, z_cut):
    """
    Return (ball_center_r, groove_r, inner_wall, outer_wall, bearing_width).

    Pitch circle from packing:
        ball_center_r = (2·ball_r + gap_mm) / (2·sin(π / ball_count))
    Inner wall = ball_center_r − ball_r − bore_r
    Outer wall = outer_r − (ball_center_r + groove_r)
    """
    ball_center_r = (2.0 * ball_r + gap_mm) / (2.0 * math.sin(math.pi / ball_count))
    groove_r      = ball_r + clearance_mm / 2.0
    inner_wall    = ball_center_r - ball_r - bore_r
    outer_wall    = outer_r - (ball_center_r + groove_r)
    bearing_width = 2.0 * z_cut
    return ball_center_r, groove_r, inner_wall, outer_wall, bearing_width


def validate_bearing(ball_r, z_cut, groove_r, bore_r, outer_r,
                     ball_center_r, inner_wall, outer_wall, gap_mm):
    errs = []
    if ball_r <= 0:
        errs.append("Ball radius must be > 0.")
    if z_cut <= 0:
        errs.append("Ball height is zero or negative — check overhang angle.")
    if z_cut >= groove_r:
        errs.append("Overhang angle too close to 90° for groove geometry.")
    if gap_mm <= 0:
        errs.append("Ball gap must be > 0.")
    if bore_r <= 0:
        errs.append("Bore diameter must be > 0.")
    if outer_r <= bore_r:
        errs.append("Outer diameter must be greater than bore diameter.")
    if inner_wall < MIN_WALL_MM:
        errs.append(
            "Inner wall too thin: %.2f mm (min %.1f mm). "
            "Reduce bore ID, reduce ball size, or increase ball count."
            % (inner_wall, MIN_WALL_MM)
        )
    if outer_wall < MIN_WALL_MM:
        errs.append(
            "Outer wall too thin: %.2f mm (min %.1f mm). "
            "Increase outer OD, reduce ball size, or increase ball count."
            % (outer_wall, MIN_WALL_MM)
        )
    return errs


# ── Profile builders ───────────────────────────────────────────────────────────

def _race_alpha_cut(groove_r, z_cut):
    return math.asin(min(1.0, z_cut / groove_r))


def _inner_race_profile(bore_r, ball_center_r, groove_r, z_cut):
    alpha_cut = _race_alpha_cut(groove_r, z_cut)
    r_lip     = ball_center_r - groove_r * math.cos(alpha_cut)

    pts = []
    pts.append((bore_r, -z_cut))
    pts.append((r_lip,  -z_cut))

    for k in range(1, N_ARC):
        alpha = -alpha_cut + 2.0 * alpha_cut * k / N_ARC
        pts.append((ball_center_r - groove_r * math.cos(alpha),
                    groove_r * math.sin(alpha)))

    pts.append((r_lip,  +z_cut))
    pts.append((bore_r, +z_cut))
    return pts


def _outer_race_profile(ball_center_r, groove_r, outer_r, z_cut):
    alpha_cut = _race_alpha_cut(groove_r, z_cut)
    r_lip     = ball_center_r + groove_r * math.cos(alpha_cut)

    pts = []
    pts.append((r_lip,   -z_cut))
    pts.append((outer_r, -z_cut))
    pts.append((outer_r, +z_cut))
    pts.append((r_lip,   +z_cut))

    for k in range(N_ARC - 1, 0, -1):
        alpha = -alpha_cut + 2.0 * alpha_cut * k / N_ARC
        pts.append((ball_center_r + groove_r * math.cos(alpha),
                    groove_r * math.sin(alpha)))

    return pts


# ── Mesh builders ──────────────────────────────────────────────────────────────

def _revolve_profile(bm, profile_rz, n_seg):
    rings = []
    for r, z in profile_rz:
        ring = [bm.verts.new((r * math.cos(2 * math.pi * i / n_seg),
                               r * math.sin(2 * math.pi * i / n_seg), z))
                for i in range(n_seg)]
        rings.append(ring)

    n_prof = len(profile_rz)
    for pi in range(n_prof):
        npi = (pi + 1) % n_prof
        for i in range(n_seg):
            ni = (i + 1) % n_seg
            bm.faces.new([rings[pi][i], rings[pi][ni],
                          rings[npi][ni], rings[npi][i]])


def build_inner_race(context, name, bore_r, ball_center_r, groove_r, z_cut, location):
    bm = bmesh.new()
    _revolve_profile(bm, _inner_race_profile(bore_r, ball_center_r, groove_r, z_cut),
                     RACE_SEGMENTS)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return _link_mesh(context, bm, name, location)


def build_outer_race(context, name, ball_center_r, groove_r, outer_r, z_cut, location):
    bm = bmesh.new()
    _revolve_profile(bm, _outer_race_profile(ball_center_r, groove_r, outer_r, z_cut),
                     RACE_SEGMENTS)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return _link_mesh(context, bm, name, location)


def build_truncated_sphere(context, name, ball_r, z_cut, location):
    bm       = bmesh.new()
    elev_cut = math.asin(max(-1.0, min(1.0, z_cut / ball_r)))

    rings = []
    for i in range(BALL_LAT + 1):
        t    = i / BALL_LAT
        elev = -elev_cut + 2.0 * elev_cut * t
        z    = ball_r * math.sin(elev)
        r    = ball_r * math.cos(elev)
        row  = [bm.verts.new((r * math.cos(2 * math.pi * j / BALL_LON),
                               r * math.sin(2 * math.pi * j / BALL_LON), z))
                for j in range(BALL_LON)]
        rings.append(row)

    for i in range(BALL_LAT):
        for j in range(BALL_LON):
            nj = (j + 1) % BALL_LON
            bm.faces.new([rings[i][j], rings[i][nj], rings[i + 1][nj], rings[i + 1][j]])

    bot = bm.verts.new((0.0, 0.0, -z_cut))
    for j in range(BALL_LON):
        nj = (j + 1) % BALL_LON
        bm.faces.new([bot, rings[0][nj], rings[0][j]])

    top = bm.verts.new((0.0, 0.0, z_cut))
    for j in range(BALL_LON):
        nj = (j + 1) % BALL_LON
        bm.faces.new([top, rings[-1][j], rings[-1][nj]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return _link_mesh(context, bm, name, location)


def _link_mesh(context, bm, name, location):
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    obj.location = location
    context.collection.objects.link(obj)
    return obj


# ── Operator ───────────────────────────────────────────────────────────────────

class OBJECT_OT_add_ball_bearing(bpy.types.Operator):
    """Add a ball bearing: inner race, outer race, and truncated-sphere balls.
    Bore ID and outer OD are primary inputs. Pitch diameter is derived from ball
    packing. Inner and outer wall thicknesses are shown as read-only and must
    both be at least 0.8 mm."""
    bl_idname  = "object.add_ball_bearing"
    bl_label   = "Add Ball Bearing"
    bl_options = {'REGISTER', 'UNDO'}

    # ── Diameter inputs ────────────────────────────────────────────────────────
    bore_diameter_mm: FloatProperty(
        name="Bore ID (mm)", default=6.0, min=0.1, soft_max=500.0,
        description="Inner diameter of the inner race (axle bore)",
    )
    outer_diameter_mm: FloatProperty(
        name="Outer OD (mm)", default=28.0, min=0.2, soft_max=600.0,
        description="Outer diameter of the outer race",
    )

    # ── Balls ──────────────────────────────────────────────────────────────────
    ball_sizing_mode: EnumProperty(
        name="Ball Sizing",
        items=[
            ('RADIUS', "Ball Radius", "Specify ball radius — height shown as read-only"),
            ('HEIGHT', "Ball Height", "Specify truncated height — radius shown as read-only"),
        ],
        default='RADIUS',
    )
    ball_radius_mm: FloatProperty(
        name="Ball Radius (mm)", default=3.0, soft_min=0.5, soft_max=50.0,
    )
    ball_height_mm: FloatProperty(
        name="Ball Height (mm)", default=5.196, soft_min=0.1, soft_max=100.0,
        description="Flat-to-flat height of the truncated ball",
    )
    overhang_angle_deg: FloatProperty(
        name="Overhang Angle (deg)", default=30.0, min=1.0, max=89.0,
        description="Maximum printable overhang from horizontal. "
                    "Ball caps are cut at this latitude — no supports needed.",
    )

    # ── Packing ────────────────────────────────────────────────────────────────
    ball_count: IntProperty(
        name="Ball Count", default=8, min=3, soft_min=6, soft_max=40,
        description="Number of balls packed around the race. "
                    "Pitch diameter is derived from this — more balls = larger pitch circle.",
    )
    gap_mm: FloatProperty(
        name="Ball Gap (mm)", default=0.5, min=0.05, soft_max=5.0,
        description="Clearance between adjacent ball surfaces",
    )

    # ── Fit ────────────────────────────────────────────────────────────────────
    clearance_mm: FloatProperty(
        name="Race Clearance (mm)", default=0.2, min=0.0, soft_max=2.0,
        description="Groove radius = ball_r + clearance/2. "
                    "Increase if the bearing binds in the races after printing.",
    )

    # ── Placement ──────────────────────────────────────────────────────────────
    parent_under_empty: BoolProperty(
        name="Parent Under Empty", default=True,
    )

    def _compute(self):
        overhang_rad = math.radians(self.overhang_angle_deg)
        ball_r, h, z_cut, r_flat = resolve_ball(
            self.ball_sizing_mode, self.ball_radius_mm, self.ball_height_mm, overhang_rad,
        )
        bore_r  = self.bore_diameter_mm / 2.0
        outer_r = self.outer_diameter_mm / 2.0
        ball_center_r, groove_r, inner_wall, outer_wall, bearing_width = derive_bearing(
            ball_r, self.ball_count, self.gap_mm,
            bore_r, outer_r, self.clearance_mm, z_cut,
        )
        # Auto-scale bore/outer to guarantee MIN_WALL_MM on both sides
        bore_r_eff  = bore_r  if inner_wall >= MIN_WALL_MM else max(0.05, ball_center_r - ball_r - MIN_WALL_MM)
        outer_r_eff = outer_r if outer_wall >= MIN_WALL_MM else ball_center_r + groove_r + MIN_WALL_MM
        if bore_r_eff != bore_r or outer_r_eff != outer_r:
            _, _, inner_wall, outer_wall, bearing_width = derive_bearing(
                ball_r, self.ball_count, self.gap_mm,
                bore_r_eff, outer_r_eff, self.clearance_mm, z_cut,
            )
        return (ball_r, h, z_cut, r_flat,
                bore_r_eff, outer_r_eff, ball_center_r, groove_r,
                inner_wall, outer_wall, bearing_width)

    def draw(self, context):
        layout = self.layout
        (ball_r, h, z_cut, r_flat,
         bore_r, outer_r, ball_center_r, groove_r,
         inner_wall, outer_wall, bearing_width) = self._compute()

        # ── Diameters ──────────────────────────────────────────────────────────
        orig_bore_r  = self.bore_diameter_mm / 2.0
        orig_outer_r = self.outer_diameter_mm / 2.0

        box = layout.box()
        box.label(text="Diameters")
        box.prop(self, "bore_diameter_mm")
        if bore_r < orig_bore_r - 1e-4:
            box.label(text="Bore reduced to %.2f mm to maintain %.1f mm inner wall" % (bore_r * 2, MIN_WALL_MM), icon='INFO')
        box.prop(self, "outer_diameter_mm")
        if outer_r > orig_outer_r + 1e-4:
            box.label(text="OD expanded to %.2f mm to maintain %.1f mm outer wall" % (outer_r * 2, MIN_WALL_MM), icon='INFO')

        # ── Balls ──────────────────────────────────────────────────────────────
        box = layout.box()
        box.label(text="Balls")
        box.prop(self, "ball_sizing_mode")
        if self.ball_sizing_mode == 'RADIUS':
            box.prop(self, "ball_radius_mm")
            box.label(text="Flat-to-flat height (derived): %.3f mm" % h)
        else:
            box.prop(self, "ball_height_mm")
            box.label(text="Radius (derived): %.3f mm" % ball_r)
        box.prop(self, "overhang_angle_deg")
        box.label(text="Flat Ø: %.2f mm" % (r_flat * 2))

        # ── Packing ────────────────────────────────────────────────────────────
        box = layout.box()
        box.label(text="Packing")
        box.prop(self, "ball_count")
        box.prop(self, "gap_mm")

        # ── Fit ────────────────────────────────────────────────────────────────
        box = layout.box()
        box.label(text="Fit")
        box.prop(self, "clearance_mm")

        # ── Derived (read-only) ────────────────────────────────────────────────
        box = layout.box()
        box.label(text="Derived")
        box.label(text="Pitch Ø:      %.2f mm" % (ball_center_r * 2))
        box.label(text="Groove Ø:     %.2f mm" % (groove_r * 2))
        box.label(text="Bearing width: %.2f mm" % bearing_width)

        def wall_row(box, label, val):
            icon = 'CHECKMARK' if val >= MIN_WALL_MM else 'ERROR'
            box.label(text="%s %.2f mm" % (label, val), icon=icon)

        wall_row(box, "Inner wall:", inner_wall)
        wall_row(box, "Outer wall:", outer_wall)

        # ── Placement ──────────────────────────────────────────────────────────
        box = layout.box()
        box.label(text="Placement")
        box.prop(self, "parent_under_empty")

    def execute(self, context):
        (ball_r, h, z_cut, r_flat,
         bore_r, outer_r, ball_center_r, groove_r,
         inner_wall, outer_wall, bearing_width) = self._compute()

        loc = tuple(context.scene.cursor.location)
        try:
            inner_obj = build_inner_race(
                context, "BearingInnerRace",
                bore_r=bore_r, ball_center_r=ball_center_r,
                groove_r=groove_r, z_cut=z_cut, location=loc,
            )
            outer_obj = build_outer_race(
                context, "BearingOuterRace",
                ball_center_r=ball_center_r, groove_r=groove_r,
                outer_r=outer_r, z_cut=z_cut, location=loc,
            )
            ball_objs = []
            for i in range(self.ball_count):
                angle    = 2.0 * math.pi * i / self.ball_count
                ball_loc = (
                    loc[0] + ball_center_r * math.cos(angle),
                    loc[1] + ball_center_r * math.sin(angle),
                    loc[2],
                )
                ball_objs.append(
                    build_truncated_sphere(
                        context, "BearingBall.%03d" % i,
                        ball_r=ball_r, z_cut=z_cut, location=ball_loc,
                    )
                )
        except (ValueError, RuntimeError) as e:
            return {'CANCELLED'}

        all_objs = [inner_obj, outer_obj] + ball_objs

        if self.parent_under_empty:
            empty = bpy.data.objects.new("BallBearing", None)
            empty.empty_display_type = 'PLAIN_AXES'
            empty.empty_display_size = outer_r * 0.3
            empty.location           = loc
            context.collection.objects.link(empty)
            for child in all_objs:
                child.parent = empty
                child.matrix_parent_inverse = empty.matrix_world.inverted()

        for o in context.selected_objects:
            o.select_set(False)
        for o in all_objs:
            o.select_set(True)
        context.view_layer.objects.active = inner_obj

        self.report({'INFO'},
            "Ball bearing: bore Ø%.2f mm, OD Ø%.2f mm, %d balls, width %.2f mm"
            % (bore_r * 2, outer_r * 2, self.ball_count, bearing_width))
        return {'FINISHED'}


# ── Registration ───────────────────────────────────────────────────────────────

classes = (OBJECT_OT_add_ball_bearing,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
