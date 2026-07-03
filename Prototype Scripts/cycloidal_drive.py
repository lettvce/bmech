"""
Cycloidal Drive Prototype
─────────────────────────
Alt+P to run. Popup dialog for parameter editing.

Topology
────────
  Input shaft → eccentric cam → cycloidal disk (orbits + counter-rotates)
  → output pins through disk holes → output shaft

  All three shafts (input, ring, output) are COAXIAL. The entire reduction
  happens in a single stage. Reduction ratio = N (ring pin count).

Components generated
────────────────────
  CycloidalDisk   — the disk whose teeth mesh with the ring pins
  RingHousing     — housing plate with N through-holes for ring pins
  RingPin_k       — N separate pin cylinders (insert metal rods or print)

Disk profile math
─────────────────
  The disk teeth are the offset curve of the ring-pin pitch curve (hypotrochoid):
    X(t) = R·cos(t) − e·cos(N·t)
    Y(t) = R·sin(t) − e·sin(N·t)

  where N = ring pin count, R = ring pin pitch radius, e = eccentricity.
  This curve has N−1 lobes (= disk tooth count) and N−1 valleys.

  The actual disk surface is offset inward by rp (ring pin radius) along the
  inward normal at each point:
    nx = −(dY/dt) / |T|,  ny = (dX/dt) / |T|
    profile(t) = (X + rp·nx,  Y + rp·ny)

  Disk profile radius ranges from ≈ R−e−rp (valleys) to ≈ R+e−rp (lobe tips).

Output pin holes
────────────────
  Output pins are coaxial with the ring (at ring center), at radius r_out_pitch.
  As the disk orbits eccentrically (radius e), the hole centers in the disk
  trace circles of radius e — so each hole must be oversized by e + clearance:
    out_hole_radius = out_pin_radius + e + clearance

Profile quality constraint
──────────────────────────
  The offset curve develops cusps (self-intersections) if the offset distance
  rp exceeds the pitch curve's minimum radius of curvature. A safe practical
  rule: rp < e · (N−1). If violated, reduce rp or increase e.
"""

import bpy
import bmesh
from bpy.props import IntProperty, FloatProperty, BoolProperty
from math import pi, sin, cos, sqrt

BOOL_EPSILON = 0.001


# ── Utilities ─────────────────────────────────────────────────────────────────

def _make_obj(context, bm, name, location=(0, 0, 0)):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    obj.location = location
    context.collection.objects.link(obj)
    return obj


def _cyl_bm(bm, r, cx, cy, z0, z1, n=32):
    angs = [2 * pi * i / n for i in range(n)]
    vb = [bm.verts.new((cx + r * cos(a), cy + r * sin(a), z0)) for a in angs]
    vt = [bm.verts.new((cx + r * cos(a), cy + r * sin(a), z1)) for a in angs]
    bm.verts.index_update()
    bm.faces.new(list(reversed(vb)))
    bm.faces.new(vt)
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])


def _bool_diff(context, target, cutter):
    mod = target.modifiers.new("BoolDiff", "BOOLEAN")
    mod.operation = 'DIFFERENCE'
    mod.solver    = 'EXACT'
    mod.object    = cutter
    with context.temp_override(active_object=target):
        bpy.ops.object.modifier_apply(modifier="BoolDiff")
    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Cycloidal disk profile ────────────────────────────────────────────────────

def cycloidal_profile(N, R, e, rp, n_pts=360):
    """
    Offset curve of the ring-pin pitch hypotrochoid.
    Returns list of (x, y) forming the disk's 2D tooth profile (CCW).
    """
    pts = []
    for i in range(n_pts):
        t  = 2 * pi * i / n_pts
        X  = R * cos(t) - e * cos(N * t)
        Y  = R * sin(t) - e * sin(N * t)
        dX = -R * sin(t) + N * e * sin(N * t)
        dY =  R * cos(t) - N * e * cos(N * t)
        mag = sqrt(dX * dX + dY * dY)
        if mag < 1e-9:
            continue
        # Inward normal of the CCW pitch curve
        nx = -dY / mag
        ny =  dX / mag
        pts.append((X + rp * nx, Y + rp * ny))
    return pts


# ── Object builders ───────────────────────────────────────────────────────────

def build_cycloidal_disk(context, N, R, e, rp, clearance, thickness,
                          n_out, r_out_pin, r_out_pitch, bore_r,
                          n_pts, name, location):
    pts = cycloidal_profile(N, R, e, rp, n_pts)
    n   = len(pts)

    bm = bmesh.new()
    vb = [bm.verts.new((x, y, 0.0))       for x, y in pts]
    vt = [bm.verts.new((x, y, thickness)) for x, y in pts]
    bm.verts.index_update()

    eb = [bm.edges.new((vb[i], vb[(i + 1) % n])) for i in range(n)]
    et = [bm.edges.new((vt[i], vt[(i + 1) % n])) for i in range(n)]
    bm.edges.index_update()

    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                             edges=eb, normal=(0, 0, -1))
    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                             edges=et, normal=(0, 0, 1))

    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])

    disk_obj = _make_obj(context, bm, name, location)

    # Output pin holes — oversized by eccentricity to accommodate orbit
    out_hole_r = r_out_pin + e + clearance
    for k in range(n_out):
        θ = 2 * pi * k / n_out
        cx = r_out_pitch * cos(θ)
        cy = r_out_pitch * sin(θ)
        bm_c = bmesh.new()
        _cyl_bm(bm_c, out_hole_r, cx, cy, -BOOL_EPSILON, thickness + 2 * BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_OutHole", location)
        _bool_diff(context, disk_obj, cutter)

    # Eccentric bore (input shaft cam passes through here)
    if bore_r > 0:
        bm_c = bmesh.new()
        _cyl_bm(bm_c, bore_r, 0, 0, -BOOL_EPSILON, thickness + 2 * BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_Bore", location)
        _bool_diff(context, disk_obj, cutter)

    return disk_obj


def build_ring_housing(context, N, R, rp, clearance, thickness, r_outer,
                        bore_r, name, location):
    # Solid housing disk
    bm = bmesh.new()
    n_o = 128
    angs_o = [2 * pi * i / n_o for i in range(n_o)]
    vb = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), 0.0))       for a in angs_o]
    vt = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), thickness)) for a in angs_o]
    bm.verts.index_update()
    bm.faces.new(list(reversed(vb)))
    bm.faces.new(vt)
    for i in range(n_o):
        ni = (i + 1) % n_o
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])

    housing_obj = _make_obj(context, bm, name, location)

    # N ring pin holes
    pin_hole_r = rp + clearance
    for k in range(N):
        θ = 2 * pi * k / N
        cx = R * cos(θ)
        cy = R * sin(θ)
        bm_c = bmesh.new()
        _cyl_bm(bm_c, pin_hole_r, cx, cy, -BOOL_EPSILON, thickness + 2 * BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_PinHole", location)
        _bool_diff(context, housing_obj, cutter)

    # Central shaft bore
    if bore_r > 0:
        bm_c = bmesh.new()
        _cyl_bm(bm_c, bore_r, 0, 0, -BOOL_EPSILON, thickness + 2 * BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_Bore", location)
        _bool_diff(context, housing_obj, cutter)

    return housing_obj


def build_ring_pins(context, N, R, rp, thickness, name, location):
    """N individual pin cylinders placed at the ring pin circle."""
    pin_objs = []
    for k in range(N):
        θ  = 2 * pi * k / N
        cx = R * cos(θ)
        cy = R * sin(θ)
        bm = bmesh.new()
        _cyl_bm(bm, rp, 0, 0, 0.0, thickness)
        pin_loc = (location[0] + cx, location[1] + cy, location[2])
        obj = _make_obj(context, bm, "%s_%02d" % (name, k), pin_loc)
        pin_objs.append(obj)
    return pin_objs


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_cycloidal_drive(bpy.types.Operator):
    """Cycloidal drive — coaxial high-ratio reduction."""
    bl_idname  = "object.cycloidal_drive"
    bl_label   = "Cycloidal Drive"
    bl_options = {'REGISTER', 'UNDO'}

    n_ring_pins:     IntProperty(  name="Ring Pins (N)",           default=11,   min=5,    max=30,
                                    description="Number of ring pins. Reduction ratio = N. Odd numbers reduce vibration.")
    ring_pin_pitch_r:FloatProperty(name="Pin Pitch Radius (mm)",   default=30.0, min=5.0,  soft_max=100.0,
                                    description="Radius of the circle on which ring pin centers sit")
    ring_pin_r:      FloatProperty(name="Pin Radius (mm)",         default=3.0,  min=0.5,  soft_max=10.0,
                                    description="Radius of each ring pin (roller)")
    eccentricity:    FloatProperty(name="Eccentricity (mm)",       default=2.0,  min=0.1,  soft_max=10.0,
                                    description="Offset of disk center from ring center. Controls tooth depth.")
    thickness:       FloatProperty(name="Thickness (mm)",          default=8.0,  min=2.0,  soft_max=40.0,
                                    description="Disk and housing plate thickness")
    clearance:       FloatProperty(name="Clearance (mm)",          default=0.2,  min=0.05, soft_max=1.0,
                                    description="Pin-to-tooth and bore running clearance")
    n_out_pins:      IntProperty(  name="Output Pins",             default=5,    min=2,    max=20,
                                    description="Number of output pins. Must be ≤ (N−1)/2 for clearance.")
    out_pin_r:       FloatProperty(name="Output Pin Radius (mm)",  default=3.0,  min=0.5,  soft_max=10.0,
                                    description="Radius of output pins (pass through disk holes)")
    out_pin_pitch_r: FloatProperty(name="Output Pin Pitch (mm)",   default=12.0, min=1.0,  soft_max=50.0,
                                    description="Radius of output pin circle. Must be inside disk minimum radius.")
    bore_d:          FloatProperty(name="Shaft Bore Ø (mm)",       default=8.0,  min=0.0,  soft_max=30.0,
                                    description="Central bore for input eccentric shaft and output shaft. 0 = none.")
    n_pts:           IntProperty(  name="Profile Resolution",      default=360,  min=60,   max=1440,
                                    description="Number of profile vertices. Higher = smoother but slower.")

    def _derived(self):
        N   = self.n_ring_pins
        R   = self.ring_pin_pitch_r
        e   = self.eccentricity
        rp  = self.ring_pin_r
        cl  = self.clearance

        r_disk_min  = R - e - rp    # approx valley radius
        r_disk_max  = R + e - rp    # approx lobe tip radius
        r_housing   = R + rp + 4.0  # 4 mm wall around pin holes
        out_hole_r  = self.out_pin_r + e + cl
        bore_r      = self.bore_d / 2.0
        ratio       = N

        # Quality check: rp should be < e*(N-1) to avoid offset curve cusps
        cusp_risk = rp >= e * (N - 1)

        # Output pin pitch must be inside disk min radius with room for hole
        out_fits = (self.out_pin_pitch_r + out_hole_r + cl) < r_disk_min

        # Bore must not overlap output holes
        bore_clear = bore_r < (self.out_pin_pitch_r - out_hole_r - cl)

        return (N, R, e, rp, cl, r_disk_min, r_disk_max, r_housing,
                out_hole_r, bore_r, ratio, cusp_risk, out_fits, bore_clear)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=340)

    def draw(self, context):
        layout = self.layout
        (N, R, e, rp, cl, r_disk_min, r_disk_max, r_housing,
         out_hole_r, bore_r, ratio, cusp_risk, out_fits, bore_clear) = self._derived()

        col = layout.column(align=True)
        col.prop(self, "n_ring_pins")
        col.prop(self, "ring_pin_pitch_r")
        col.prop(self, "ring_pin_r")
        col.prop(self, "eccentricity")
        col.prop(self, "thickness")
        col.prop(self, "clearance")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "n_out_pins")
        col.prop(self, "out_pin_r")
        col.prop(self, "out_pin_pitch_r")
        col.prop(self, "bore_d")
        col.prop(self, "n_pts")

        layout.separator()
        box = layout.box()
        box.label(text="Reduction ratio:     %d : 1"    % ratio)
        box.label(text="Disk lobes:          %d"         % (N - 1))
        box.label(text="Disk radius range:   %.1f – %.1f mm" % (r_disk_min, r_disk_max))
        box.label(text="Output hole radius:  %.2f mm"   % out_hole_r)
        box.label(text="Housing outer Ø:     %.1f mm"   % (r_housing * 2))

        if cusp_risk:
            layout.label(
                text="Pin radius may cause cusps — try rp < %.1f mm" % (e * (N - 1)),
                icon='ERROR')
        if not out_fits:
            layout.label(
                text="Output pins don't fit inside disk — reduce out_pin_pitch_r or out_pin_r",
                icon='ERROR')
        if not bore_clear and bore_r > 0:
            layout.label(
                text="Bore overlaps output pin holes — reduce bore or out_pin_pitch_r",
                icon='ERROR')
        if self.n_out_pins > (N - 1) // 2:
            layout.label(
                text="Output pin count high for N — consider ≤ %d" % ((N - 1) // 2),
                icon='INFO')

    def execute(self, context):
        (N, R, e, rp, cl, r_disk_min, r_disk_max, r_housing,
         out_hole_r, bore_r, ratio, cusp_risk, out_fits, bore_clear) = self._derived()

        if not out_fits:
            self.report({'ERROR'}, "Output pins don't fit inside disk minimum radius")
            return {'CANCELLED'}
        if not bore_clear and bore_r > 0:
            self.report({'ERROR'}, "Bore overlaps output pin holes")
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()
        loc    = tuple(cursor)

        # Disk: offset by eccentricity along +X (starting angle = 0)
        disk_loc = (loc[0] + e, loc[1], loc[2])
        disk_obj = build_cycloidal_disk(
            context, N, R, e, rp, cl, self.thickness,
            self.n_out_pins, self.out_pin_r, self.out_pin_pitch_r, bore_r,
            self.n_pts, "CycloidalDisk", disk_loc,
        )

        # Ring housing plate (coaxial with ring center = cursor)
        housing_obj = build_ring_housing(
            context, N, R, rp, cl, self.thickness, r_housing, bore_r,
            "RingHousing", loc,
        )

        # Ring pins (coaxial with ring center)
        pin_objs = build_ring_pins(
            context, N, R, rp, self.thickness, "RingPin", loc,
        )

        bpy.ops.object.select_all(action='DESELECT')
        for o in [disk_obj, housing_obj] + pin_objs:
            o.select_set(True)
        context.view_layer.objects.active = disk_obj

        self.report({'INFO'},
            "Cycloidal drive: %d:1 ratio, %d-lobe disk, %.1f–%.1f mm disk radius"
            % (ratio, N - 1, r_disk_min, r_disk_max))
        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_cycloidal_drive)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_cycloidal_drive)
bpy.ops.object.cycloidal_drive('INVOKE_DEFAULT')
