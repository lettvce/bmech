"""
Geneva Drive Prototype
──────────────────────
Alt+P to run. Popup dialog for parameter editing.

Converts continuous rotation → intermittent rotation.
Drive wheel rotates continuously; Geneva wheel advances by one step
(360°/n_slots) each revolution of the drive wheel, then locks.

Geometry
────────
  Geneva wheel: 2D star profile (n radial slots with semicircular bottoms)
  revolved into a solid by building top/bottom faces with triangle_fill
  and quad side walls.

  Drive wheel: disk + pin cylinder joined with Boolean UNION.
  Pin is placed at crank_r from disk center. When it enters a Geneva slot,
  the slot geometry forces the Geneva wheel to rotate exactly 360°/n_slots.

Key math
────────
  alpha      = pi/n                  half-angle at Geneva center at entry
  d          = crank_r / sin(alpha)  center distance (drive ↔ Geneva axes)
  r_slot     = crank_r / tan(alpha)  Geneva-center to pin-center at deepest
  slot_hw    = pin_r + clearance     half-width of each slot
  R_geneva   = r_slot + slot_hw + margin   Geneva wheel outer radius

Drive disk radius is bounded so the disk body cannot collide with the
Geneva wheel outer edge between pin engagements:
  r_disk_max = d - R_geneva - clearance
"""

import bpy
import bmesh
from bpy.props import IntProperty, FloatProperty, BoolProperty
from math import pi, sin, cos, tan, atan2

BOOL_EPSILON = 0.001


# ── Low-level helpers ─────────────────────────────────────────────────────────

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


def _disk_bm(bm, outer_r, bore_r, z0, z1, n_outer=64, n_bore=32):
    """Add annular (or solid) disk geometry to bm between z0 and z1."""
    angs_o = [2 * pi * i / n_outer for i in range(n_outer)]
    vb = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), z0)) for a in angs_o]
    vt = [bm.verts.new((outer_r * cos(a), outer_r * sin(a), z1)) for a in angs_o]
    bm.verts.index_update()

    if bore_r > 0.0:
        angs_b = [2 * pi * i / n_bore for i in range(n_bore)]
        vbb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z0)) for a in angs_b]
        vtb = [bm.verts.new((bore_r * cos(a), bore_r * sin(a), z1)) for a in angs_b]
        bm.verts.index_update()

        eb  = [bm.edges.new((vb[i],  vb[(i+1) % n_outer])) for i in range(n_outer)]
        ebb = [bm.edges.new((vbb[i], vbb[(i+1) % n_bore])) for i in range(n_bore)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=eb + ebb, normal=(0, 0, -1))

        et  = [bm.edges.new((vt[i],  vt[(i+1) % n_outer])) for i in range(n_outer)]
        etb = [bm.edges.new((vtb[i], vtb[(i+1) % n_bore])) for i in range(n_bore)]
        bm.edges.index_update()
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=et + etb, normal=(0, 0, 1))

        for i in range(n_bore):
            ni = (i + 1) % n_bore
            bm.faces.new([vbb[ni], vbb[i], vtb[i], vtb[ni]])
    else:
        bm.faces.new(list(reversed(vb)))
        bm.faces.new(vt)

    for i in range(n_outer):
        ni = (i + 1) % n_outer
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])


def _cyl_bm(bm, r, cx, cy, z0, z1, n=32):
    """Solid cylinder at (cx,cy), z0→z1, added to bm."""
    angs = [2 * pi * i / n for i in range(n)]
    vb = [bm.verts.new((cx + r * cos(a), cy + r * sin(a), z0)) for a in angs]
    vt = [bm.verts.new((cx + r * cos(a), cy + r * sin(a), z1)) for a in angs]
    bm.verts.index_update()
    bm.faces.new(list(reversed(vb)))
    bm.faces.new(vt)
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])


def _bool_op(context, target, cutter, operation='DIFFERENCE'):
    mod = target.modifiers.new("BoolOp", "BOOLEAN")
    mod.operation = operation
    mod.solver    = 'EXACT'
    mod.object    = cutter
    with context.temp_override(active_object=target):
        bpy.ops.object.modifier_apply(modifier="BoolOp")
    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Geneva wheel ──────────────────────────────────────────────────────────────

def _geneva_profile(n, crank_r, pin_r, clearance, margin=2.0,
                    outer_segs=12, slot_segs=8):
    """
    2D CCW polygon for one face of the Geneva wheel.
    For each slot: right wall → semicircle bottom → left wall → outer arc to next slot.
    """
    alpha    = pi / n
    r_slot   = crank_r / tan(alpha)        # slot reach from Geneva center
    slot_hw  = pin_r + clearance            # half slot width (radial + print gap)
    r_inner  = r_slot - slot_hw             # radius of slot-bottom semicircle center
    R        = r_slot + slot_hw + margin    # Geneva wheel outer radius

    pts = []
    for k in range(n):
        θ  = 2 * pi * k / n
        rx, ry = cos(θ), sin(θ)            # radial outward unit vector
        tx, ty = -sin(θ), cos(θ)           # tangential CCW unit vector

        # Right slot wall
        pts.append((R * rx - slot_hw * tx,       R * ry - slot_hw * ty))
        pts.append((r_inner * rx - slot_hw * tx, r_inner * ry - slot_hw * ty))

        # Semicircle at slot bottom (CW sweep, right side → left side)
        bc = (r_inner * rx, r_inner * ry)
        for s in range(1, slot_segs):
            t = s / slot_segs
            a = θ - pi / 2 - t * pi
            pts.append((bc[0] + slot_hw * cos(a), bc[1] + slot_hw * sin(a)))

        # Left slot wall
        pts.append((r_inner * rx + slot_hw * tx, r_inner * ry + slot_hw * ty))
        pts.append((R * rx + slot_hw * tx,       R * ry + slot_hw * ty))

        # Outer arc to next slot's right-outer corner
        θ_next   = 2 * pi * (k + 1) / n
        rx_n, ry_n = cos(θ_next), sin(θ_next)
        tx_n, ty_n = -sin(θ_next), cos(θ_next)

        lo   = (R * rx + slot_hw * tx,       R * ry + slot_hw * ty)
        ro_n = (R * rx_n - slot_hw * tx_n,   R * ry_n - slot_hw * ty_n)

        ang_lo   = atan2(lo[1],   lo[0])
        ang_ro_n = atan2(ro_n[1], ro_n[0])
        if ang_ro_n <= ang_lo:
            ang_ro_n += 2 * pi

        for s in range(1, outer_segs):
            t = s / outer_segs
            a = ang_lo + t * (ang_ro_n - ang_lo)
            pts.append((R * cos(a), R * sin(a)))

    return pts, R


def build_geneva_wheel(context, n, crank_r, pin_r, clearance, thickness, bore_r,
                        name="GenevaWheel", location=(0, 0, 0)):
    pts, R = _geneva_profile(n, crank_r, pin_r, clearance)
    n_pts  = len(pts)

    bm = bmesh.new()
    vb = [bm.verts.new((x, y, 0.0))       for x, y in pts]
    vt = [bm.verts.new((x, y, thickness)) for x, y in pts]
    bm.verts.index_update()

    eb = [bm.edges.new((vb[i], vb[(i + 1) % n_pts])) for i in range(n_pts)]
    et = [bm.edges.new((vt[i], vt[(i + 1) % n_pts])) for i in range(n_pts)]
    bm.edges.index_update()

    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                             edges=eb, normal=(0, 0, -1))
    bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                             edges=et, normal=(0, 0, 1))

    for i in range(n_pts):
        ni = (i + 1) % n_pts
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])

    gw_obj = _make_obj(context, bm, name, location)

    if bore_r > 0:
        bm_c = bmesh.new()
        _cyl_bm(bm_c, bore_r, 0, 0, -BOOL_EPSILON, thickness + 2 * BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_BoreCut", location)
        _bool_op(context, gw_obj, cutter, 'DIFFERENCE')

    return gw_obj, R


# ── Drive wheel ───────────────────────────────────────────────────────────────

def build_drive_wheel(context, n, crank_r, pin_r, clearance, thickness,
                       pin_height, bore_r, R_geneva,
                       name="DriveWheel", location=(0, 0, 0)):
    alpha  = pi / n
    d      = crank_r / sin(alpha)
    # Disk radius: must not hit Geneva wheel outer edge between engagements
    r_disk = max(d - R_geneva - clearance - 0.5, 2.0)

    # Drive disk
    bm_disk = bmesh.new()
    _disk_bm(bm_disk, r_disk, bore_r, 0.0, thickness)
    disk_obj = _make_obj(context, bm_disk, name, location)

    # Pin cylinder (protrudes above the disk)
    bm_pin = bmesh.new()
    _cyl_bm(bm_pin, pin_r, crank_r, 0.0, -BOOL_EPSILON, thickness + pin_height)
    pin_obj = _make_obj(context, bm_pin, name + "_Pin", location)

    # Union pin into disk
    _bool_op(context, disk_obj, pin_obj, 'UNION')

    if bore_r > 0:
        bm_c = bmesh.new()
        _cyl_bm(bm_c, bore_r, 0, 0, -BOOL_EPSILON, thickness + pin_height + BOOL_EPSILON)
        cutter = _make_obj(context, bm_c, name + "_BoreCut", location)
        _bool_op(context, disk_obj, cutter, 'DIFFERENCE')

    return disk_obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_geneva_drive(bpy.types.Operator):
    """Geneva drive — intermittent rotation mechanism."""
    bl_idname  = "object.geneva_drive"
    bl_label   = "Geneva Drive"
    bl_options = {'REGISTER', 'UNDO'}

    n_slots:    IntProperty(  name="Slots (n)",          default=4,    min=3,    max=12,
                               description="Number of Geneva wheel slots. 3=120°/step, 4=90°/step, 6=60°/step")
    crank_r:    FloatProperty(name="Crank Radius (mm)",  default=20.0, min=3.0,  soft_max=100.0,
                               description="Distance from drive wheel center to pin center")
    pin_r:      FloatProperty(name="Pin Radius (mm)",    default=2.0,  min=0.5,  soft_max=10.0,
                               description="Radius of the drive pin")
    clearance:  FloatProperty(name="Clearance (mm)",     default=0.25, min=0.05, soft_max=1.0,
                               description="Radial gap between pin and slot walls — tune per printer")
    thickness:  FloatProperty(name="Thickness (mm)",     default=6.0,  min=1.0,  soft_max=30.0,
                               description="Disk thickness — applies to both wheels")
    pin_height: FloatProperty(name="Pin Protrusion (mm)", default=4.0, min=1.0,  soft_max=20.0,
                               description="How far the pin extends above the drive disk face")
    bore_d:     FloatProperty(name="Axle Bore Ø (mm)",   default=5.0,  min=0.0,  soft_max=30.0,
                               description="Bore diameter for both wheels. 0 = no bore")
    bore_comp:  FloatProperty(name="Bore Compensation (mm)", default=0.2, min=0.0, soft_max=1.0,
                               description="Added to bore radius for FDM shrinkage")

    def _derived(self):
        alpha   = pi / self.n_slots
        d       = self.crank_r / sin(alpha)
        r_slot  = self.crank_r / tan(alpha)
        slot_hw = self.pin_r + self.clearance
        R_g     = r_slot + slot_hw + 2.0
        r_disk  = max(d - R_g - self.clearance - 0.5, 2.0)
        bore_r  = (self.bore_d / 2.0 + self.bore_comp) if self.bore_d > 0 else 0.0
        step_deg = 360.0 / self.n_slots
        return alpha, d, r_slot, slot_hw, R_g, r_disk, bore_r, step_deg

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        alpha, d, r_slot, slot_hw, R_g, r_disk, bore_r, step_deg = self._derived()

        col = layout.column(align=True)
        col.prop(self, "n_slots")
        col.prop(self, "crank_r")
        col.prop(self, "pin_r")
        col.prop(self, "clearance")
        col.prop(self, "thickness")
        col.prop(self, "pin_height")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "bore_d")
        col.prop(self, "bore_comp")

        layout.separator()
        box = layout.box()
        box.label(text="Center distance:    %.2f mm" % d)
        box.label(text="Geneva wheel OD:    %.2f mm" % (R_g * 2))
        box.label(text="Drive disk radius:  %.2f mm" % r_disk)
        box.label(text="Step angle:         %.1f °"  % step_deg)
        box.label(text="Slot reach:         %.2f mm" % r_slot)

        if self.pin_r * 2 >= slot_hw * 2:
            layout.label(text="Pin fills slot — increase clearance", icon='ERROR')
        if bore_r >= r_disk:
            layout.label(text="Bore too large for drive disk", icon='ERROR')
        if bore_r >= r_slot - slot_hw:
            layout.label(text="Bore too large for Geneva wheel slot depth", icon='ERROR')
        if r_disk <= 0:
            layout.label(text="Crank radius too large — no room for drive disk", icon='ERROR')

    def execute(self, context):
        alpha, d, r_slot, slot_hw, R_g, r_disk, bore_r, step_deg = self._derived()

        if r_disk <= 0 or (bore_r > 0 and bore_r >= r_disk):
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()

        # Geneva wheel at cursor
        gw_obj, R_geneva = build_geneva_wheel(
            context, self.n_slots, self.crank_r, self.pin_r, self.clearance,
            self.thickness, bore_r,
            name="GenevaWheel", location=cursor,
        )

        # Drive wheel offset along +X by center distance
        drive_loc = cursor.copy()
        drive_loc.x += d
        dw_obj = build_drive_wheel(
            context, self.n_slots, self.crank_r, self.pin_r, self.clearance,
            self.thickness, self.pin_height, bore_r, R_geneva,
            name="DriveWheel", location=drive_loc,
        )

        bpy.ops.object.select_all(action='DESELECT')
        gw_obj.select_set(True)
        dw_obj.select_set(True)
        context.view_layer.objects.active = gw_obj

        self.report({'INFO'},
            "Geneva drive: %d slots, %.1f° per step, center dist %.2f mm"
            % (self.n_slots, step_deg, d))
        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_geneva_drive)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_geneva_drive)
bpy.ops.object.geneva_drive('INVOKE_DEFAULT')
