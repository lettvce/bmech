"""
Bayonet Container Prototype
────────────────────────────
Alt+P to run. Popup dialog for parameter editing.

Lid slides OVER the container and rotates to lock via L-shaped channels.

Assembly
────────
  1. Align lid lugs with container entry slots (same angular positions)
  2. Push lid straight down — lugs travel down the vertical entry slots
  3. Rotate lid CCW (from above) by lock_angle — lugs travel into horizontal grooves
  4. Optional detent bump gives a tactile click at the locked position
  Reverse to remove.

Generated objects
─────────────────
  BayonetContainer  — cylindrical body, closed bottom, N channels near top
  BayonetLid        — cap, closed top, N inward-protruding lugs near bottom

Channel geometry (cut into container outer wall)
────────────────────────────────────────────────
  Entry slot:     vertical, full lug_height radial depth, entry_depth tall
  Locking groove: horizontal (CCW), same radial depth, lug_depth + clearance tall,
                  extends lock_angle degrees from the entry slot right edge
  Detent bump:    small positive arc at groove end — lug must snap over it

Lug geometry (on lid inner wall)
──────────────────────────────────
  Protrudes inward (toward center) by lug_height from lid inner wall.
  Sits near the lid bottom so it engages the container channels when pressed down.

Clearance guide
───────────────
  0.2 mm — tight press fit (may need flexing to assemble)
  0.3 mm — good running fit for most printers
  0.4 mm — loose, easy assembly, less retention

Wall and lug sizing constraint
───────────────────────────────
  wall_thickness must be > lug_height + 1 mm (channel can't eat through the wall)
"""

import bpy
import bmesh
from bpy.props import IntProperty, FloatProperty, BoolProperty
from math import pi, cos, sin

BOOL_EPSILON = 0.001


# ── Low-level helpers ─────────────────────────────────────────────────────────

def _make_obj(context, bm, name, location):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    obj.location = location
    context.collection.objects.link(obj)
    return obj


def _bool_op(context, target, cutter, operation='DIFFERENCE'):
    mod = target.modifiers.new("BoolOp", "BOOLEAN")
    mod.operation = operation
    mod.solver    = 'EXACT'
    mod.object    = cutter
    with context.temp_override(active_object=target):
        bpy.ops.object.modifier_apply(modifier="BoolOp")
    bpy.data.objects.remove(cutter, do_unlink=True)


def _arc_solid(bm, r_inner, r_outer, z_bot, z_top, ang_start, ang_end, n_segs=16):
    """Annular sector solid — used for both cutters and lug additions."""
    angs = [ang_start + (ang_end - ang_start) * i / n_segs for i in range(n_segs + 1)]
    n    = n_segs

    vbi = [bm.verts.new((r_inner * cos(a), r_inner * sin(a), z_bot)) for a in angs]
    vbo = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), z_bot)) for a in angs]
    vti = [bm.verts.new((r_inner * cos(a), r_inner * sin(a), z_top)) for a in angs]
    vto = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), z_top)) for a in angs]
    bm.verts.index_update()

    for i in range(n):
        ni = i + 1
        bm.faces.new([vbi[i],  vbo[i],  vbo[ni], vbi[ni]])   # bottom
        bm.faces.new([vti[ni], vto[ni], vto[i],  vti[i]])    # top
        bm.faces.new([vbo[i],  vto[i],  vto[ni], vbo[ni]])   # outer wall
        bm.faces.new([vbi[ni], vti[ni], vti[i],  vbi[i]])    # inner wall

    bm.faces.new([vbo[0], vbi[0], vti[0], vto[0]])    # start cap
    bm.faces.new([vbi[n], vbo[n], vto[n], vti[n]])    # end cap


def _cylinder_shell(bm, r_outer, r_inner, z_bot, z_top,
                    close_bottom=True, close_top=False, n_segs=128):
    """Annular cylindrical shell."""
    angs = [2 * pi * i / n_segs for i in range(n_segs)]
    n    = n_segs

    vbo = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), z_bot)) for a in angs]
    vto = [bm.verts.new((r_outer * cos(a), r_outer * sin(a), z_top)) for a in angs]
    vbi = [bm.verts.new((r_inner * cos(a), r_inner * sin(a), z_bot)) for a in angs]
    vti = [bm.verts.new((r_inner * cos(a), r_inner * sin(a), z_top)) for a in angs]
    bm.verts.index_update()

    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vbo[i], vbo[ni], vto[ni], vto[i]])          # outer wall
        bm.faces.new([vbi[ni], vbi[i], vti[i], vti[ni]])          # inner wall

    if close_bottom:
        # Wall quads already created these edges — retrieve, don't re-create
        eb_o = [bm.edges.get((vbo[i], vbo[(i+1) % n])) for i in range(n)]
        eb_i = [bm.edges.get((vbi[i], vbi[(i+1) % n])) for i in range(n)]
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=eb_o + eb_i, normal=(0, 0, -1))

    if close_top:
        et_o = [bm.edges.get((vto[i], vto[(i+1) % n])) for i in range(n)]
        et_i = [bm.edges.get((vti[i], vti[(i+1) % n])) for i in range(n)]
        bmesh.ops.triangle_fill(bm, use_beauty=True, use_dissolve=True,
                                 edges=et_o + et_i, normal=(0, 0, 1))


# ── Object builders ───────────────────────────────────────────────────────────

def build_container(context, R_out, R_in, height, n_lugs,
                    slot_arc_half, lock_rad, lug_h, entry_depth,
                    groove_z_h, detent_h, name, location):
    bm = bmesh.new()
    _cylinder_shell(bm, R_out, R_in, 0.0, height,
                    close_bottom=True, close_top=False)
    obj = _make_obj(context, bm, name, location)

    for k in range(n_lugs):
        θ = 2 * pi * k / n_lugs

        # Vertical entry slot (opens at container top, goes down by entry_depth)
        bm_c = bmesh.new()
        _arc_solid(bm_c,
                   R_out - lug_h,
                   R_out + BOOL_EPSILON,
                   height - entry_depth,
                   height + BOOL_EPSILON,
                   θ - slot_arc_half,
                   θ + slot_arc_half,
                   n_segs=8)
        _bool_op(context, obj, _make_obj(context, bm_c, name + "_SC", location))

        # Horizontal locking groove (CCW from right edge of entry slot)
        groove_ang_start = θ + slot_arc_half
        groove_ang_end   = groove_ang_start + lock_rad

        bm_c = bmesh.new()
        _arc_solid(bm_c,
                   R_out - lug_h,
                   R_out + BOOL_EPSILON,
                   height - entry_depth - groove_z_h,
                   height - entry_depth + BOOL_EPSILON,
                   groove_ang_start,
                   groove_ang_end,
                   n_segs=16)
        _bool_op(context, obj, _make_obj(context, bm_c, name + "_GC", location))

        # Detent bump at groove end (union a small ridge back in)
        if detent_h > 0:
            bm_c = bmesh.new()
            _arc_solid(bm_c,
                       R_out - lug_h,
                       R_out + BOOL_EPSILON,
                       height - entry_depth - detent_h,
                       height - entry_depth + BOOL_EPSILON,
                       groove_ang_end - slot_arc_half,
                       groove_ang_end,
                       n_segs=8)
            _bool_op(context, obj,
                     _make_obj(context, bm_c, name + "_Det", location), 'UNION')

    return obj


def build_lid(context, R_lid_out, R_lid_in, lid_h, n_lugs,
              slot_arc_half, lug_h, lug_depth, clearance, name, location):
    bm = bmesh.new()
    _cylinder_shell(bm, R_lid_out, R_lid_in, 0.0, lid_h,
                    close_bottom=False, close_top=True)
    obj = _make_obj(context, bm, name, location)

    # Lugs near lid bottom (they engage the container entry slots first)
    lug_z_bot = clearance
    lug_z_top = clearance + lug_depth

    for k in range(n_lugs):
        θ = 2 * pi * k / n_lugs
        bm_c = bmesh.new()
        _arc_solid(bm_c,
                   R_lid_in - lug_h,
                   R_lid_in + BOOL_EPSILON,
                   lug_z_bot,
                   lug_z_top,
                   θ - slot_arc_half,
                   θ + slot_arc_half,
                   n_segs=8)
        _bool_op(context, obj,
                 _make_obj(context, bm_c, name + "_Lug", location), 'UNION')

    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_bayonet_container(bpy.types.Operator):
    """Bayonet-mount container — push down and rotate lid to lock."""
    bl_idname  = "object.bayonet_container"
    bl_label   = "Bayonet Container"
    bl_options = {'REGISTER', 'UNDO'}

    container_od:    FloatProperty(name="Container OD (mm)",     default=60.0, min=10.0, soft_max=200.0,
                                    description="Outer diameter of the container body")
    container_h:     FloatProperty(name="Container Height (mm)", default=50.0, min=10.0, soft_max=300.0)
    wall_t:          FloatProperty(name="Wall Thickness (mm)",   default=3.0,  min=1.0,  soft_max=10.0)
    lid_h:           FloatProperty(name="Lid Height (mm)",       default=20.0, min=8.0,  soft_max=80.0)
    n_lugs:          IntProperty(  name="Lugs",                  default=3,    min=2,    max=6,
                                    description="2 = 180° turn, 3 = 120° turn (more secure), 4 = 90° turn")
    lug_width:       FloatProperty(name="Lug Width (mm)",        default=8.0,  min=2.0,  soft_max=30.0,
                                    description="Circumferential arc length of each lug at container surface")
    lug_height:      FloatProperty(name="Lug Height (mm)",       default=2.0,  min=0.5,  soft_max=6.0,
                                    description="Radial protrusion of lug into channel")
    lug_depth:       FloatProperty(name="Lug Depth (mm)",        default=4.0,  min=1.0,  soft_max=15.0,
                                    description="Axial (Z) height of the lug and locking groove")
    lock_angle:      FloatProperty(name="Lock Angle (°)",        default=40.0, min=10.0, max=170.0,
                                    description="Rotation angle to reach locked position. Max ≈ 360/n_lugs − lug_arc")
    clearance:       FloatProperty(name="Clearance (mm)",        default=0.3,  min=0.1,  soft_max=1.0,
                                    description="Running fit gap — tune per printer")
    detent_h:        FloatProperty(name="Detent Height (mm)",    default=0.3,  min=0.0,  soft_max=1.0,
                                    description="Click bump height at groove end. 0 = no detent. Keep ≤ 0.4 mm for rigid materials.")

    def _derived(self):
        R_out        = self.container_od / 2.0
        R_in         = R_out - self.wall_t
        R_lid_in     = R_out + self.clearance
        R_lid_out    = R_lid_in + self.wall_t

        lug_arc_rad  = self.lug_width / R_out
        cl_arc_rad   = self.clearance / R_out
        slot_arc_half= (lug_arc_rad + cl_arc_rad) / 2.0

        lock_rad     = self.lock_angle * pi / 180.0
        entry_depth  = self.lug_depth + self.clearance * 2 + 1.0
        groove_z_h   = self.lug_depth + self.clearance

        max_lock_deg = 360.0 / self.n_lugs - (lug_arc_rad * 180.0 / pi)

        ok_wall    = self.wall_t > self.lug_height + 1.0
        ok_lock    = self.lock_angle < max_lock_deg
        ok_entry   = entry_depth < self.container_h - 5.0
        ok_lid_lug = self.lug_depth + self.clearance * 2 < self.lid_h - 2.0
        ok_bore    = R_in > 5.0

        return (R_out, R_in, R_lid_in, R_lid_out, slot_arc_half, lock_rad,
                entry_depth, groove_z_h, max_lock_deg,
                ok_wall, ok_lock, ok_entry, ok_lid_lug, ok_bore)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=320)

    def draw(self, context):
        layout = self.layout
        (R_out, R_in, R_lid_in, R_lid_out, slot_arc_half, lock_rad,
         entry_depth, groove_z_h, max_lock_deg,
         ok_wall, ok_lock, ok_entry, ok_lid_lug, ok_bore) = self._derived()

        col = layout.column(align=True)
        col.label(text="Container")
        col.prop(self, "container_od")
        col.prop(self, "container_h")
        col.prop(self, "wall_t")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Lid")
        col.prop(self, "lid_h")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Bayonet")
        col.prop(self, "n_lugs")
        col.prop(self, "lug_width")
        col.prop(self, "lug_height")
        col.prop(self, "lug_depth")
        col.prop(self, "lock_angle")
        col.prop(self, "clearance")
        col.prop(self, "detent_h")

        layout.separator()
        box = layout.box()
        box.label(text="Container ID:    %.1f mm" % (R_in * 2))
        box.label(text="Lid OD:          %.1f mm" % (R_lid_out * 2))
        box.label(text="Entry depth:     %.1f mm" % entry_depth)
        box.label(text="Max lock angle:  %.1f °"  % max_lock_deg)
        box.label(text="Turn to lock:    %.1f °"  % self.lock_angle)

        if not ok_wall:
            layout.label(text="Wall too thin — must be > lug_height + 1 mm", icon='ERROR')
        if not ok_lock:
            layout.label(text="Lock angle too large — lugs from adjacent channels overlap", icon='ERROR')
        if not ok_entry:
            layout.label(text="Container too short for entry depth", icon='ERROR')
        if not ok_lid_lug:
            layout.label(text="Lid too short for lug + clearance", icon='ERROR')
        if not ok_bore:
            layout.label(text="Container inner diameter very small", icon='INFO')

    def execute(self, context):
        (R_out, R_in, R_lid_in, R_lid_out, slot_arc_half, lock_rad,
         entry_depth, groove_z_h, max_lock_deg,
         ok_wall, ok_lock, ok_entry, ok_lid_lug, ok_bore) = self._derived()

        if not (ok_wall and ok_lock and ok_entry and ok_lid_lug):
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()
        loc    = tuple(cursor)

        container_obj = build_container(
            context, R_out, R_in, self.container_h, self.n_lugs,
            slot_arc_half, lock_rad, self.lug_height, entry_depth,
            groove_z_h, self.detent_h,
            "BayonetContainer", loc,
        )

        # Lid placed to the side for a clear view of both parts
        lid_offset = R_lid_out + R_out + 15.0
        lid_loc    = (loc[0] + lid_offset, loc[1], loc[2])

        lid_obj = build_lid(
            context, R_lid_out, R_lid_in, self.lid_h, self.n_lugs,
            slot_arc_half, self.lug_height, self.lug_depth, self.clearance,
            "BayonetLid", lid_loc,
        )

        bpy.ops.object.select_all(action='DESELECT')
        container_obj.select_set(True)
        lid_obj.select_set(True)
        context.view_layer.objects.active = container_obj

        self.report({'INFO'},
            "Bayonet container: %.0f mm OD, %d lugs, %.0f° to lock"
            % (self.container_od, self.n_lugs, self.lock_angle))
        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_bayonet_container)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_bayonet_container)
bpy.ops.object.bayonet_container('INVOKE_DEFAULT')
