"""
PiP Track Prototype
────────────────────
Run with Alt+P. Popup dialog for parameters.

Reference: Flexi Track Fidget Toy (MakerWorld #1758050) — PiP flexi tread,
integral pin-and-socket knuckles. Its changelog fixed overly tight assembly
and easily broken tracks; both are first-class failure modes here (clearance
-> pip_gap/side_gap; breakage -> neck_thickness validation).

Axes:
  X = travel direction, links arrayed along +X at pitch_mm
  Y = hinge axis (width)
  Z = layer/up axis (thickness)

Per-link layout (local origin at this link's TRAILING hinge, x=0):
  fork   (trailing, -X end) : two lugs, one at each Y edge, each bored
                              through along Y at radius pin_r + pip_gap
  tongue + pin (leading, +X end): a central web (tongue_width_mm wide)
                              carrying a full-width pin cylinder centered
                              at x = pitch_mm
  body   : structural slab connecting fork region to tongue region

Engagement (roller-chain style): the sprocket seats directly on the
exposed pin barrel between adjacent link bodies — the pin IS the roller,
same convention as every other PiP part in this library ("the knuckle IS
the roller"). pitch_mm is therefore also the sprocket pitch.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty
from math import pi, cos, sin, atan, degrees

BOOL_EPSILON = 0.001
WALL_T       = 1.0   # guaranteed wall thickness around any bore (mm)


# ── Geometry helpers (add into an existing bmesh) ─────────────────────────────

def _add_box(bm, x0, x1, y0, y1, z0, z1):
    v = [bm.verts.new(p) for p in [
        (x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
        (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1),
    ]]
    bm.verts.index_update()
    bm.faces.new([v[0],v[3],v[2],v[1]])
    bm.faces.new([v[4],v[5],v[6],v[7]])
    bm.faces.new([v[0],v[1],v[5],v[4]])
    bm.faces.new([v[1],v[2],v[6],v[5]])
    bm.faces.new([v[2],v[3],v[7],v[6]])
    bm.faces.new([v[3],v[0],v[4],v[7]])


def _add_cyl_y(bm, cx, cz, radius, y0, y1, n):
    """Add a solid cylinder (axis along Y, XZ-centre at (cx, cz)) to bm."""
    ang = [2 * pi * i / n for i in range(n)]
    vb  = [bm.verts.new((cx + radius * cos(a), y0, cz + radius * sin(a))) for a in ang]
    vt  = [bm.verts.new((cx + radius * cos(a), y1, cz + radius * sin(a))) for a in ang]
    bm.verts.index_update()
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([vb[i], vb[ni], vt[ni], vt[i]])
    cb = bm.verts.new((cx, y0, cz))
    ct = bm.verts.new((cx, y1, cz))
    bm.verts.index_update()
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([cb, vb[ni], vb[i]])
        bm.faces.new([ct, vt[i], vt[ni]])


def _make_cutter_obj(cx, cz, radius, y0, y1, n, name, context):
    """Standalone cylinder object used as a boolean cutter."""
    bm = bmesh.new()
    _add_cyl_y(bm, cx, cz, radius, y0, y1, n)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Me")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    context.collection.objects.link(obj)
    return obj


def _bool_diff(context, body, cutter):
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    context.view_layer.objects.active = body
    mod           = body.modifiers.new("Bool", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object    = cutter
    mod.solver    = 'EXACT'
    with context.temp_override(active_object=body):
        bpy.ops.object.modifier_apply(modifier="Bool")
    bpy.data.objects.remove(cutter, do_unlink=True)


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_pip_track(bpy.types.Operator):
    """Print-in-place track — fork/tongue/pin knuckle links, sprocket-compatible."""
    bl_idname  = "object.pip_track"
    bl_label   = "PiP Track"
    bl_options = {'REGISTER', 'UNDO'}

    pitch_mm:          FloatProperty(name="Pitch (mm)",            default=12.0, min=2.0,  soft_max=60.0,
                                      description="Hinge-to-hinge spacing; must match the mating sprocket pitch")
    width_mm:          FloatProperty(name="Width (mm)",            default=16.0, min=4.0,  soft_max=100.0,
                                      description="Full lateral width along the hinge axis")
    link_thickness_mm: FloatProperty(name="Link Thickness (mm)",   default=4.0,  min=1.0,  soft_max=20.0)
    link_count:        IntProperty(  name="Link Count",            default=8,    min=1,    soft_max=60,
                                      description="Links in the straight strip")

    pin_diameter_mm:   FloatProperty(name="Pin Ø (mm)",            default=3.0,  min=0.5,  soft_max=15.0,
                                      description="Integral pin — also the sprocket's roller diameter")
    pip_gap:           FloatProperty(name="PiP Gap (mm)",          default=0.20, min=0.05, soft_max=1.0,
                                      description="Radial clearance, pin to fork bore")
    side_gap_mm:       FloatProperty(name="Side Gap (mm)",         default=0.30, min=0.05, soft_max=2.0,
                                      description="Axial clearance, tongue to fork inner faces")
    tongue_width_mm:   FloatProperty(name="Tongue Width (mm)",     default=5.0,  min=1.0,  soft_max=50.0,
                                      description="Width of the central web carrying the pin")
    hinge_z_offset_mm: FloatProperty(name="Hinge Z Offset (mm)",   default=0.0,  soft_min=-5.0, soft_max=5.0,
                                      description="Pin-centre offset from mid-thickness")
    neck_thickness_mm: FloatProperty(name="Neck Thickness (mm)",   default=1.6,  min=0.4,  soft_max=8.0,
                                      description="Tongue web depth at its root — breakage-risk parameter")
    end_gap_mm:        FloatProperty(name="End Gap (mm)",          default=0.6,  min=0.05, soft_max=5.0,
                                      description="Clearance between adjacent body end faces")

    outer_segs:            IntProperty(  name="Segments",              default=24,  min=8,   soft_max=64)
    bore_compensation:     FloatProperty(name="Bore Compensation (mm)", default=0.0, min=0.0, soft_max=1.0,
                                          description="FDM holes print tight — added to bore diameter")
    pin_compensation_mm:   FloatProperty(name="Pin Compensation (mm)",  default=0.0, soft_min=-1.0, soft_max=1.0,
                                          description="External features print oversized — added to pin diameter")

    def _derived(self):
        pin_r          = (self.pin_diameter_mm + self.pin_compensation_mm) / 2.0
        bore_r         = pin_r + self.pip_gap + self.bore_compensation / 2.0
        half_w         = self.width_mm / 2.0
        fork_lug_width = (self.width_mm - self.tongue_width_mm - 2.0 * self.side_gap_mm) / 2.0
        fork_reach     = bore_r + WALL_T
        tongue_reach   = self.neck_thickness_mm
        min_pitch      = fork_reach + tongue_reach + 1.0
        hinge_h        = 2.0 * (bore_r + WALL_T)
        min_lug_wall   = WALL_T
        strip_length   = self.link_count * self.pitch_mm
        lever_arm      = hinge_h + self.link_thickness_mm
        max_artic_deg  = degrees(atan(self.end_gap_mm / lever_arm)) if lever_arm > 0 else 0.0

        errors = []
        if self.pip_gap <= 0:
            errors.append("PiP gap must be > 0 — pin would fuse to the bore")
        if fork_lug_width <= 0:
            errors.append("Tongue width + 2x side gap >= track width — no room for fork lugs")
        if self.pitch_mm < min_pitch:
            errors.append("Pitch too small for fork reach + tongue reach (need >= %.2f mm)" % min_pitch)

        warnings = []
        if self.neck_thickness_mm < 1.0:
            warnings.append("Neck thickness < 1.0 mm — easily-broken-track risk")
        if self.pip_gap < 0.15:
            warnings.append("PiP gap < 0.15 mm — likely fused on most FDM printers")
        if self.pip_gap > 0.40:
            warnings.append("PiP gap > 0.40 mm — joint may be sloppy")
        if self.link_count == 1:
            warnings.append("Link count = 1 — no articulating joint is produced")

        return dict(
            pin_r=pin_r, bore_r=bore_r, half_w=half_w,
            fork_lug_width=fork_lug_width, fork_reach=fork_reach,
            tongue_reach=tongue_reach, min_pitch=min_pitch,
            hinge_h=hinge_h, min_lug_wall=min_lug_wall,
            strip_length=strip_length, max_artic_deg=max_artic_deg,
            errors=errors, warnings=warnings,
        )

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=380)

    def draw(self, context):
        layout = self.layout
        d = self._derived()

        col = layout.column(align=True)
        col.prop(self, "pitch_mm")
        col.prop(self, "width_mm")
        col.prop(self, "link_thickness_mm")
        col.prop(self, "link_count")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "pin_diameter_mm")
        col.prop(self, "pip_gap")
        col.prop(self, "side_gap_mm")
        col.prop(self, "tongue_width_mm")
        col.prop(self, "hinge_z_offset_mm")
        col.prop(self, "neck_thickness_mm")
        col.prop(self, "end_gap_mm")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "outer_segs")
        col.prop(self, "bore_compensation")
        col.prop(self, "pin_compensation_mm")

        layout.separator()
        box = layout.box()
        box.label(text="Bore Ø:          %.2f mm" % (d["bore_r"] * 2.0))
        box.label(text="Fork lug width:  %.2f mm" % d["fork_lug_width"])
        box.label(text="Strip length:    %.2f mm" % d["strip_length"])
        box.label(text="Min lug wall:    %.2f mm" % d["min_lug_wall"])
        box.label(text="Max articulation (est.): %.1f °" % d["max_artic_deg"])

        for msg in d["warnings"]:
            layout.label(text=msg, icon='ERROR')
        for msg in d["errors"]:
            layout.label(text=msg, icon='CANCEL')

    def execute(self, context):
        d = self._derived()
        if d["errors"]:
            self.report({'ERROR'}, "; ".join(d["errors"]))
            return {'CANCELLED'}

        pin_r          = d["pin_r"]
        bore_r         = d["bore_r"]
        half_w         = d["half_w"]
        fork_lug_width = d["fork_lug_width"]
        fork_reach     = d["fork_reach"]
        tongue_reach   = d["tongue_reach"]
        hinge_h        = d["hinge_h"]
        pin_cz         = bore_r + WALL_T + self.hinge_z_offset_mm
        n              = self.outer_segs
        p              = self.pitch_mm
        cursor         = context.scene.cursor.location.copy()

        link_objs = []

        for li in range(self.link_count):
            bm = bmesh.new()

            # Body — connects the fork region to the tongue region
            _add_box(bm, fork_reach, p - tongue_reach, -half_w, half_w,
                     0.0, hinge_h + self.link_thickness_mm)

            # Fork lugs (trailing end, x=0) — one at each Y edge
            _add_box(bm, -fork_reach, fork_reach, -half_w, -half_w + fork_lug_width,
                     0.0, hinge_h)
            _add_box(bm, -fork_reach, fork_reach, half_w - fork_lug_width, half_w,
                     0.0, hinge_h)

            # Tongue web (leading end, x=pitch) — central band carrying the pin
            _add_box(bm, p - tongue_reach, p, -self.tongue_width_mm / 2.0, self.tongue_width_mm / 2.0,
                     0.0, hinge_h)

            # Pin — full width, seats into the NEXT link's fork bores
            _add_cyl_y(bm, p, pin_cz, pin_r, -half_w, half_w, n)

            bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
            me = bpy.data.meshes.new("__PTBodyMe")
            bm.to_mesh(me)
            bm.free()
            me.update()

            body = bpy.data.objects.new("__PTBody", me)
            context.collection.objects.link(body)

            # Bore through the fork lugs (single cutter spans the full width;
            # the gap between lugs has no material there anyway)
            fork_cutter = _make_cutter_obj(
                0.0, pin_cz, bore_r,
                -half_w - BOOL_EPSILON, half_w + BOOL_EPSILON,
                n, "__PTForkBore", context,
            )
            _bool_diff(context, body, fork_cutter)

            body.location = (cursor.x + li * p, cursor.y, cursor.z)
            body.name     = "PiPTrack"
            body["bmech_pitch"]        = p
            body["bmech_pin_diameter"] = self.pin_diameter_mm
            body["bmech_width"]        = self.width_mm
            link_objs.append(body)

        bpy.ops.object.select_all(action='DESELECT')
        for ob in link_objs:
            ob.select_set(True)
        context.view_layer.objects.active = link_objs[-1]

        self.report({'INFO'},
            "PiP track: %d link(s), pitch %.1f mm, pin Ø %.1f mm"
            % (self.link_count, p, self.pin_diameter_mm))
        return {'FINISHED'}


# ── Register and run ──────────────────────────────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_pip_track)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_pip_track)
bpy.ops.object.pip_track('INVOKE_DEFAULT')
