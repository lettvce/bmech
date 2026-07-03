"""
Compliant Alligator Clip Prototype
────────────────────────────────────
Alt+P to run. Popup dialog for parameter editing.

Single-piece, printed flat. No assembly required.

Mechanism
─────────
  Two arms (upper and lower) connected at a thin rectangular flexure bridge
  in the center. The flexure is the hinge AND the spring — it stores elastic
  energy when bent and restores the jaws to the closed position.

  Lever action (class-1 lever, fulcrum = flexure):
    Squeeze handles → flexure bends → jaws OPEN
    Release handles → flexure springs back → jaws CLOSE

  The rest state has jaws nearly closed (jaw_gap) and handles spread apart
  (handle_gap > jaw_gap). The pre-stress is built into the geometry.

Print orientation
─────────────────
  Print FLAT on the bed (XY plane). The flexure spans across the waist of
  the clip and must be printed with its thickness in the XY plane (not in Z).
  This gives the flexure the correct spring properties — if printed upright,
  the flexure bends through layer boundaries and fails quickly.

Flexure thickness guide (for 0.4mm nozzle)
────────────────────────────────────────────
  < 0.8mm  — very soft, may be too weak for repeated use
  0.8–1.2mm — good range for PETG/TPU (start here)
  1.0–1.5mm — stiffer, more durable for PLA
  > 2.0mm  — strong but hard to squeeze

2D profile outline (CCW, Z=0 face)
─────────────────────────────────────
  Upper jaw outer tip → handle outer tip     (straight top edge)
  Handle right face (down to inner tip)
  Handle inner → flexure (upper arm inner edge, with notch at flexure)
  Upper jaw face (with optional teeth, going left)
  JAW GAP (vertical, open left side of clip)
  Lower jaw face (with optional teeth, going right)
  Lower flexure → handle inner
  Handle right face (down to outer tip)
  Lower outer edge back to lower jaw outer tip  (straight bottom edge)
  Jaw back face (close, outer back wall of jaws)
"""

import bpy
import bmesh
from bpy.props import IntProperty, FloatProperty
from math import pi


# ── Profile builder ───────────────────────────────────────────────────────────

def _jaw_teeth_upper(x_start, x_end, y_base, n_teeth, tooth_h):
    """
    Serration points on the upper jaw inner face, going LEFT (x decreasing).
    x_start > x_end (x_start is closer to flexure, x_end is jaw tip).
    Returns list of (x, y) points. First point is NOT x_start (already in list).
    Last point lands at (x_end, y_base).
    """
    pts = []
    face_len = x_start - x_end
    if n_teeth > 0 and tooth_h > 0 and face_len > 1e-6:
        pitch = face_len / n_teeth
        for i in range(n_teeth):
            x_tip  = x_start - (i + 0.5) * pitch
            x_base = x_start - (i + 1.0) * pitch
            pts.append((x_tip,  y_base - tooth_h))  # tip toward center
            pts.append((x_base, y_base))
    else:
        pts.append((x_end, y_base))
    return pts


def _jaw_teeth_lower(x_start, x_end, y_base, n_teeth, tooth_h):
    """
    Serration points on the lower jaw inner face, going RIGHT (x increasing).
    x_start < x_end.
    Returns list of (x, y). First point is NOT x_start. Last point at (x_end, y_base).
    """
    pts = []
    face_len = x_end - x_start
    if n_teeth > 0 and tooth_h > 0 and face_len > 1e-6:
        pitch = face_len / n_teeth
        for i in range(n_teeth):
            x_tip  = x_start + (i + 0.5) * pitch
            x_base = x_start + (i + 1.0) * pitch
            pts.append((x_tip,  y_base + tooth_h))   # tip toward center
            pts.append((x_base, y_base))
    else:
        pts.append((x_end, y_base))
    return pts


def clip_profile(jaw_l, jaw_gap, handle_l, handle_gap, body_w,
                 flex_t, flex_w, taper_l, n_teeth, tooth_h):
    """
    2D CCW polygon of the alligator clip (viewed from +Z).

    Coordinate system:
      Origin: flexure center
      -X: jaw direction (jaws open left)
      +X: handle direction

    Key Y values:
      ±(jaw_gap/2 + body_w)     outer boundary (jaw side)
      ±(handle_gap/2 + body_w)  outer boundary (handle side)
      ±jaw_gap/2                upper/lower jaw inner face
      ±handle_gap/2             upper/lower handle inner face
      ±flex_t/2                 flexure (minimum gap)
    """
    hj  = jaw_gap    / 2.0
    hh  = handle_gap / 2.0
    hft = flex_t     / 2.0
    hfw = flex_w     / 2.0

    x_jaw    = -jaw_l
    x_jf     = -(hfw + taper_l)   # jaw face end (taper starts here)
    x_flj    = -hfw               # flexure jaw edge
    x_flh    = +hfw               # flexure handle edge
    x_hf     = +(hfw + taper_l)   # handle face start (taper ends here)
    x_hdl    = +handle_l

    # Outer Y on each side (outer boundary is body_w above inner face)
    oj = hj + body_w    # outer Y at jaw side
    oh = hh + body_w    # outer Y at handle side

    pts = []

    # ── Upper outer edge (jaw tip → handle tip, going RIGHT) ─────────────────
    pts.append((x_jaw, +oj))
    pts.append((x_hdl, +oh))

    # ── Handle right cap (going DOWN) ────────────────────────────────────────
    pts.append((x_hdl, +hh))

    # ── Upper arm inner edge (going LEFT) ────────────────────────────────────
    pts.append((x_hf,  +hh))            # handle constant zone
    pts.append((x_flh, +hft))           # handle-flexure taper end
    pts.append((x_flj, +hft))           # flexure (constant)
    pts.append((x_jf,  +hj))            # flexure-jaw taper end

    # Upper jaw face (teeth or straight, going LEFT to jaw tip)
    pts.extend(_jaw_teeth_upper(x_jf, x_jaw, +hj, n_teeth, tooth_h))

    # ── Jaw gap (going DOWN — open left side) ────────────────────────────────
    pts.append((x_jaw, -hj))

    # ── Lower arm inner edge (going RIGHT) ───────────────────────────────────
    pts.extend(_jaw_teeth_lower(x_jaw, x_jf, -hj, n_teeth, tooth_h))

    pts.append((x_flj, -hft))           # flexure-jaw taper end
    pts.append((x_flh, -hft))           # flexure (constant)
    pts.append((x_hf,  -hh))            # flexure-handle taper end
    pts.append((x_hdl, -hh))            # handle constant zone

    # ── Handle right cap (going DOWN) ────────────────────────────────────────
    pts.append((x_hdl, -oh))

    # ── Lower outer edge (handle tip → jaw tip, going LEFT) ──────────────────
    pts.append((x_jaw, -oj))

    # ── Jaw back face closes polygon (going UP) ───────────────────────────────
    # (from -oj to +oj at x_jaw — the outer back wall of the jaws)

    return pts


# ── Mesh builder ──────────────────────────────────────────────────────────────

def build_clip(context, pts, depth, name, location):
    n  = len(pts)
    bm = bmesh.new()

    vb = [bm.verts.new((x, y, 0.0))   for x, y in pts]
    vt = [bm.verts.new((x, y, depth)) for x, y in pts]
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

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()

    obj          = bpy.data.objects.new(name, me)
    obj.location = location
    context.collection.objects.link(obj)
    return obj


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_alligator_clip(bpy.types.Operator):
    """Compliant alligator clip — single-piece, printed flat."""
    bl_idname  = "object.alligator_clip"
    bl_label   = "Alligator Clip"
    bl_options = {'REGISTER', 'UNDO'}

    jaw_length:    FloatProperty(name="Jaw Length (mm)",        default=25.0, min=5.0,  soft_max=80.0,
                                  description="Length of the gripping jaw section")
    jaw_gap:       FloatProperty(name="Jaw Gap (mm)",           default=2.0,  min=0.6,  soft_max=20.0,
                                  description="Jaw tip opening in rest state (should be small — clip springs closed)")
    handle_length: FloatProperty(name="Handle Length (mm)",     default=35.0, min=5.0,  soft_max=100.0,
                                  description="Length of the squeeze handle section")
    handle_gap:    FloatProperty(name="Handle Gap (mm)",        default=10.0, min=1.0,  soft_max=40.0,
                                  description="Handle opening in rest state. Must be > jaw_gap for lever action.")
    body_width:    FloatProperty(name="Arm Width (mm)",         default=5.0,  min=1.5,  soft_max=20.0,
                                  description="Thickness of each arm (jaw and handle) measured from its inner face")
    flexure_t:     FloatProperty(name="Flexure Thickness (mm)", default=1.0,  min=0.3,  soft_max=4.0,
                                  description="Thin bridge thickness at hinge — lower = softer spring. PETG: 0.8–1.2mm, PLA: 1.0–1.5mm")
    flexure_w:     FloatProperty(name="Flexure Width (mm)",     default=3.0,  min=1.0,  soft_max=15.0,
                                  description="Length of the constant-thickness hinge zone")
    taper_l:       FloatProperty(name="Taper Length (mm)",      default=6.0,  min=2.0,  soft_max=25.0,
                                  description="Gradual transition between arm thickness and flexure thickness. Longer = lower stress concentration.")
    teeth_count:   IntProperty(  name="Teeth",                  default=6,    min=0,    max=40,
                                  description="Serration count on each jaw face. 0 = smooth")
    tooth_height:  FloatProperty(name="Tooth Height (mm)",      default=0.5,  min=0.1,  soft_max=2.0,
                                  description="Height of each triangular serration")
    clip_depth:    FloatProperty(name="Clip Depth (mm)",        default=5.0,  min=1.0,  soft_max=20.0,
                                  description="Print thickness (clip lies flat — this is the Z height)")

    def _derived(self):
        jaw_face_l   = self.jaw_length - (self.flexure_w / 2.0 + self.taper_l)
        hdl_face_l   = self.handle_length - (self.flexure_w / 2.0 + self.taper_l)
        lever_ratio  = self.jaw_length / self.handle_length
        total_length = self.jaw_length + self.handle_length
        total_width  = self.handle_gap + 2.0 * self.body_width

        # Approximate jaw opening per mm of handle closure
        mm_open_per_mm_squeeze = lever_ratio

        ok_lever   = self.handle_gap > self.jaw_gap
        ok_flex    = self.flexure_t < self.jaw_gap and self.flexure_t < self.handle_gap
        ok_jaw_r   = jaw_face_l > 2.0
        ok_hdl_r   = hdl_face_l > 2.0
        ok_teeth   = self.tooth_height * 2 < self.jaw_gap

        return (jaw_face_l, hdl_face_l, lever_ratio, total_length, total_width,
                mm_open_per_mm_squeeze, ok_lever, ok_flex, ok_jaw_r, ok_hdl_r, ok_teeth)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=330)

    def draw(self, context):
        layout = self.layout
        (jaw_face_l, hdl_face_l, lever_ratio, total_length, total_width,
         mm_open, ok_lever, ok_flex, ok_jaw_r, ok_hdl_r, ok_teeth) = self._derived()

        col = layout.column(align=True)
        col.label(text="Jaw")
        col.prop(self, "jaw_length")
        col.prop(self, "jaw_gap")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Handle")
        col.prop(self, "handle_length")
        col.prop(self, "handle_gap")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Arms & Flexure")
        col.prop(self, "body_width")
        col.prop(self, "flexure_t")
        col.prop(self, "flexure_w")
        col.prop(self, "taper_l")

        layout.separator()
        col = layout.column(align=True)
        col.label(text="Teeth")
        col.prop(self, "teeth_count")
        col.prop(self, "tooth_height")

        layout.separator()
        col.prop(self, "clip_depth")

        layout.separator()
        box = layout.box()
        box.label(text="Total length:    %.1f mm" % total_length)
        box.label(text="Total width:     %.1f mm" % total_width)
        box.label(text="Lever ratio:     1 : %.2f  (jaw : handle)" % lever_ratio)
        box.label(text="Jaw opens ~%.2f mm per mm handle squeeze" % mm_open)
        box.label(text="Jaw face length: %.1f mm" % jaw_face_l)

        if not ok_lever:
            layout.label(text="handle_gap must be > jaw_gap for lever action", icon='ERROR')
        if not ok_flex:
            layout.label(text="flexure_t must be < both gap values", icon='ERROR')
        if not ok_jaw_r:
            layout.label(text="Jaw face too short — increase jaw_length or reduce flexure_w/taper", icon='ERROR')
        if not ok_hdl_r:
            layout.label(text="Handle face too short — increase handle_length or reduce flexure_w/taper", icon='ERROR')
        if not ok_teeth and self.teeth_count > 0:
            layout.label(text="Teeth too tall — tooth tips cross center", icon='ERROR')

    def execute(self, context):
        (jaw_face_l, hdl_face_l, lever_ratio, total_length, total_width,
         mm_open, ok_lever, ok_flex, ok_jaw_r, ok_hdl_r, ok_teeth) = self._derived()

        if not ok_lever or not ok_flex or not ok_jaw_r or not ok_hdl_r:
            return {'CANCELLED'}
        if self.teeth_count > 0 and not ok_teeth:
            return {'CANCELLED'}

        pts = clip_profile(
            self.jaw_length, self.jaw_gap,
            self.handle_length, self.handle_gap,
            self.body_width,
            self.flexure_t, self.flexure_w, self.taper_l,
            self.teeth_count, self.tooth_height,
        )

        cursor = context.scene.cursor.location.copy()
        obj = build_clip(context, pts, self.clip_depth, "AlligatorClip", cursor)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'},
            "Alligator clip: %.0f mm total, 1:%.2f lever, flexure %.1f mm"
            % (total_length, lever_ratio, self.flexure_t))
        return {'FINISHED'}


# ── Deleted when graduating to mechanisms_core ────────────────────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_alligator_clip)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_alligator_clip)
bpy.ops.object.alligator_clip('INVOKE_DEFAULT')
