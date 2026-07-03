"""
Hex Thru-Nut Generator

Two-step construction, matching threaded_fastener.py's own documented
"Internal + Additive" workflow ("union with a tube bore to build nut thread
ridges") instead of inverting it:
  1. Cut a plain round bore through the hex prism, sized to the thread's
     root diameter (major_r) — large enough to leave real empty space for
     the ridges to protrude into.
  2. Union the internal-thread ridge (_internal_profile + _build_helix) onto
     that bore — the ridge is material meant to be ADDED, not a void shape
     meant to be subtracted.
  3. Intersect against a clean Z-bound to guarantee a flush top/bottom,
     since _build_helix's own step count rounds up and can overshoot the
     requested height slightly on its own.

Thread math is duplicated from threaded_fastener.py per this project's
convention (each generator module is self-contained, no cross-file thread
math imports).
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil, sqrt
from bpy.props import FloatProperty, IntProperty

BOOL_EPSILON = 0.001


# ── Thread geometry (duplicated from threaded_fastener.py) ────────────────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward — cutter shape for tapping female threads."""
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
    ]


def _build_helix(bm, profile, pitch, height, res):
    n            = len(profile)
    profile_span = max(dz for _, dz in profile)
    steps        = int(ceil((height - profile_span) * res / pitch)) + 1
    rings = []
    for i in range(steps):
        ang  = 2.0 * pi * i / res
        zb   = pitch * i / res
        ring = [bm.verts.new((r * cos(ang), r * sin(ang), zb + dz))
                for r, dz in profile]
        rings.append(ring)

    for i in range(len(rings) - 1):
        for k in range(n - 1):
            bm.faces.new([rings[i][k], rings[i][k + 1],
                          rings[i + 1][k + 1], rings[i + 1][k]])
        bm.faces.new([rings[i][0], rings[i][n - 1],
                      rings[i + 1][n - 1], rings[i + 1][0]])

    bm.faces.new(list(reversed(rings[0])))
    bm.faces.new(rings[-1])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])


# ── Hex prism ──────────────────────────────────────────────────────────────────

def _add_hex_prism(bm, across_flats, z0, z1):
    """Regular hexagon (flat-to-flat = across_flats), extruded z0 -> z1."""
    r     = across_flats / sqrt(3.0)
    verts = [(r * cos(radians(30.0 + 60.0 * i)), r * sin(radians(30.0 + 60.0 * i)))
              for i in range(6)]
    bot = [bm.verts.new((x, y, z0)) for x, y in verts]
    top = [bm.verts.new((x, y, z1)) for x, y in verts]
    bm.verts.index_update()
    bm.faces.new(list(reversed(bot)))
    bm.faces.new(top)
    for i in range(6):
        ni = (i + 1) % 6
        bm.faces.new([bot[i], bot[ni], top[ni], top[i]])


def _add_cyl_z(bm, radius, z0, z1, n):
    """Solid, fully-capped cylinder, axis along Z, centred on the origin in XY."""
    ang = [2.0 * pi * i / n for i in range(n)]
    bot = [bm.verts.new((radius * cos(a), radius * sin(a), z0)) for a in ang]
    top = [bm.verts.new((radius * cos(a), radius * sin(a), z1)) for a in ang]
    bm.verts.index_update()
    bm.faces.new(list(reversed(bot)))
    bm.faces.new(top)
    for i in range(n):
        ni = (i + 1) % n
        bm.faces.new([bot[i], bot[ni], top[ni], top[i]])


def _to_obj(bm, name, context):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Mesh")
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


def _bool_intersect(context, body, bound):
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    context.view_layer.objects.active = body
    mod           = body.modifiers.new("Bool", 'BOOLEAN')
    mod.operation = 'INTERSECT'
    mod.object    = bound
    mod.solver    = 'EXACT'
    with context.temp_override(active_object=body):
        bpy.ops.object.modifier_apply(modifier="Bool")
    bpy.data.objects.remove(bound, do_unlink=True)


def _bool_union(context, body, addend):
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    context.view_layer.objects.active = body
    mod           = body.modifiers.new("Bool", 'BOOLEAN')
    mod.operation = 'UNION'
    mod.object    = addend
    mod.solver    = 'EXACT'
    with context.temp_override(active_object=body):
        bpy.ops.object.modifier_apply(modifier="Bool")
    bpy.data.objects.remove(addend, do_unlink=True)

    # EXACT-solver unions of complex meshes can come back with inconsistent
    # face winding even when topologically solid — re-normalize afterward so
    # the result doesn't confuse the DIFFERENCE step that follows.
    fix_bm = bmesh.new()
    fix_bm.from_mesh(body.data)
    bmesh.ops.recalc_face_normals(fix_bm, faces=fix_bm.faces[:])
    fix_bm.to_mesh(body.data)
    fix_bm.free()
    body.data.update()


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_hex_nut(bpy.types.Operator):
    """Hex thru-nut — internal thread cut all the way through a hex prism."""
    bl_idname  = "object.hex_nut"
    bl_label   = "Hex Nut"
    bl_options = {'REGISTER', 'UNDO'}

    z_height_mm:      FloatProperty(name="Height (mm)",         default=6.5,  min=0.5, soft_max=50.0)
    across_flats_mm:  FloatProperty(name="Across Flats (mm)",   default=13.0, min=1.0, soft_max=100.0)
    thread_diameter_mm: FloatProperty(name="Thread Ø (mm)",     default=8.0,  min=0.5, soft_max=80.0,
                                      description="Nominal major diameter of the internal thread — the 'hole size'")
    pitch_mm:         FloatProperty(name="Pitch (mm)",          default=1.25, min=0.1, soft_max=10.0)
    flank_angle_deg:  FloatProperty(name="Flank Angle (°)",     default=60.0, min=1.0, max=179.0,
                                      description="60° = metric/UNC, 55° = BSP, 29° = ACME")
    truncation:       FloatProperty(name="Truncation",          default=0.125, min=0.0, max=0.3)
    resolution:       IntProperty(  name="Resolution",          default=32,   min=8,   soft_max=128)
    inner_compensation_mm: FloatProperty(name="Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
                                      description="FDM: printed holes come out tight — added to thread major radius")

    def _derived(self):
        # Standard thread nomenclature: the nominal/basic size IS the major
        # diameter, for both external (bolt) and internal (nut) threads —
        # e.g. M8 means an 8.000 mm major diameter on both parts. The minor
        # diameter (here: the internal ridge's tip, its innermost/smallest
        # reach) is DERIVED from major_r via pitch + flank angle, not the
        # other way around.
        major_r = self.thread_diameter_mm / 2.0 + self.inner_compensation_mm
        minor_r, cf, fdz, depth = _thread_params(
            major_r, self.pitch_mm, self.flank_angle_deg, self.truncation)
        wall = self.across_flats_mm / 2.0 - major_r
        return major_r, minor_r, cf, fdz, depth, wall

    def draw(self, context):
        layout = self.layout
        major_r, minor_r, cf, fdz, depth, wall = self._derived()

        col = layout.column(align=True)
        col.prop(self, "z_height_mm")
        col.prop(self, "across_flats_mm")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "thread_diameter_mm")
        col.prop(self, "pitch_mm")
        col.prop(self, "flank_angle_deg")
        col.prop(self, "truncation")
        col.prop(self, "resolution")
        col.prop(self, "inner_compensation_mm")

        layout.separator()
        box = layout.box()
        box.label(text="Thread depth:  %.3f mm" % depth)
        box.label(text="Minor Ø:       %.3f mm" % (minor_r * 2.0))
        box.label(text="Across corners: %.2f mm" % (self.across_flats_mm * 2.0 / sqrt(3.0)))
        box.label(text="Min wall (flats): %.2f mm" % wall)

        if fdz <= 0:
            layout.label(text="Truncation too high — no room for flanks at this pitch", icon='ERROR')
        if wall <= 0.5:
            layout.label(text="Thin or negative wall — increase across flats or reduce thread Ø", icon='ERROR')

    def execute(self, context):
        major_r, minor_r, cf, fdz, depth, wall = self._derived()

        if fdz <= 0 or wall <= 0:
            self.report({'ERROR'}, "Invalid geometry — check truncation and across-flats vs thread diameter")
            return {'CANCELLED'}

        cursor = context.scene.cursor.location.copy()

        bm = bmesh.new()
        _add_hex_prism(bm, self.across_flats_mm, 0.0, self.z_height_mm)
        body = _to_obj(bm, "HexNut", context)
        body.location = cursor

        # _internal_profile + _build_helix is threaded_fastener.py's own
        # "Internal + Additive" mode — its docstring says so directly: union
        # with a tube bore to build nut thread ridges. That means the bore
        # needs to be cut LARGE (out to the thread's root diameter,
        # major_r) so there's actual empty space for the ridges to
        # protrude into — cutting it to the ridge-tip/minor diameter leaves
        # the body already solid everywhere the ridge occupies, so unioning
        # the ridge in would be a no-op: hole, no threads.
        overlap = max(0.02, min(0.2 * depth, 0.15))
        bore_bm = bmesh.new()
        _add_cyl_z(bore_bm, major_r - overlap,
                   -BOOL_EPSILON, self.z_height_mm + BOOL_EPSILON, self.resolution)
        bore_cutter = _to_obj(bore_bm, "__HexNutBore", context)
        bore_cutter.location = cursor
        _bool_diff(context, body, bore_cutter)

        # No +-BOOL_EPSILON axial padding here — that convention is for
        # SUBTRACTIVE cutters (over-extending past a boundary is harmless
        # when removing material). This ridge gets UNIONed (added), so any
        # extension past the nut's real z=0..z_height range would show up
        # as a visible nub poking out the top/bottom faces.
        ridge_bm = bmesh.new()
        prof = _internal_profile(major_r, minor_r, cf, fdz)
        _build_helix(ridge_bm, prof, self.pitch_mm, self.z_height_mm, self.resolution)
        ridge = _to_obj(ridge_bm, "__HexNutRidge", context)
        ridge.location = cursor
        _bool_union(context, body, ridge)

        # _build_helix's own step count rounds UP to guarantee full coverage,
        # so it can still slightly overshoot z_height regardless of padding.
        # Clip the result back to the nut's true bounds to guarantee a flush
        # top/bottom rather than relying on exact step-count arithmetic.
        clip_bm = bmesh.new()
        clip_r  = self.across_flats_mm  # comfortably larger than the hex's own circumradius
        _add_cyl_z(clip_bm, clip_r, 0.0, self.z_height_mm, self.resolution)
        clip_bound = _to_obj(clip_bm, "__HexNutClip", context)
        clip_bound.location = cursor
        _bool_intersect(context, body, clip_bound)

        body["bmech_thread_diameter"] = self.thread_diameter_mm
        body["bmech_pitch"]           = self.pitch_mm
        body["bmech_flank_angle_deg"] = self.flank_angle_deg
        body["bmech_truncation"]      = self.truncation

        bpy.ops.object.select_all(action='DESELECT')
        body.select_set(True)
        context.view_layer.objects.active = body

        self.report({'INFO'},
            "Hex nut: M%.1f thread, %.1f mm across flats, %.1f mm tall"
            % (self.thread_diameter_mm, self.across_flats_mm, self.z_height_mm))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_hex_nut,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
