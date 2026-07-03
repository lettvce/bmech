"""
Hex Bolt Generator

Additive construction using threaded_fastener.py's own proven "External +
Additive" mode VERBATIM (_thread_params, _external_profile, _build_helix —
documented there as "union with a shaft cylinder to make a bolt"):

  1. Build a core stack: [shank] -> thread-core cylinder -> [tip], each
     segment a fully independent closed solid, stacked touching.
  2. Union the external thread ridge onto the thread-core segment. The core
     radius uses a depth-scaled overlap (not a hairline epsilon) so the
     union has genuine 3D volume in common with the ridge — two nearly-
     coincident CURVED surfaces running the full length of a helix is
     numerically fragile for the EXACT solver otherwise, and can read a
     hairline overlap as a shared boundary and cancel material instead of
     merging it. Normals are re-verified after the union.
  3. Build the hex head as its own separate solid and union it on last — a
     flush, blocky join, much friendlier for the EXACT solver than a thin
     curved union.

Axial layout (Z, head at z=0):
  hex head            z: [0, hex_length_mm]
  shank   (optional)  z: [hex_top, hex_top + shank_length_mm]
  thread core+ridge   z: [shank_top, shank_top + thread_length_mm]
  tip     (optional)  z: [thread_top, thread_top + tip_length_mm]
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, sqrt, ceil
from bpy.props import FloatProperty, IntProperty, BoolProperty

BOOL_EPSILON = 0.001


# ── Thread geometry (duplicated verbatim from threaded_fastener.py) ───────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points outward — bolt ridge on outside of the shaft."""
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
    ]


def _build_helix(bm, profile, pitch, height, res):
    """
    Sweep profile along a helix. Closed manifold:
    - Thread strip faces (flanks + crest) between consecutive rings
    - Root flat faces (minor_r quad) closing the gap between strips
    - Start/end cap quads sealing the open ends
    """
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


# ── Primitive helpers (add into an existing bmesh) ────────────────────────────

def _add_hex_prism(bm, across_flats, z0, z1):
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


_CHAIN_POLE_EPS = 0.01


def _add_chained_revolve(bm, checkpoints, n):
    """
    Continuous, single-walled solid of revolution through an ordered list
    of (radius, z) checkpoints — shank -> thread-core -> tip, as ONE chain.

    Each consecutive pair becomes a cone/cylinder/flat-step side wall,
    SHARING vertices at every junction. This matters specifically because
    independently-capped touching primitives (the previous approach) create
    duplicate coincident faces wherever two segments meet at the SAME
    radius (e.g. a tip starting exactly at the thread-core's own radius) —
    the thread-core's top cap and the tip's bottom cap would be two
    separate faces covering the identical disk, which is a much more
    fragile degenerate case for the EXACT boolean solver than an ordinary
    touching-solids junction. Chaining through shared vertices has no
    internal faces at all, so the problem can't occur.

    A checkpoint with radius <= _CHAIN_POLE_EPS collapses to a single
    shared pole vertex (a true point, e.g. a sharp tip) instead of a
    degenerate zero-radius ring. The real (non-pole) first/last checkpoints
    get capped, since this stack's own ends are true exterior faces (the
    bottom unions onto the hex head; an un-tipped top is just the bolt's
    end face).
    """
    ang = [2.0 * pi * i / n for i in range(n)]

    point_verts = []
    for r, z in checkpoints:
        if r <= _CHAIN_POLE_EPS:
            point_verts.append(bm.verts.new((0.0, 0.0, z)))
        else:
            point_verts.append([bm.verts.new((r * cos(a), r * sin(a), z)) for a in ang])
    bm.verts.index_update()

    for k in range(len(checkpoints) - 1):
        a_r, _ = checkpoints[k]
        b_r, _ = checkpoints[k + 1]
        a_v = point_verts[k]
        b_v = point_verts[k + 1]
        a_pole = a_r <= _CHAIN_POLE_EPS
        b_pole = b_r <= _CHAIN_POLE_EPS

        if a_pole and b_pole:
            continue
        elif a_pole:
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([a_v, b_v[i], b_v[ni]])
        elif b_pole:
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([a_v[i], a_v[ni], b_v])
        else:
            for i in range(n):
                ni = (i + 1) % n
                bm.faces.new([a_v[i], a_v[ni], b_v[ni], b_v[i]])

    first_r, _ = checkpoints[0]
    last_r, _  = checkpoints[-1]
    if first_r > _CHAIN_POLE_EPS:
        bm.faces.new(list(reversed(point_verts[0])))
    if last_r > _CHAIN_POLE_EPS:
        bm.faces.new(point_verts[-1])


def _to_obj(bm, name, context):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    me = bpy.data.meshes.new(name + "Mesh")
    bm.to_mesh(me)
    bm.free()
    me.update()
    obj = bpy.data.objects.new(name, me)
    context.collection.objects.link(obj)
    return obj


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
    # the result doesn't render as if it has holes.
    fix_bm = bmesh.new()
    fix_bm.from_mesh(body.data)
    bmesh.ops.recalc_face_normals(fix_bm, faces=fix_bm.faces[:])
    fix_bm.to_mesh(body.data)
    fix_bm.free()
    body.data.update()


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_hex_bolt(bpy.types.Operator):
    """Hex bolt — head, optional shank, external thread, optional tip."""
    bl_idname  = "object.hex_bolt"
    bl_label   = "Hex Bolt"
    bl_options = {'REGISTER', 'UNDO'}

    hex_length_mm:      FloatProperty(name="Head Length (mm)",     default=5.5,  min=0.5, soft_max=30.0)
    hex_across_flats_mm: FloatProperty(name="Head Across Flats (mm)", default=13.0, min=1.0, soft_max=100.0)

    shank_enable:       BoolProperty( name="Shank",                default=True)
    shank_length_mm:    FloatProperty(name="Shank Length (mm)",    default=10.0, min=0.1, soft_max=200.0)
    shank_diameter_mm:  FloatProperty(name="Shank Ø (mm)",         default=8.0,  min=0.1, soft_max=80.0,
                                      description="Unthreaded shaft diameter — usually equal to thread Ø")

    thread_length_mm:   FloatProperty(name="Thread Length (mm)",   default=20.0, min=0.5, soft_max=200.0)
    thread_diameter_mm: FloatProperty(name="Thread Ø (mm)",        default=8.0,  min=0.5, soft_max=80.0)
    pitch_mm:           FloatProperty(name="Pitch (mm)",           default=1.25, min=0.1, soft_max=10.0)
    flank_angle_deg:    FloatProperty(name="Flank Angle (°)",      default=60.0, min=1.0, max=179.0,
                                      description="60° = metric/UNC, 55° = BSP, 29° = ACME")
    truncation:         FloatProperty(name="Truncation",           default=0.125, min=0.0, max=0.3)
    resolution:         IntProperty(  name="Resolution",           default=32,   min=8,   soft_max=128)
    outer_compensation_mm: FloatProperty(name="Compensation (mm)", default=0.0,  min=0.0, soft_max=0.5,
                                      description="FDM: printed external features tend to shrink — "
                                                  "added to thread major radius")

    tip_enable:         BoolProperty( name="Tip",                  default=True)
    tip_length_mm:      FloatProperty(name="Tip Length (mm)",      default=3.0,  min=0.1, soft_max=30.0)
    tip_diameter_mm:    FloatProperty(name="Tip Ø (mm)",           default=0.0,  min=0.0, soft_max=80.0,
                                      description="0 = sharp point, >0 = flat dog-point tip")

    def _derived(self):
        major_r = self.thread_diameter_mm / 2.0 + self.outer_compensation_mm
        minor_r, cf, fdz, depth = _thread_params(
            major_r, self.pitch_mm, self.flank_angle_deg, self.truncation)
        head_wall = self.hex_across_flats_mm / 2.0 - self.thread_diameter_mm / 2.0
        total_length = (self.hex_length_mm
                        + (self.shank_length_mm if self.shank_enable else 0.0)
                        + self.thread_length_mm
                        + (self.tip_length_mm if self.tip_enable else 0.0))
        return major_r, minor_r, cf, fdz, depth, head_wall, total_length

    def draw(self, context):
        layout = self.layout
        major_r, minor_r, cf, fdz, depth, head_wall, total_length = self._derived()

        col = layout.column(align=True)
        col.prop(self, "hex_length_mm")
        col.prop(self, "hex_across_flats_mm")

        layout.separator()
        layout.prop(self, "shank_enable")
        if self.shank_enable:
            col = layout.column(align=True)
            col.prop(self, "shank_length_mm")
            col.prop(self, "shank_diameter_mm")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "thread_length_mm")
        col.prop(self, "thread_diameter_mm")
        col.prop(self, "pitch_mm")
        col.prop(self, "flank_angle_deg")
        col.prop(self, "truncation")
        col.prop(self, "resolution")
        col.prop(self, "outer_compensation_mm")

        layout.separator()
        layout.prop(self, "tip_enable")
        if self.tip_enable:
            col = layout.column(align=True)
            col.prop(self, "tip_length_mm")
            col.prop(self, "tip_diameter_mm")

        layout.separator()
        box = layout.box()
        box.label(text="Thread depth:  %.3f mm" % depth)
        box.label(text="Minor Ø:       %.3f mm" % (minor_r * 2.0))
        box.label(text="Head wall:     %.2f mm" % head_wall)
        box.label(text="Total length:  %.2f mm" % total_length)

        if fdz <= 0:
            layout.label(text="Truncation too high — no room for flanks at this pitch", icon='ERROR')
        if head_wall <= 0.5:
            layout.label(text="Head too small for thread diameter", icon='ERROR')

    def execute(self, context):
        major_r, minor_r, cf, fdz, depth, head_wall, total_length = self._derived()

        if fdz <= 0 or head_wall <= 0:
            self.report({'ERROR'}, "Invalid geometry — check truncation and head size vs thread diameter")
            return {'CANCELLED'}

        n      = self.resolution
        cursor = context.scene.cursor.location.copy()

        # ── Core stack: [shank] -> thread-core -> [tip] (head added later) ───
        # Built as ONE chained revolve (see _add_chained_revolve) rather than
        # independently-capped stacked primitives, so the thread-core-to-tip
        # junction (which sits at the exact same radius, core_r) doesn't end
        # up with two duplicate coincident cap faces there.
        z = self.hex_length_mm  # everything sits above where the head will be unioned on

        # A hairline overlap is fine for a flat cut extended past a flat/simple
        # surface, but two nearly-coincident CURVED surfaces running the full
        # length of a helix is numerically fragile for the EXACT solver — it
        # can read the near-coincidence as a shared boundary and cancel
        # material instead of merging it. Use a real, depth-scaled overlap.
        overlap = max(0.02, min(0.2 * depth, 0.15))
        core_r  = minor_r + overlap

        def _add_cp(cp_list, r, cz):
            if cp_list and abs(cp_list[-1][0] - r) < 1e-9 and abs(cp_list[-1][1] - cz) < 1e-9:
                return  # identical to the previous checkpoint — skip the zero-length segment
            cp_list.append((r, cz))

        checkpoints = []
        if self.shank_enable:
            shank_r = self.shank_diameter_mm / 2.0
            _add_cp(checkpoints, shank_r, z)
            z += self.shank_length_mm
            _add_cp(checkpoints, shank_r, z)

        thread_z0 = z
        _add_cp(checkpoints, core_r, z)
        z += self.thread_length_mm
        _add_cp(checkpoints, core_r, z)

        if self.tip_enable:
            z += self.tip_length_mm
            _add_cp(checkpoints, self.tip_diameter_mm / 2.0, z)

        bm = bmesh.new()
        _add_chained_revolve(bm, checkpoints, n)
        core = _to_obj(bm, "__HexBoltCore", context)
        core.location = cursor

        # ── External thread ridges, unioned onto the thread-core segment ────
        thread_bm = bmesh.new()
        prof = _external_profile(major_r, minor_r, cf, fdz)
        _build_helix(thread_bm, prof, self.pitch_mm, self.thread_length_mm, n)
        thread_obj = _to_obj(thread_bm, "__HexBoltThread", context)
        thread_obj.location = (cursor.x, cursor.y, cursor.z + thread_z0)

        _bool_union(context, core, thread_obj)

        # ── Head, built separately and unioned on last ───────────────────────
        head_bm = bmesh.new()
        _add_hex_prism(head_bm, self.hex_across_flats_mm, 0.0, self.hex_length_mm)
        head = _to_obj(head_bm, "__HexBoltHead", context)
        head.location = cursor

        _bool_union(context, core, head)
        core.name = "HexBolt"

        core["bmech_thread_diameter"] = self.thread_diameter_mm
        core["bmech_pitch"]           = self.pitch_mm
        core["bmech_flank_angle_deg"] = self.flank_angle_deg
        core["bmech_truncation"]      = self.truncation

        bpy.ops.object.select_all(action='DESELECT')
        core.select_set(True)
        context.view_layer.objects.active = core

        self.report({'INFO'},
            "Hex bolt: M%.1f thread, %.1f mm total length"
            % (self.thread_diameter_mm, total_length))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_hex_bolt,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
