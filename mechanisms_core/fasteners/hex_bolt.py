"""
Hex Bolt Generator

Subtractive construction: cut the external thread as a helical groove out
of a solid blank, rather than union a separate ridge onto an undersized
core (threaded_fastener.py's "External + Additive" mode, this file's own
approach before this rewrite).

  1. Build a blank stack: [shank] -> thread blank cylinder (at the FULL
     major diameter) -> [tip], as one continuous chained revolve.
  2. Cut the thread by DIFFERENCE-ing a helical groove cutter out of the
     thread-blank segment. A plain difference against a solid blank is a
     numerically easy case for the EXACT solver — the cutter naturally
     overlaps solid material throughout — unlike the old additive path,
     which needed a depth-scaled core-radius fudge just so the union of two
     nearly-coincident curved surfaces had genuine volume to merge instead
     of reading as a hairline touch and cancelling material.
  3. Build the hex head as its own separate solid and union it on last — a
     flush, blocky join, much friendlier for the EXACT solver than a thin
     curved union.

Cutter profile note: the cutter is built with _internal_profile (the same
shape threaded_fastener.py's own "External + Subtractive" mode uses), but
with the crest-flat and root-flat lengths SWAPPED from what that profile's
own argument names suggest (root_flat, not crest_flat, goes in the
"crest_flat" slot). Passing crest_flat straight through, as threaded_fastener.py
does, produces a geometrically valid but backwards thread: a wide flat at
the crest and a narrow one at the root, rather than the standard (and this
project's own additive-ridge) convention of a narrow crest flat and a wide
root flat. Verified empirically in a headless Blender test before relying
on it — see the conversation history, not worth re-deriving from the
profile math alone.

Tip: a flat perpendicular step down to tip_r at the thread's own end Z,
then a STRAIGHT (non-tapered) pin out to tip_length — not a cone. A cone
here would shrink the blank's diameter while the thread cutter above is
still cutting a constant-diameter helix through it, so the thread runs out
against a shrinking target and dead-ends right at the tip instead of just
stopping cleanly. The flat step keeps the entire thread_length span at a
uniform major_r for the cutter, and the pin — meant to sit under the
thread's minor Ø, see tip_diameter_mm's own doc — becomes a plain
unthreaded lead-in a nut can start on before the crests engage.

Axial layout (Z, head at z=0):
  hex head              z: [0, hex_length_mm]
  shank     (optional)  z: [hex_top, hex_top + shank_length_mm]
  thread blank + cut    z: [shank_top, shank_top + thread_length_mm]
  tip       (optional)  z: [thread_top, thread_top + tip_length_mm]  (flat step, straight pin)
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, sqrt, ceil
from bpy.props import FloatProperty, IntProperty, BoolProperty
from . import fastener_matching

BOOL_EPSILON = 0.001


# ── Thread geometry (duplicated verbatim from threaded_fastener.py) ───────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """
    Crest points inward. Used here as the SUBTRACTIVE cutter for an
    external thread — see the module docstring for why crest_flat must be
    passed root_flat (2 x truncation x pitch), not truncation x pitch, to
    get the crest/root proportions the right way round.
    """
    return [
        (major_r, 0.0),
        (minor_r, flank_dz),
        (minor_r, flank_dz + crest_flat),
        (major_r, flank_dz * 2.0 + crest_flat),
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

class OBJECT_OT_hex_bolt(bpy.types.Operator):
    """Hex bolt — head, optional shank, external thread, optional tip."""
    bl_idname  = "object.hex_bolt"
    bl_label   = "Hex Bolt"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        fastener_matching.sync_thread_dims(self, context.window_manager.bmech_fastener_target)

    def invoke(self, context, event):
        fastener_matching.reset_target(context)
        return self.execute(context)

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
    fit_offset_mm:      FloatProperty(name="Fit Offset (mm)",      default=0.0,  min=0.0, soft_max=0.5,
                                      description="FDM: subtracted from thread diameter for a looser, "
                                                  "better-fitting mesh against a mating internal thread "
                                                  "whose own diameter is increased by the same offset. "
                                                  "Not synced by Match Target")

    tip_enable:         BoolProperty( name="Tip",                  default=True)
    tip_length_mm:      FloatProperty(name="Tip Length (mm)",      default=3.0,  min=0.1, soft_max=30.0)
    tip_diameter_mm:    FloatProperty(name="Tip Ø (mm)",           default=0.0,  min=0.0, soft_max=80.0,
                                      description="0 = sharp point, >0 = flat dog-point tip. Keep under the "
                                                  "thread's minor Ø so it works as an unthreaded lead-in pin "
                                                  "a nut can start on")

    def _derived(self):
        major_r = self.thread_diameter_mm / 2.0 + self.outer_compensation_mm - self.fit_offset_mm / 2.0
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

        layout.prop(context.window_manager, "bmech_fastener_target", text="Match Target")
        has_target = context.window_manager.bmech_fastener_target is not None

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
        # thread_diameter_mm/pitch_mm/flank_angle_deg/truncation all need to
        # match a mating nut exactly for the threads to physically engage —
        # unlike the gear family, there's no partial-match case here, so all
        # four freeze together whenever any valid target is set.
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "thread_diameter_mm")
        driven.prop(self, "pitch_mm")
        driven.prop(self, "flank_angle_deg")
        driven.prop(self, "truncation")
        col.prop(self, "resolution")
        col.prop(self, "outer_compensation_mm")
        col.prop(self, "fit_offset_mm")

        layout.separator()
        layout.prop(self, "tip_enable")
        if self.tip_enable:
            col = layout.column(align=True)
            col.prop(self, "tip_length_mm")
            col.prop(self, "tip_diameter_mm")
            col.label(text="Max tip Ø for a working lead-in: %.3f mm" % (minor_r * 2.0))

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

        # ── Blank stack: [shank] -> thread blank -> [tip] (head added later) ─
        # Built as ONE chained revolve (see _add_chained_revolve) rather than
        # independently-capped stacked primitives, so the thread-blank-to-tip
        # junction (which sits at the exact same radius) doesn't end up with
        # two duplicate coincident cap faces there. The thread blank sits at
        # the FULL major_r — subtraction needs no undersized core the way the
        # old additive-union approach did.
        z = self.hex_length_mm  # everything sits above where the head will be unioned on

        # The thread cutter's crest lands need to poke slightly past major_r
        # so the DIFFERENCE has genuine volume to remove there, not a
        # hairline touch along the full helix length that the EXACT solver
        # can read as a no-op instead of a cut. Only the cutter is padded —
        # the blank itself stays at the real, undistorted major_r.
        overlap = max(0.02, min(0.2 * depth, 0.15))

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
        _add_cp(checkpoints, major_r, z)
        z += self.thread_length_mm
        _add_cp(checkpoints, major_r, z)

        if self.tip_enable:
            # Flat step straight down to tip_r at the thread's own end z, THEN
            # a straight (non-tapered) pin out to tip_length. Not a cone: a
            # cone here would taper the blank's diameter while the thread
            # cutter above is still cutting a constant-diameter helix through
            # it, so the thread would run out against a shrinking target and
            # dead-end right at the tip instead of just stopping cleanly —
            # exactly the "won't start in a nut" problem this fixes. The flat
            # step keeps the whole thread_length span at a uniform major_r for
            # the cutter, and the pin (meant to sit under minor_r — see
            # tip_diameter_mm's own doc) becomes an unthreaded lead-in.
            #
            # tip_r is clamped to just under minor_r, the same way gear
            # pressure_angle_deg gets clamped to its own max: silently, by
            # mutating the property itself so the redo panel shows the
            # corrected value, rather than erroring or leaving it to build a
            # tip that's too fat to work as a lead-in.
            max_tip_r = max(minor_r - overlap, 0.0)
            if self.tip_diameter_mm / 2.0 > max_tip_r:
                self.tip_diameter_mm = max_tip_r * 2.0
            tip_r = self.tip_diameter_mm / 2.0
            _add_cp(checkpoints, tip_r, z)
            z += self.tip_length_mm
            _add_cp(checkpoints, tip_r, z)

        bm = bmesh.new()
        _add_chained_revolve(bm, checkpoints, n)
        blank = _to_obj(bm, "__HexBoltBlank", context)
        blank.location = cursor

        # ── External thread, cut as a helical groove out of the blank ───────
        # _internal_profile is the same shape threaded_fastener.py's own
        # "External + Subtractive" mode uses — but note root_flat (2x
        # truncation x pitch) goes in the crest_flat argument slot, not cf,
        # or the crest/root flat proportions come out backwards. See the
        # module docstring.
        #
        # The cutter runs one extra pitch past thread_length_mm on purpose:
        # cut it exactly to length and the LAST crest just dead-ends flush at
        # the blank's own flat step, the same hard-stop look as before. The
        # extra pitch carries the helix into the tip pin (already thinner
        # than minor_r, so fully consumed) or, with no tip, into open space
        # past the blank's end — either way harmless, since a boolean
        # difference into material that isn't there does nothing.
        root_flat = 2.0 * self.truncation * self.pitch_mm
        cutter_bm = bmesh.new()
        prof = _internal_profile(major_r + overlap, minor_r, root_flat, fdz)
        _build_helix(cutter_bm, prof, self.pitch_mm, self.thread_length_mm + self.pitch_mm, n)
        cutter_obj = _to_obj(cutter_bm, "__HexBoltThreadCutter", context)
        cutter_obj.location = (cursor.x, cursor.y, cursor.z + thread_z0)

        _bool_diff(context, blank, cutter_obj)

        # ── Head, built separately and unioned on last ───────────────────────
        head_bm = bmesh.new()
        _add_hex_prism(head_bm, self.hex_across_flats_mm, 0.0, self.hex_length_mm)
        head = _to_obj(head_bm, "__HexBoltHead", context)
        head.location = cursor

        _bool_union(context, blank, head)
        blank.name = "HexBolt"

        fastener_matching.stamp_thread(blank, "hex_bolt", self.thread_diameter_mm,
                                        self.pitch_mm, self.flank_angle_deg, self.truncation)

        bpy.ops.object.select_all(action='DESELECT')
        blank.select_set(True)
        context.view_layer.objects.active = blank

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
