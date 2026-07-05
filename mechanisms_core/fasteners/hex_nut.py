"""
Hex Thru-Nut Generator

Subtractive construction: cut the internal thread out of a solid hex prism,
rather than cut an oversized round bore and union a separate ridge into it
(this file's own approach before this rewrite). Two DIFFERENT coincident
surfaces show up in this construction, and they were fixed with two
different techniques — conflating them (fixing one the way the other
needs) reintroduces whichever bug that approach doesn't cover:

  1. Cut the helical thread-groove cutter (_external_profile + _build_helix)
     out of the solid hex prism FIRST: EXACTLY major_r radially, but
     AXIALLY it deliberately overshoots HALF a pitch past BOTH z=0 and
     z_height_mm (see "Axial wind-up" below for why, and why half a pitch
     rather than a full one).
  2. Cut a plain pilot bore through the same prism SECOND, sized to
     minor_r + BOOL_EPSILON — a hairline OVER the thread's minor diameter,
     not exactly equal to it (see "Radial coincidence" below for why).
  3. Weld the result with a Merge by Distance pass (bmesh.ops.remove_doubles,
     a small fixed tolerance), which cleans up whatever coincident geometry
     remains from steps 1-2.

Radial coincidence (minor_r): the pilot bore's wall and the thread cutter's
own root-flat faces are both nominally at minor_r. The `overlap`-padding
pattern used elsewhere in this codebase (hex_bolt.py) fudges one of two
coincident surfaces slightly off its true dimension to dodge an EXACT-
solver ambiguity. An earlier version of this file tried undersizing the
pilot bore that way and found it traded mesh defects for a real, physical,
uncut lip at the bore's mouth, visibly narrower than the true minor
diameter — undersizing was the wrong direction, since the pilot bore
cutting AFTER the thread is what actually opens up the bore to its final
size; anything the pilot bore doesn't reach stays as an unwanted remnant of
the solid prism instead of empty bore. Oversizing the pilot bore slightly
(`+ BOOL_EPSILON`, applied after the thread cut, not before) avoids that
remnant while still breaking the exact radial coincidence.

Axial wind-up (z=0 / z=z_height_mm): a single-start helix cut EXACTLY
flush to the real end faces starts its very first ring — which also forms
the cutter's own flat end cap — before the thread profile has completed
even a quarter turn. The visible result is a flat, UNTHREADED band at both
mouths of the nut (every angle reads a uniform minor_r there, not the
oscillating crest/root pattern a real thread has), with the flat band's own
side walls looking like they wrongly bridge separate ridges. This is a
dimensional defect, not a topological one — it can pass a min-radius-only
check completely undetected, since the flat band happens to sit exactly at
minor_r too. It needs axial overshoot, so the helix is already fully wound
up into a repeating cycle by the time it reaches the real z=0/z_height_mm
faces. The overshoot amount matters: a FULL pitch of overshoot places the
exact same profile phase at z=0 that would land there anyway (the crest's
return to minor_r), which is exactly coincident with the pilot bore's own
wall over an extended stretch and produced a genuine torn hole in one
configuration tested (real non-manifold boundary edges from a missing
face, not duplicate geometry — Merge by Distance cannot weld that shut).
HALF a pitch lands a different, non-minor_r phase at z=0 instead, avoiding
that extra coincidence while still providing wind-up room.

Both the pilot-bore-after-thread-cut ordering and the minor_r + BOOL_EPSILON
sizing were arrived at through direct iteration in Blender by Blake (not
verified independently in this file's own headless test suite before
shipping) — trust the working combination over re-deriving it from the
reasoning above if the two ever seem to disagree.

Cutter profile note: root_flat (2 x truncation x pitch), not crest_flat
(truncation x pitch), goes in _external_profile's crest_flat argument slot
for this to come out the standard way round — narrow crest at minor_r, wide
root at major_r. Passing crest_flat straight through produces a valid but
backwards thread. Verified empirically in a headless Blender test before
relying on it — see the conversation history, not worth re-deriving from
the profile math alone.

Thread math is duplicated from threaded_fastener.py per this project's
convention (each generator module is self-contained, no cross-file thread
math imports).
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil, sqrt
from bpy.props import FloatProperty, IntProperty
from . import fastener_matching

BOOL_EPSILON = 0.001


# ── Thread geometry (duplicated from threaded_fastener.py) ────────────────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _external_profile(major_r, minor_r, crest_flat, flank_dz):
    """
    Crest points outward. Used here as the SUBTRACTIVE cutter for an
    internal thread — see the module docstring for why crest_flat must be
    passed root_flat (2 x truncation x pitch), not truncation x pitch, to
    get the crest/root proportions the right way round.
    """
    return [
        (minor_r, 0.0),
        (major_r, flank_dz),
        (major_r, flank_dz + crest_flat),
        (minor_r, flank_dz * 2.0 + crest_flat),
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


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_hex_nut(bpy.types.Operator):
    """Hex thru-nut — internal thread cut all the way through a hex prism."""
    bl_idname  = "object.hex_nut"
    bl_label   = "Hex Nut"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        fastener_matching.sync_thread_dims(self, context.window_manager.bmech_fastener_target)

    def invoke(self, context, event):
        fastener_matching.reset_target(context)
        return self.execute(context)

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

        layout.prop(context.window_manager, "bmech_fastener_target", text="Match Target")
        has_target = context.window_manager.bmech_fastener_target is not None

        col = layout.column(align=True)
        col.prop(self, "z_height_mm")
        col.prop(self, "across_flats_mm")

        layout.separator()
        col = layout.column(align=True)
        # All four freeze together whenever a target is set — see
        # hex_bolt.py's draw() for why there's no partial-match case here.
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "thread_diameter_mm")
        driven.prop(self, "pitch_mm")
        driven.prop(self, "flank_angle_deg")
        driven.prop(self, "truncation")
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

        # Thread cutter, cut FIRST (before the pilot bore below). EXACTLY
        # major_r radially. AXIALLY it deliberately overshoots HALF a pitch
        # past BOTH z=0 and z_height_mm, so the helix is already wound up
        # into a repeating crest/root cycle by the time it reaches the real
        # end faces — cut exactly flush instead, the cutter's very first/
        # last ring (also its own flat end cap) lands before the profile
        # has completed even a quarter turn, leaving a flat UNTHREADED band
        # at both mouths of the nut. See the module docstring for why half
        # a pitch specifically, not a full one. root_flat (not crest_flat)
        # goes in the crest_flat argument slot — see module docstring.
        shift = 0.5 * self.pitch_mm
        root_flat = 2.0 * self.truncation * self.pitch_mm
        cutter_bm = bmesh.new()
        prof = _external_profile(major_r, minor_r, root_flat, fdz)
        _build_helix(cutter_bm, prof, self.pitch_mm, self.z_height_mm + 2.0 * shift, self.resolution)
        cutter = _to_obj(cutter_bm, "__HexNutThreadCutter", context)
        cutter.location = (cursor.x, cursor.y, cursor.z - shift)
        _bool_diff(context, body, cutter)

        # Pilot bore, cut SECOND (after the thread cutter above, not
        # before), sized to minor_r + BOOL_EPSILON — a hairline OVER the
        # true minor diameter, not exactly equal to it. This ordering and
        # sizing is what actually opens the thread up to its final bore
        # diameter; see the module docstring for why the reverse order
        # (bore first, undersized) left an uncut lip instead.
        pilot_bm = bmesh.new()
        _add_cyl_z(pilot_bm, minor_r + BOOL_EPSILON,
                   -BOOL_EPSILON, self.z_height_mm + BOOL_EPSILON, self.resolution)
        pilot_cutter = _to_obj(pilot_bm, "__HexNutPilot", context)
        pilot_cutter.location = cursor
        _bool_diff(context, body, pilot_cutter)

        # Merge by Distance: cleans up whatever coincident geometry remains
        # from the two cuts above (see module docstring) rather than trying
        # to dodge every coincidence with padding beforehand.
        merge_bm = bmesh.new()
        merge_bm.from_mesh(body.data)
        bmesh.ops.remove_doubles(merge_bm, verts=merge_bm.verts[:], dist=BOOL_EPSILON / 10.0)
        bmesh.ops.recalc_face_normals(merge_bm, faces=merge_bm.faces[:])
        merge_bm.to_mesh(body.data)
        merge_bm.free()
        body.data.update()

        fastener_matching.stamp_thread(body, "hex_nut", self.thread_diameter_mm,
                                        self.pitch_mm, self.flank_angle_deg, self.truncation)

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
