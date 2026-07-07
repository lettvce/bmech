"""
Threaded Container Generator

A screw-top jar body: solid floor, straight outer wall, open mouth at the
top, with an EXTERNAL thread cut directly into the existing outer wall
near the top — no separate raised neck boss. thread_diameter_mm doubles
as both the container's own outer diameter AND the thread's major
diameter (they're the same surface), matching hex_bolt.py/hex_nut.py's
convention of naming the shared Match Target dimension `thread_diameter_mm`
even where a more specific name (here, "container OD") would also fit —
this lets `fastener_matching.sync_thread_dims` work unmodified.

Base profile (2D, r vs z, revolved 360° around the Z axis) — a "C" shape
read bottom to top: outer-bottom pole -> up the outer wall -> across the
open top rim -> down the inner wall -> inner-floor pole. Both ends of
this chain are poles (r=0), so the container is fully closed at the
bottom (solid floor) and naturally open at the top (the rim is a flat
annulus, not a capped disc) — no boolean needed for the basic hollow
shape, the same "chained revolve" technique hex_bolt.py uses for its
shank/thread/tip stack, just applied to a non-monotonic (out-then-back-in)
radius profile instead of a monotonic one.

Thread cut: the SAME technique as hex_bolt.py's external thread — a
helical groove cutter (`_internal_profile`, crest pointing inward, used
subtractively) differenced out of the EXISTING solid wall near the top.
This works directly, with no base-shape rework, because the container's
outer wall is ALREADY solid at exactly the thread's major radius (unlike
threaded_lid.py's internal thread, which needed its base shape reworked
into a solid plug first — see that file's own docstring for why an
internal thread can't be cut the same simple way).

`cap_start`/`cap_end`/`closed` flags on the chained-revolve helper —
hex_bolt.py's own version always caps a non-pole end. Not needed for this
file's own base shape (both ends already are poles), but threaded_lid.py
(built the same session) needs both, and duplicating an identical copy
here keeps both files' revolve helpers byte-for-byte the same, matching
this family's "self-contained, duplicated verbatim" convention.
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil
from bpy.props import FloatProperty, IntProperty
from . import fastener_matching


BOOL_EPSILON = 0.001
_CHAIN_POLE_EPS = 0.01


def _add_chained_revolve(bm, checkpoints, n, cap_start=True, cap_end=True, closed=False):
    """See threaded_lid.py's copy of this function for the full docstring
    — identical logic, duplicated per this family's convention."""
    ang = [2.0 * pi * i / n for i in range(n)]

    point_verts = []
    for r, z in checkpoints:
        if r <= _CHAIN_POLE_EPS:
            point_verts.append(bm.verts.new((0.0, 0.0, z)))
        else:
            point_verts.append([bm.verts.new((r * cos(a), r * sin(a), z)) for a in ang])
    bm.verts.index_update()

    def _bridge(a_r, a_v, b_r, b_v):
        a_pole = a_r <= _CHAIN_POLE_EPS
        b_pole = b_r <= _CHAIN_POLE_EPS
        if a_pole and b_pole:
            return
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

    for k in range(len(checkpoints) - 1):
        a_r, _ = checkpoints[k]
        b_r, _ = checkpoints[k + 1]
        _bridge(a_r, point_verts[k], b_r, point_verts[k + 1])

    if closed:
        last_r, _ = checkpoints[-1]
        first_r, _ = checkpoints[0]
        _bridge(last_r, point_verts[-1], first_r, point_verts[0])
        return

    first_r, _ = checkpoints[0]
    last_r, _ = checkpoints[-1]
    if first_r > _CHAIN_POLE_EPS and cap_start:
        bm.faces.new(list(reversed(point_verts[0])))
    if last_r > _CHAIN_POLE_EPS and cap_end:
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


# ── Thread geometry (duplicated from hex_bolt.py per family convention) ───────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _min_truncation_for_max_depth(pitch, flank_deg, max_depth):
    """
    Smallest `truncation` that keeps thread depth <= max_depth for a given
    pitch/flank_angle. Thread depth depends ONLY on (pitch, flank_angle_deg,
    truncation) — not on major_r/wall_thickness/diameter at all — so this
    same closed-form solve works for any "depth is too much for this
    part's size" case (see clamp_pressure_angle in gears/gear_matching.py
    for the analogous gear-family pattern this mirrors: solve for the
    property value that exactly satisfies the constraint, then clamp the
    operator's own property to it, rather than cancelling the operator).

    Derivation: depth = fdz/tan(ha), fdz = pitch*(1 - 3*truncation)/2.
    Solving depth <= max_depth for truncation gives the formula below.

    [BUG, fixed] Returns the RAW (unclamped) result — an earlier version
    clamped this to truncation's own valid range [0, 0.3] internally,
    which made the caller's own "if min_trunc > 0.3: irreconcilable"
    check impossible to ever trigger (the value it was checking had
    already been silently capped at exactly 0.3, never above). The
    caller is responsible for checking whether the raw result exceeds
    0.3 (meaning no valid truncation can achieve the requested max_depth)
    before clamping it for actual use.
    """
    ha = max(radians(flank_deg / 2.0), radians(0.5))
    return (1.0 - 2.0 * max_depth * tan(ha) / pitch) / 3.0


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward. Used here as the SUBTRACTIVE cutter for an
    external thread — see hex_bolt.py's own module docstring for why
    crest_flat must be passed root_flat (2 x truncation x pitch), not
    truncation x pitch, to get the crest/root proportions right."""
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

class OBJECT_OT_threaded_container(bpy.types.Operator):
    """Screw-top jar body with an external thread cut into its neck"""
    bl_idname  = "object.threaded_container"
    bl_label   = "Threaded Container"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        fastener_matching.sync_thread_dims(self, context.window_manager.bmech_fastener_target)

    def invoke(self, context, event):
        fastener_matching.reset_target(context)
        return self.execute(context)

    thread_diameter_mm: FloatProperty(name="Thread Ø (mm)", default=60.0, min=5.0, soft_max=300.0,
                                      description="Container OD AND the thread's major diameter — "
                                                  "the thread cuts directly into this existing wall")
    wall_thickness_mm: FloatProperty(name="Wall Thickness (mm)", default=3.0, min=0.4, soft_max=20.0,
                                      description="Side wall AND floor thickness (one value, both places)")
    height_mm:         FloatProperty(name="Height (mm)", default=60.0, min=1.0, soft_max=500.0,
                                      description="Total outer height, floor to rim")
    thread_length_mm:  FloatProperty(name="Thread Length (mm)", default=10.0, min=1.0, soft_max=100.0,
                                      description="How far down from the rim the thread extends")
    pitch_mm:          FloatProperty(name="Pitch (mm)", default=4.0, min=0.5, soft_max=20.0,
                                      description="Distance between thread crests — jar/bottle threads "
                                                  "are typically much coarser than fastener threads")
    flank_angle_deg:   FloatProperty(name="Flank Angle (°)", default=30.0, min=1.0, max=179.0,
                                      description="Jar/bottle threads are typically shallower "
                                                  "(30° buttress-style) than the 60° fastener default")
    truncation:        FloatProperty(name="Truncation", default=0.25, min=0.0, max=0.3)
    outer_compensation_mm: FloatProperty(name="Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
                                      description="FDM: printed external features tend to shrink — "
                                                  "added to thread major radius")
    fit_offset_mm:     FloatProperty(name="Fit Offset (mm)", default=0.0, min=0.0, soft_max=0.5,
                                      description="FDM: subtracted from thread diameter for a looser, "
                                                  "better-fitting mesh against a mating internal thread "
                                                  "(threaded_lid) whose own diameter is increased by the "
                                                  "same offset. Not synced by Match Target")
    resolution:        IntProperty(name="Resolution", default=64, min=8, soft_max=256)

    def _derived(self):
        outer_r = self.thread_diameter_mm / 2.0 + self.outer_compensation_mm - self.fit_offset_mm / 2.0
        inner_r = outer_r - self.wall_thickness_mm
        floor_z = self.wall_thickness_mm
        minor_r, cf, fdz, depth = _thread_params(
            outer_r, self.pitch_mm, self.flank_angle_deg, self.truncation)
        return outer_r, inner_r, floor_z, minor_r, cf, fdz, depth

    def _clamp(self):
        """
        Silently correct out-of-range property combinations by mutating
        the properties themselves — the same pattern
        gear_matching.clamp_pressure_angle and hex_bolt.py's own
        tip_diameter_mm clamp use, so the redo panel shows the corrected
        value instead of the operator just cancelling. Called once at the
        top of execute(), before any geometry is built. Returns a
        genuinely-irreconcilable-conflict message (or None) for cases with
        no meaningful "closest valid value" to clamp to — those still
        cancel, matching bevel_gear.py's own precedent for hard
        impossibilities a clamp can't paper over.
        """
        outer_r = self.thread_diameter_mm / 2.0 + self.outer_compensation_mm - self.fit_offset_mm / 2.0

        # 1. wall_thickness_mm must leave a real interior cavity: less
        # than both outer_r (or there's no inner wall at all) and
        # height_mm (or there's no floor-to-rim cavity). 0.1mm margin so
        # the result isn't a zero-thickness degenerate cavity.
        max_wall = min(outer_r, self.height_mm) - 0.1
        if max_wall <= 0:
            return "OD/height too small for any positive wall thickness"
        if self.wall_thickness_mm > max_wall:
            self.wall_thickness_mm = max_wall

        # 2. truncation must be high enough that thread depth stays safely
        # under the (now-valid) wall thickness — see
        # _min_truncation_for_max_depth's own docstring for the closed-form
        # solve. Uses the same 70%-of-wall margin as the old warning
        # threshold, so a clamp never lands right at the fragile boundary.
        max_depth = self.wall_thickness_mm * 0.7
        min_trunc = _min_truncation_for_max_depth(self.pitch_mm, self.flank_angle_deg, max_depth)
        if min_trunc > 0.3:
            return "Pitch too coarse for this wall thickness even at max truncation"
        if self.truncation < min_trunc:
            self.truncation = min_trunc

        # 3. thread_length_mm can't exceed the available wall above the
        # floor. Respects the property's own min=1.0.
        floor_z = self.wall_thickness_mm
        max_thread_length = max(1.0, self.height_mm - floor_z)
        if self.thread_length_mm > max_thread_length:
            self.thread_length_mm = max_thread_length

        return None

    def draw(self, context):
        layout = self.layout
        outer_r, inner_r, floor_z, minor_r, cf, fdz, depth = self._derived()

        layout.prop(context.window_manager, "bmech_fastener_target", text="Match Target")
        has_target = context.window_manager.bmech_fastener_target is not None

        col = layout.column(align=True)
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "thread_diameter_mm")
        col.prop(self, "wall_thickness_mm")
        col.prop(self, "height_mm")
        col.prop(self, "resolution")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "thread_length_mm")
        driven2 = col.column(align=True)
        driven2.enabled = not has_target
        driven2.prop(self, "pitch_mm")
        driven2.prop(self, "flank_angle_deg")
        driven2.prop(self, "truncation")
        col.prop(self, "outer_compensation_mm")
        col.prop(self, "fit_offset_mm")

        layout.separator()
        box = layout.box()
        box.label(text="Interior Ø: %.2f mm" % (inner_r * 2.0))
        box.label(text="Interior depth: %.2f mm" % (self.height_mm - floor_z))
        box.label(text="Thread depth: %.3f mm" % depth)
        box.label(text="Thread minor Ø: %.2f mm" % (minor_r * 2.0))

        # These conditions are normally resolved automatically by _clamp()
        # in execute() before the panel ever redraws with these (now
        # already-corrected) values — see _clamp()'s own docstring. Kept
        # here as a bottom-of-panel, non-blocking info label rather than
        # removed outright: it's the one place a genuinely irreconcilable
        # combination (see _clamp()'s two "return a message" cases) is
        # surfaced, and it also covers fdz<=0, which can't actually occur
        # given truncation's own max=0.3 but costs nothing to keep as a
        # defensive display.
        if inner_r <= 0 or floor_z >= self.height_mm:
            layout.label(text="Wall too thick for this OD/height — no interior cavity", icon='INFO')
        if fdz <= 0:
            layout.label(text="Truncation too high — no room for flanks at this pitch", icon='INFO')
        if depth >= self.wall_thickness_mm * 0.7:
            layout.label(text="Thread depth near wall thickness limit", icon='INFO')
        if self.thread_length_mm > (self.height_mm - floor_z):
            layout.label(text="Thread length near container height limit", icon='INFO')

    def execute(self, context):
        irreconcilable = self._clamp()
        if irreconcilable is not None:
            self.report({'ERROR'}, irreconcilable)
            return {'CANCELLED'}

        outer_r, inner_r, floor_z, minor_r, cf, fdz, depth = self._derived()
        n = self.resolution
        cursor = context.scene.cursor.location.copy()

        checkpoints = [
            (0.0,     0.0),
            (outer_r, 0.0),
            (outer_r, self.height_mm),
            (inner_r, self.height_mm),
            (inner_r, floor_z),
            (0.0,     floor_z),
        ]

        bm = bmesh.new()
        _add_chained_revolve(bm, checkpoints, n)
        obj = _to_obj(bm, "ThreadedContainer", context)
        obj.location = cursor

        # Thread cutter: same technique as hex_bolt.py's external thread —
        # crest-inward cutter, root_flat swapped into the crest_flat slot,
        # depth-scaled radial overlap so the EXACT solver has genuine
        # volume to remove (not a hairline touch at the wall's own true
        # major_r), one full pitch of axial overshoot past the top so the
        # last crest doesn't dead-end flush at the open rim (harmless —
        # the overshoot extends into open air above the container, and a
        # boolean difference into material that isn't there does nothing).
        overlap = max(0.02, min(0.2 * depth, 0.15))
        root_flat = 2.0 * self.truncation * self.pitch_mm
        thread_z0 = self.height_mm - self.thread_length_mm

        cutter_bm = bmesh.new()
        prof = _internal_profile(outer_r + overlap, minor_r, root_flat, fdz)
        _build_helix(cutter_bm, prof, self.pitch_mm, self.thread_length_mm + self.pitch_mm, n)
        cutter_obj = _to_obj(cutter_bm, "__ContainerThreadCutter", context)
        cutter_obj.location = (cursor.x, cursor.y, cursor.z + thread_z0)

        _bool_diff(context, obj, cutter_obj)

        # Cleanup: the EXACT solver leaves a handful of zero-area sliver
        # faces (two numerically-identical vertices within one triangle)
        # from this cut, independent of the cutter's own overshoot amount
        # or resolution (confirmed empirically — varying overshoot from 0
        # to 2 full pitches never changed the defect count, ruling out a
        # wind-up/coincidence theory; only the mesh resolution did).
        # `dissolve_degenerate` is bmesh's dedicated tool for exactly this
        # class of artifact and removes it cleanly.
        clean_bm = bmesh.new()
        clean_bm.from_mesh(obj.data)
        bmesh.ops.dissolve_degenerate(clean_bm, dist=0.0001, edges=clean_bm.edges[:])
        bmesh.ops.recalc_face_normals(clean_bm, faces=clean_bm.faces[:])
        clean_bm.to_mesh(obj.data)
        clean_bm.free()
        obj.data.update()

        fastener_matching.stamp_thread(obj, "threaded_container", self.thread_diameter_mm,
                                        self.pitch_mm, self.flank_angle_deg, self.truncation)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'},
            "Threaded container: %.1f mm OD thread, %.1f mm tall, %.2f mm wall"
            % (self.thread_diameter_mm, self.height_mm, self.wall_thickness_mm))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_threaded_container,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
