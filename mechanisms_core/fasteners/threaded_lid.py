"""
Threaded Lid Generator

An inverted cup that screws onto a threaded_container.py neck: solid cap
at the top, open skirt below, with an INTERNAL thread ADDED (unioned) onto
the skirt's already-hollow inner wall, anchored at the ceiling (+ a small
axial gap) and extending DOWN for thread_length_mm, above an unthreaded
GUIDE zone (down to the real mouth) that lets the container's neck pass
through freely before engaging the thread. thread_diameter_mm is both the
"OD of the container this lid fits" and the thread's own major diameter
(same surface, same convention threaded_container.py uses) — named to
match hex_bolt.py/hex_nut.py so `fastener_matching.sync_thread_dims` works
unmodified.

[REDESIGN] Every earlier version of this file cut the internal thread
SUBTRACTIVELY out of a pre-built solid plug (mirroring hex_nut.py's
recipe) — which required the thread's own region of the skirt to be built
solid first, a separate pilot bore, and a careful overshoot/clip dance at
both ends of the cut so the helical cutter's own wind-up seam never landed
inside real material (see git history for that version's full reasoning).
This version instead builds the WHOLE skirt hollow at major_r from the
start (a much simpler 6-point base profile, see below) and ADDS the
thread as a standalone helical ridge solid, unioned onto the wall. This
trades away the numerically "easy" case for the EXACT solver (a
DIFFERENCE against solid material always has genuine material to remove)
for a numerically harder one — hex_bolt.py's own docstring records that
this exact codebase tried additive threading once before and abandoned it
for that reason. Kept here anyway (a deliberate choice, not an oversight)
because the additive ridge only needs to touch the wall at all — not
terminate flush with any OTHER pre-existing boundary — so it never needs
hex_nut.py's overshoot/clip dance in the first place: the same radial
`overlap` padding hex_bolt.py's old additive mode used (pushing the
ridge's root radius past major_r by a depth-scaled amount, so the union
has real volume to merge instead of a hairline touch) is applied here,
and the ridge is allowed to hard-start/hard-stop in open bore air at both
ends, since additive material terminating in open air needs no special
treatment (same principle every subtractive cutter in this codebase
already relies on, just applied to the added solid instead of a removed
one).

Base profile (2D, r vs z, revolved 360° around Z), tracing checkpoints in
the order they're built — z=0 is the CEILING (top of the physical lid),
z=height is the real mouth (the open end the container's neck enters
through) — the reverse of this file's own PREVIOUS z convention, adopted
because it lets `height` fall out as a pure sum of its three parts
(cap + thread + guide) without needing a separate "skirt depth minus cap"
subtraction anywhere:
  (0, 0) pole — center of the solid cap's top exterior surface
  -> (outer_r, 0) — flat exterior top surface, pole to ring
  -> (outer_r, height) — down the full outer wall
  -> (major_r, height) — across the mouth's own annular rim (outer_r to
     major_r), the real opening the container's neck enters through
  -> (major_r, wall_thickness_mm) — up the (already-hollow, unthreaded-
     in-the-base-mesh) bore wall from the mouth rim to the ceiling's
     underside
  -> (0, wall_thickness_mm) pole — flat interior ceiling disc, closing
     the bottom of the solid cap
  -> closed=True back to (0, 0): both ends are poles on the same central
     axis, so `_bridge` skips building a face there (see its own
     docstring) — the cap (z=0 to wall_thickness_mm, r=0 to outer_r) is
     one uniformly solid region needing no internal face, same
     "pole-to-pole through solid material" reasoning every chained
     revolve in this codebase already relies on.
This produces a solid cap sitting above a fully hollow skirt (hollow at
major_r all the way from the ceiling's underside down to the real mouth)
— there is no separate "solid plug" region and no pre-hollowed pilot bore
distinction anymore, since the thread is added onto this uniform wall
rather than cut out of a thicker one.

`_add_chained_revolve`/`_to_obj` are duplicated from threaded_container.py
verbatim rather than imported — matches this family's established
"self-contained, duplicated verbatim" convention (see
docs/fasteners/README.md). `_thread_params`/`_build_helix` are duplicated
from hex_nut.py the same way. `_internal_profile` (crest pointing
inward — the correct shape for this file's additive ridge, see its own
docstring for why `_external_profile` was wrong here despite being
hex_nut.py's own choice for its unrelated subtractive cutter) is the same
shape function threaded_container.py duplicates too, though that file
uses it subtractively for its own external thread, with `root_flat` in
the argument slot rather than `crest_flat` — same shape, different
argument depending on subtractive-vs-additive, see threaded_fastener.py's
module docstring for where this shape's own "crest points inward, nut
ridge on inside of bore" name comes from.
"""

import bpy
import bmesh
from math import cos, sin, tan, pi, radians, ceil
from bpy.props import FloatProperty, IntProperty
from . import fastener_matching


BOOL_EPSILON = 0.001
_CHAIN_POLE_EPS = 0.01


def _add_chained_revolve(bm, checkpoints, n, cap_start=True, cap_end=True, closed=False):
    """See threaded_container.py's copy of this function for the full
    docstring — identical logic, duplicated per this family's convention.

    `closed=True` connects the LAST checkpoint back to the FIRST using the
    SAME vertex ring objects already built for checkpoint 0 — not a fresh
    duplicate ring at the same position.
    """
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


# ── Thread geometry (duplicated from hex_nut.py per family convention) ────────

def _thread_params(major_r, pitch, flank_deg, truncation):
    ha    = max(radians(flank_deg / 2.0), radians(0.5))
    cf    = truncation * pitch
    rf    = 2.0 * truncation * pitch
    fdz   = max((pitch - cf - rf) / 2.0, 0.0)
    depth = fdz / tan(ha) if fdz > 0 else 0.0
    return major_r - depth, cf, fdz, depth


def _min_truncation_for_max_depth(pitch, flank_deg, max_depth):
    """See threaded_container.py's copy of this function for the full
    derivation — identical formula, duplicated per this family's
    convention. Thread depth depends only on (pitch, flank_angle_deg,
    truncation), never on major_r/wall_thickness/diameter, so the same
    closed-form solve works for this file's "minor_r too small" clamp as
    well as the container's "depth vs. wall thickness" clamp. Returns the
    RAW (unclamped) result — see threaded_container.py's own docstring
    for why clamping internally here breaks the caller's own
    "irreconcilable if this exceeds 0.3" check."""
    ha = max(radians(flank_deg / 2.0), radians(0.5))
    return (1.0 - 2.0 * max_depth * tan(ha) / pitch) / 3.0


def _internal_profile(major_r, minor_r, crest_flat, flank_dz):
    """Crest points inward — "nut ridge on inside of bore"
    (threaded_fastener.py's own phrase for this exact shape), used here
    as the ADDITIVE ridge solid for an internal thread.

    [BUG, fixed] An earlier version of this file used `_external_profile`
    ("bolt ridge on outside of shaft") instead — copied verbatim from
    hex_nut.py's SUBTRACTIVE cutter, without re-deriving that a cutter's
    correct shape and an additive ridge's correct shape are NOT the same
    thing even when both are "for an internal thread." `_external_profile`
    wraps its two equal-radius points at `minor_r`, so the ridge it builds
    reaches `minor_r` for its ENTIRE "on" duration (not just briefly at a
    crest) — a flat-bottomed block, not a tapered thread tooth. This
    profile wraps its equal-radius points at `major_r` instead, so the
    ridge's attachment to the wall is the constant floor, and it only
    reaches down to `minor_r` briefly, at the `crest_flat`-length
    plateau, tapering through the flanks either side — the actual shape
    of a real trapezoidal thread tooth. Confirmed by the user directly
    from a Blender screenshot: the old profile produced a single boxy
    protruding block, not a spiral ridge — a defect no aggregate
    duty-cycle ratio check (this file's own earlier ray-cast test) could
    catch, since both profiles produce the same open/blocked angular
    ratio despite being shaped completely differently.

    `major_r` here should already include the radial `overlap` padding
    (see execute()) — this profile's own "root" reach IS the ridge's
    attachment radius to the surrounding wall, not a subtractive cutter's
    crest reach, so the padding has to be baked into the caller's
    `major_r` argument before it's passed in."""
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


def _bool_op(context, body, addend_or_cutter, operation):
    bpy.ops.object.select_all(action='DESELECT')
    body.select_set(True)
    context.view_layer.objects.active = body
    mod           = body.modifiers.new("Bool", 'BOOLEAN')
    mod.operation = operation
    mod.object    = addend_or_cutter
    mod.solver    = 'EXACT'
    with context.temp_override(active_object=body):
        bpy.ops.object.modifier_apply(modifier="Bool")
    bpy.data.objects.remove(addend_or_cutter, do_unlink=True)
    # [FIX, carried over from blender_extension_lessons.txt] EXACT-solver
    # UNIONs of complex meshes can come back with inconsistent face winding
    # even when genuinely manifold/solid — recalc unconditionally rather
    # than only when a defect is suspected.
    bmesh_tmp = bmesh.new()
    bmesh_tmp.from_mesh(body.data)
    bmesh.ops.recalc_face_normals(bmesh_tmp, faces=bmesh_tmp.faces[:])
    bmesh_tmp.to_mesh(body.data)
    bmesh_tmp.free()
    body.data.update()


# ── Operator ──────────────────────────────────────────────────────────────────

class OBJECT_OT_threaded_lid(bpy.types.Operator):
    """Screw-top jar lid with an internal thread added onto its skirt wall"""
    bl_idname  = "object.threaded_lid"
    bl_label   = "Threaded Lid"
    bl_options = {'REGISTER', 'UNDO'}

    def bmech_sync_target(self, context):
        fastener_matching.sync_thread_dims(self, context.window_manager.bmech_fastener_target)

    def invoke(self, context, event):
        fastener_matching.reset_target(context)
        return self.execute(context)

    thread_diameter_mm: FloatProperty(name="Thread Ø (mm)", default=60.0, min=5.0, soft_max=300.0,
                                      description="OD of the container this lid fits, AND the thread's "
                                                  "major diameter")
    wall_thickness_mm: FloatProperty(name="Wall Thickness (mm)", default=3.0, min=0.4, soft_max=20.0,
                                      description="Skirt wall AND cap thickness (one value, both places)")
    thread_length_mm:  FloatProperty(name="Thread Length (mm)", default=8.0, min=1.0, soft_max=90.0,
                                      description="How far down from the ceiling the thread extends")
    guide_length_mm:   FloatProperty(name="Guide Length (mm)", default=4.0, min=1.0, soft_max=90.0,
                                      description="Unthreaded lead-in below the thread, down to the real "
                                                  "mouth — lets the container's neck self-align before "
                                                  "engaging the thread")
    pitch_mm:          FloatProperty(name="Pitch (mm)", default=4.0, min=0.5, soft_max=20.0,
                                      description="Distance between thread crests — jar/bottle threads "
                                                  "are typically much coarser than fastener threads")
    flank_angle_deg:   FloatProperty(name="Flank Angle (°)", default=90.0, min=1.0, max=179.0,
                                      description="Wider than the 60° fastener default — "
                                                  "jar/bottle threads don't need a fastener's "
                                                  "self-locking taper")
    truncation:        FloatProperty(name="Truncation", default=0.25, min=0.0, max=0.3)
    inner_compensation_mm: FloatProperty(name="Compensation (mm)", default=0.0, min=0.0, soft_max=0.5,
                                      description="FDM: printed internal features come out tight — "
                                                  "added to thread major radius")
    fit_offset_mm:     FloatProperty(name="Fit Offset (mm)", default=0.0, min=0.0, soft_max=0.5,
                                      description="FDM: added to thread diameter for a looser, "
                                                  "better-fitting mesh against a mating external thread "
                                                  "(threaded_container) whose own diameter is reduced by "
                                                  "the same offset. Not synced by Match Target")
    resolution:        IntProperty(name="Resolution", default=64, min=8, soft_max=256)

    def _derived(self):
        major_r = self.thread_diameter_mm / 2.0 + self.inner_compensation_mm + self.fit_offset_mm / 2.0
        outer_r = major_r + self.wall_thickness_mm
        height  = self.wall_thickness_mm + self.thread_length_mm + self.guide_length_mm
        minor_r, cf, fdz, depth = _thread_params(
            major_r, self.pitch_mm, self.flank_angle_deg, self.truncation)
        return major_r, outer_r, height, minor_r, cf, fdz, depth

    def _clamp(self):
        """
        Silently correct out-of-range property combinations by mutating
        the properties themselves, instead of cancelling — same pattern
        as gear_matching.clamp_pressure_angle and
        threaded_container.py's own `_clamp`. Called once at the top of
        execute(). Returns a genuinely-irreconcilable-conflict message
        (or None) for the one case with no meaningful closest-valid-value
        to clamp to.

        Only two constraints remain here (down from three in every earlier
        version) — since `height` is now a pure derived sum rather than a
        fixed budget the other properties have to fit inside, there is no
        longer any "wall_thickness_mm vs. lid_height_mm" conflict to
        reconcile at all.
        """
        major_r = self.thread_diameter_mm / 2.0 + self.inner_compensation_mm + self.fit_offset_mm / 2.0

        # 1. truncation must be high enough that the thread ridge's own
        # minor_r stays positive with a real margin (not right at zero) —
        # depends only on major_r, same as every earlier version of this
        # file (wall_thickness plays no part: an internal thread's minor_r
        # is a property of major_r and the thread's own dimensions alone).
        margin = max(1.0, major_r * 0.05)
        max_depth = major_r - margin
        min_trunc = _min_truncation_for_max_depth(self.pitch_mm, self.flank_angle_deg, max_depth)
        if min_trunc > 0.3:
            return "Pitch too coarse for this thread diameter even at max truncation"
        if self.truncation < min_trunc:
            self.truncation = min_trunc

        # 2. guide_length_mm must leave a real, comfortably-thick unthreaded
        # gap between the thread's own end and the real mouth — confirmed
        # in the earlier subtractive design that anything under ~1mm here
        # is numerically fragile for the EXACT solver; the same margin is
        # kept as a floor for this additive ridge's own end, even though
        # the specific mechanism (a razor-thin exact-coincidence wall) no
        # longer applies the same way, since it's still the only place a
        # thin unthreaded lead-in could end up too close to the ridge's
        # own hard stop for the neck to actually self-align in it.
        clearance_margin = max(1.0, 0.25 * self.pitch_mm)
        if self.guide_length_mm < clearance_margin:
            self.guide_length_mm = clearance_margin

        # 3. thread_length_mm must exceed the thread profile's own axial
        # span (`profile_span = 2*fdz + crest_flat`, the height one full
        # crest-to-crest repeat occupies) by a real margin — not just be
        # positive. [BUG, fixed] `_build_helix` computes
        # `steps = ceil((height - profile_span) * res / pitch) + 1`; once
        # `thread_length_mm <= profile_span` this goes to zero or negative
        # rings, and `_build_helix` throws `IndexError` on `rings[0]`
        # outright (confirmed empirically — e.g. thread_length_mm=4 with
        # pitch_mm=8 crashes every time) rather than merely producing a
        # degenerate mesh. A 0.5-pitch margin comfortably guarantees
        # several real ring steps.
        _, cf, fdz, _ = _thread_params(major_r, self.pitch_mm, self.flank_angle_deg, self.truncation)
        min_thread_length = (2.0 * fdz + cf) + 0.5 * self.pitch_mm
        if self.thread_length_mm < min_thread_length:
            self.thread_length_mm = min_thread_length

        return None

    def draw(self, context):
        layout = self.layout
        major_r, outer_r, height, minor_r, cf, fdz, depth = self._derived()

        layout.prop(context.window_manager, "bmech_fastener_target", text="Match Target")
        has_target = context.window_manager.bmech_fastener_target is not None

        col = layout.column(align=True)
        driven = col.column(align=True)
        driven.enabled = not has_target
        driven.prop(self, "thread_diameter_mm")
        col.prop(self, "wall_thickness_mm")
        col.prop(self, "resolution")

        layout.separator()
        col = layout.column(align=True)
        col.prop(self, "thread_length_mm")
        col.prop(self, "guide_length_mm")
        driven2 = col.column(align=True)
        driven2.enabled = not has_target
        driven2.prop(self, "pitch_mm")
        driven2.prop(self, "flank_angle_deg")
        driven2.prop(self, "truncation")
        col.prop(self, "inner_compensation_mm")
        col.prop(self, "fit_offset_mm")

        layout.separator()
        box = layout.box()
        box.label(text="Outer Ø: %.2f mm" % (outer_r * 2.0))
        box.label(text="Height: %.2f mm (cap %.2f + thread %.2f + guide %.2f)"
                  % (height, self.wall_thickness_mm, self.thread_length_mm, self.guide_length_mm))
        box.label(text="Thread depth: %.3f mm" % depth)
        box.label(text="Thread minor Ø: %.2f mm" % (minor_r * 2.0))

        # These conditions are normally resolved automatically by _clamp()
        # in execute() before the panel ever redraws with these (now
        # already-corrected) values — see _clamp()'s own docstring.
        if fdz <= 0:
            layout.label(text="Truncation too high — no room for flanks at this pitch", icon='INFO')
        if minor_r <= 0:
            layout.label(text="Thread too deep for this diameter — minor Ø ≤ 0", icon='INFO')
        elif self.thread_length_mm < 1.5 * self.pitch_mm:
            # Carried over from every earlier version: confirmed empirically
            # that thread_length_mm == pitch_mm exactly (1.0 turns) produces
            # a real non-manifold defect — 1.5+ turns is always clean in
            # testing. This stays a genuine (non-clamped) warning since a
            # 1-1.49-turn thread is weak/poorly-engaging but not necessarily
            # mesh-broken at every value in that range.
            layout.label(text="Thread length < 1.5 pitches — weak engagement, and exactly "
                               "1.0 pitch is a confirmed mesh defect", icon='ERROR')

    def execute(self, context):
        irreconcilable = self._clamp()
        if irreconcilable is not None:
            self.report({'ERROR'}, irreconcilable)
            return {'CANCELLED'}

        major_r, outer_r, height, minor_r, cf, fdz, depth = self._derived()
        n = self.resolution
        cursor = context.scene.cursor.location.copy()

        checkpoints = [
            (0.0,     0.0),                       # exterior top pole
            (outer_r, 0.0),                       # flat exterior top surface
            (outer_r, height),                    # down the full outer wall
            (major_r, height),                    # mouth's own annular rim
            (major_r, self.wall_thickness_mm),    # up the hollow bore wall
            (0.0,     self.wall_thickness_mm),    # interior ceiling pole
        ]

        bm = bmesh.new()
        _add_chained_revolve(bm, checkpoints, n, closed=True)
        obj = _to_obj(bm, "ThreadedLid", context)
        obj.location = cursor

        # Additive thread ridge: a standalone helical solid (root reaching
        # to `major_r + overlap`, crest at `minor_r`), UNIONED onto the
        # already-hollow bore wall — see the module docstring's [REDESIGN]
        # note for why this file uses UNION instead of every other
        # generator's DIFFERENCE-into-solid-material approach, and why that
        # tradeoff is judged acceptable here specifically (no flush
        # boundary to land on at either end, since both ends terminate in
        # open bore air).
        #
        # `overlap`: the same depth-scaled radial padding hex_bolt.py's own
        # (since-abandoned) additive mode used, so the ridge's root has
        # genuine volumetric overlap with the wall instead of a hairline
        # touch at exactly major_r.
        overlap = max(0.02, min(0.2 * depth, 0.15))
        # `ceiling_eps`: a small axial gap between the ridge's own start and
        # the interior ceiling disc (at z=wall_thickness_mm), so the
        # ridge's flat end cap doesn't land exactly on that pre-existing
        # face — the same class of exact-coincidence hairline touch this
        # codebase has repeatedly found needs a deliberate small offset,
        # scaled to pitch since the ridge's own geometric detail scales the
        # same way.
        ceiling_eps = max(0.05, 0.05 * self.pitch_mm)
        thread_z0 = self.wall_thickness_mm + ceiling_eps

        # [BUG, fixed] `_internal_profile` (crest pointing inward) goes
        # here — NOT `_external_profile` (crest pointing outward), which
        # an earlier version of this file used, copied verbatim from
        # hex_nut.py's SUBTRACTIVE cutter without re-deriving that a
        # cutter's correct shape and an additive ridge's correct shape
        # aren't the same thing. `_external_profile` wraps its two
        # equal-radius points at `minor_r`, so `_build_helix` fills from
        # that constant floor up to a `major_r`-peaking ceiling — meaning
        # the ridge reaches `minor_r` for its ENTIRE "on" duration, not
        # just briefly at a crest: a flat-bottomed block, not a tapered
        # thread tooth. Confirmed directly from a user screenshot: the
        # old profile produced a single boxy protruding block, not a
        # spiral ridge — see `_internal_profile`'s own docstring for the
        # full geometric explanation, including why this file's earlier
        # ray-cast test (checking only the open/blocked angular RATIO)
        # couldn't catch it: both profiles give the same ~50/50 ratio
        # despite being shaped completely differently. `crest_flat` (not
        # `root_flat`) still goes in the argument slot — with
        # `_internal_profile`, that plateau is what actually lands at
        # `minor_r`, so the SHORT value has to be the one used there.
        ridge_bm = bmesh.new()
        prof = _internal_profile(major_r + overlap, minor_r, cf, fdz)
        _build_helix(ridge_bm, prof, self.pitch_mm, self.thread_length_mm, n)
        ridge = _to_obj(ridge_bm, "__LidThreadRidge", context)
        ridge.location = (cursor.x, cursor.y, cursor.z + thread_z0)

        _bool_op(context, obj, ridge, 'UNION')

        # Merge by Distance: cleans up whatever coincident geometry
        # remains from the union above, same as every other cut/union in
        # this codebase.
        merge_bm = bmesh.new()
        merge_bm.from_mesh(obj.data)
        bmesh.ops.remove_doubles(merge_bm, verts=merge_bm.verts[:], dist=BOOL_EPSILON / 10.0)
        bmesh.ops.recalc_face_normals(merge_bm, faces=merge_bm.faces[:])
        merge_bm.to_mesh(obj.data)
        merge_bm.free()
        obj.data.update()

        fastener_matching.stamp_thread(obj, "threaded_lid", self.thread_diameter_mm,
                                        self.pitch_mm, self.flank_angle_deg, self.truncation)

        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        self.report({'INFO'},
            "Threaded lid: fits %.1f mm OD, %.1f mm tall, %.2f mm wall"
            % (self.thread_diameter_mm, height, self.wall_thickness_mm))
        return {'FINISHED'}


# ── Registration ──────────────────────────────────────────────────────────────

classes = (OBJECT_OT_threaded_lid,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
