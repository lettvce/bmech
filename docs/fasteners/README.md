# Fastener Family — Shared Conventions

Covers everything true for every generator in `mechanisms_core/fasteners/`.
Primitive-specific docs (one per generator, in this folder) assume you've
read this first. See [../CONVENTIONS.md](../CONVENTIONS.md) for the
cross-family rules this family follows, and [../gears/README.md](../gears/README.md)
for a contrasting example of how a different family organizes its shared
code.

## Primitives in this family

| Primitive | File | Builds |
|---|---|---|
| Raw threaded fastener | `fasteners/threaded_fastener.py` | A thread helix solid, for manual boolean combination |
| Hex bolt | `fasteners/hex_bolt.py` | Head + shank + tip blank, chained revolve, with the external thread cut out of it, head unioned on last |
| Hex nut | `fasteners/hex_nut.py` | Hex prism with an internal thread cut through it |
| Threaded container | `fasteners/threaded_container.py` | Screw-top jar body — solid floor, open mouth, external thread cut into the existing neck wall |
| Threaded lid | `fasteners/threaded_lid.py` | Screw-top jar lid — solid cap, open skirt, internal thread ADDED (unioned) onto the hollow bore wall above an unthreaded guide zone |

## This family does NOT share a helper module — and that's deliberate

Unlike the gear family (which centralizes shared logic in
`gears/gear_matching.py` and has every gear import it), **every fastener
generator module is fully self-contained.** `hex_bolt.py` and `hex_nut.py`
each carry their own copies of `_thread_params`, `_build_helix`, and
either `_external_profile` or `_internal_profile` — logic that
originates in `threaded_fastener.py` and is duplicated **verbatim**, not
imported. `hex_nut.py`'s source states the reason directly:

> "Thread math is duplicated from threaded_fastener.py per this project's
> convention (each generator module is self-contained, no cross-file
> thread math imports)."

`hex_bolt.py` and `hex_nut.py` also independently duplicate their own
hex-prism/cylinder/boolean-union helpers between each other, extending the
same convention beyond just thread math.

**If you're adding a new fastener generator that needs thread geometry,
copy `_thread_params`/`_build_helix`/the relevant profile function from
`threaded_fastener.py` into your new file rather than importing them.**
This is the opposite instinct from the gear family, where reuse via
import is preferred — know which family you're in before reaching for
either pattern.

## Match Target: a deliberate exception to the no-shared-module rule

`hex_bolt.py` and `hex_nut.py` support Match Target — pick a nut as a
bolt's target (or vice versa) and its `thread_diameter_mm`, `pitch_mm`,
`flank_angle_deg`, and `truncation` all copy across at once, the same
picker/freeze/rebuild-in-place UX the gear family established in
[gears/gear_matching.py](../gears/README.md#the-match-target-system-gear_matchingpy).
See [CONVENTIONS.md#the-match-target-pattern--letting-one-parts-output-drive-anothers-properties](../CONVENTIONS.md#the-match-target-pattern--letting-one-parts-output-drive-anothers-properties)
for the general Blender-API gotchas this pattern runs into regardless of
family (picker-on-`WindowManager`, reset-in-`invoke`-not-`execute`,
treating a reset-to-`None` as a no-op, rebuild-via-delete-and-re-execute
instead of `bpy.ops.ed.undo()`, and the fact that none of the pick/reset/
rebuild behavior can be exercised by a headless test).

This lives in a small shared module, `fasteners/fastener_matching.py` —
which looks like it contradicts the "no shared module" rule stated above,
but doesn't: that rule is specifically about **thread geometry math**
(`_thread_params`, `_build_helix`, the profile builders), which is pure
per-call computation with no reason it couldn't be duplicated safely. The
Match Target *picker* is a single `WindowManager` property, a genuinely
different kind of thing — it can't be meaningfully duplicated at all: two
independent `register()` calls each doing
`bpy.types.WindowManager.bmech_fastener_target = PointerProperty(...)`
wouldn't coexist as two working copies, the second one to run would just
silently replace the first's poll/update functions. `fastener_matching.py`
holds only that picker/poll/sync/stamp machinery — no thread math at all —
and is registered once from `fasteners/__init__.py`, the same way
`gears/gear_matching.py` is registered once from `gears/__init__.py`.

**No partial-match nuance, unlike gears.** The gear family's Match Target
freezing is kind-dependent (a spur target drives module/pressure-angle
but never helix/hand; a herringbone-vs-helical pairing leaves `hand`
editable even when driven — see
[gears/README.md](../gears/README.md#the-match-target-system-gear_matchingpy)).
A bolt and the nut that fits it need **all four** thread dimensions
(diameter, pitch, flank angle, truncation) to match simultaneously for
the threads to physically engage — there's no case where only some of
them matter — so `fastener_matching.sync_thread_dims` always copies all
four together, and both operators' `draw()` freeze all four as one driven
column, not field-by-field.

**The poll is deliberately NOT loose here — the opposite of the gear
family's own established reasoning.** `gear_target_poll` accepts any
object stamped with `bmech_module` regardless of kind, because cross-kind
gear matches are sometimes legitimate (see
[gears/README.md](../gears/README.md#the-match-target-system-gear_matchingpy)).
Threads have no equivalent legitimate same-orientation case: two external
threads can never mate, and neither can two internal ones — it's a hard
physical constraint of the domain, not a matter of user judgment the way
gear meshing correctness is. `fastener_target_poll` enforces this
directly: it only offers objects whose orientation (external/internal) is
the **opposite** of whichever operator is asking.

Enforcing that needs the poll to know which orientation the asking
operator itself has: `hex_bolt.py` is permanently `EXTERNAL`, `hex_nut.py`
permanently `INTERNAL`, but `threaded_fastener.py` can be either,
decided live by its own `thread_type` property. Since a `PointerProperty`
poll callback only receives `(self, object)` — no operator reference —
`fastener_target_poll` reads `bpy.context.active_operator` directly to
identify the caller (the same global-context pattern this system's own
update callback already relies on) and looks up `bl_idname` to decide
which orientation to require. If the asking operator can't be identified
(headless mode, or some future fastener generator this poll doesn't know
about yet), it falls back to permissive — same "don't hard-fail when
context is unavailable" instinct as the rest of this pattern.

`threaded_fastener.py` participates fully as of this feature — it stamps
`bmech_kind` as `"external_thread"`/`"internal_thread"` based on its own
`thread_type` (not `operation`; a `SUBTRACTIVE` cutter still represents
the orientation it will leave behind once you finish the boolean by
hand). Picking a target on it does double duty: copies the four thread
dimensions like the hex generators, **and** forces `thread_type` to the
opposite of the target's orientation (`operation` stays free — it only
controls how the resulting thread gets built, not whether it fits). See
[threaded_fastener.md](threaded_fastener.md#match-target) for the full
writeup.

## Thread nomenclature: nominal size is always major diameter

Every thread-based generator (`threaded_fastener.py`, `hex_bolt.py`,
`hex_nut.py`) treats the user-facing diameter property as the **major**
diameter, for both external (bolt) and internal (nut) threads — e.g. an
M8 property means an 8.000mm major diameter on both the bolt and the nut
that fits it. `hex_nut.py`'s source states this explicitly:

> "Standard thread nomenclature: the nominal/basic size IS the major
> diameter, for both external (bolt) and internal (nut) threads... The
> minor diameter... is DERIVED from major_r via pitch + flank angle, not
> the other way around."

The minor diameter is **never** a user input anywhere in this family —
always computed by `_thread_params(major_r, pitch_mm, flank_angle_deg,
truncation)`:

```
half_angle  = max(radians(flank_angle_deg / 2), radians(0.5))   # guards tan(0)
crest_flat  = truncation * pitch_mm
root_flat   = 2 * truncation * pitch_mm
flank_dz    = max((pitch_mm - crest_flat - root_flat) / 2, 0.0)  # clamped, not negative
depth       = flank_dz / tan(half_angle)   if flank_dz > 0  else 0
minor_r     = major_r - depth
```

## FDM compensation: always added, one direction per field

Every compensation property in this family is **added** to a radius or
diameter, never subtracted — consistent with the rest of this library.
Which field you set depends on which side of the part is printed:

- **External features that print undersized** (a bolt's thread):
  compensation is added to grow the feature back out.
- **Internal features that print tight** (a nut's thread bore, a cut
  hole): compensation is added to open the hole back up.

`threaded_fastener.py` exposes both `outer_compensation_mm` (External +
Additive mode) and `inner_compensation_mm` (Subtractive or Internal +
Additive modes) on the same operator, since one operator covers all four
mode combinations. `hex_bolt.py` only exposes `outer_compensation_mm`
(it only ever builds External + Additive). `hex_nut.py` only exposes
`inner_compensation_mm`.

## Boolean-solver patterns specific to this family

Fastener geometry leans on Blender's `EXACT` boolean solver far more
heavily than the gear family. Both `hex_bolt.py` and `hex_nut.py` cut
their threads **subtractively** — a helical groove cutter differenced out
of a solid blank — rather than unioning a separate thread ridge onto an
undersized core (`threaded_fastener.py`'s "Additive" modes still work
that way, and are documented separately, but the two hex-part generators
were rewritten off of it). A plain difference against a solid blank is a
numerically easy case for the EXACT solver: the cutter naturally overlaps
real material throughout, unlike a union of two independently-built,
nearly-coincident curved surfaces. Some patterns that show up repeatedly
and are worth knowing before editing this code:

- **Cutter overlap is depth-scaled, not a fixed hairline epsilon —
  `hex_bolt.py` only.** `hex_bolt.py` computes
  `overlap = max(0.02, min(0.2 * depth, 0.15))` and builds the thread
  cutter's crest reach slightly past the blank's true major radius before
  differencing it, and overshoots the cutter one extra pitch past
  `thread_length_mm` so the last crest doesn't dead-end flush at the
  blank's own flat step (a hard-stop that used to make the thread fail to
  start in a nut). Two nearly-coincident surfaces along a full helix
  length are numerically fragile for the EXACT solver — a hairline gap
  can be misread as a shared boundary and read as a no-op instead of a
  genuine cut, or a cutter's flat end cap landing exactly on a real face
  can leave the thread short of that face. A small but real volume of
  overlap/overshoot avoids both.
- **`hex_nut.py` mixes two different fixes for two different coincidences
  — it is NOT simply "exact dimensions, no padding."** It differences the
  thread cutter FIRST (exactly `major_r` radially, half a pitch of axial
  overshoot past both `z=0` and `z=z_height_mm`), then cuts a plain pilot
  bore SECOND, sized to `minor_r + BOOL_EPSILON` — a hairline over the true
  minor diameter, not exactly equal to it. Treating both coincidences (the
  radial one at `minor_r`, the axial one at the end faces) the same way
  breaks one or the other:
  - The **radial** coincidence (pilot bore wall vs. thread cutter's
    root-flat faces, both nominally at `minor_r`) — an earlier version cut
    the bore *first*, sized to exactly `minor_r`, and found that undersizing
    it further the way `hex_bolt.py`'s `overlap` pattern would left a real,
    physical, uncut lip at the bore's mouth, narrower than the true minor
    diameter — not a cosmetic mesh artifact. Cutting the thread first and
    the pilot bore second, oversized by a hairline rather than undersized,
    avoids that lip while still breaking the coincidence.
  - The **axial** coincidence (a helix cut flush to the real end faces
    starts its own flat end cap before the thread profile has completed
    even a quarter turn) is a dimensional defect, not a mesh-topology one
    — the result is a flat UNTHREADED band at both mouths of the nut, and
    Merge by Distance has nothing to weld there since nothing is
    duplicated, just wrong-shaped. This needs axial overshoot instead, so
    the helix is already wound up into a repeating cycle before reaching
    the real boundary. Critically, the overshoot amount is **half** a
    pitch, not a full one: a full-pitch shift looks like more margin but
    actually lands the exact same profile phase (the crest's return to
    `minor_r`) at `z=0` that would be there anyway — exactly coincident
    with the pilot bore's own wall over an extended stretch, which produced
    a genuine TORN HOLE in one configuration tested (real non-manifold
    boundary edges from a missing face) that Merge by Distance cannot weld
    shut, since there's no duplicate geometry, just an absent one. Half a
    pitch lands a different, non-`minor_r` phase at `z=0` instead, avoiding
    that extra coincidence while still giving the helix enough room to
    fully engage.

  The cut ordering and the `+ BOOL_EPSILON` pilot-bore sizing came from
  direct iteration in Blender by the maintainer, not from this repo's own
  headless verification suite — trust the working combination in the code
  over re-deriving it from this explanation if they ever disagree. See
  [hex_nut.md](hex_nut.md#build-method--two-different-coincidence-problems-two-different-fixes)
  for the verified results (five thread sizes, M4–M12). If you're adding a
  new subtractive-thread generator: match the *category* of fix to the
  *category* of coincidence (load-bearing dimension → exact + merge;
  wind-up/phase → overshoot, and check whether your specific overshoot
  amount reproduces the boundary condition it was meant to avoid) rather
  than copying one pattern wholesale.
- **Chained revolves, not stacked independently-capped primitives.**
  `hex_bolt.py`'s `_add_chained_revolve` builds the shank→thread-blank→tip
  stack as one continuous solid sharing vertices at each junction, rather
  than three separately-capped cylinders touching end to end. Two
  independently-capped segments meeting at the same radius would produce
  duplicate coincident faces (each segment's own cap covering the
  identical disk) — a fragile degenerate case for the EXACT solver.
  Chaining through shared vertices means there's no internal face for
  this to happen to. The tip segment specifically is a flat perpendicular
  step down to `tip_r` at the thread's own end Z, then a straight
  non-tapered pin — not a cone, which would shrink the blank's diameter
  while the constant-diameter thread cutter is still cutting through it
  and cause the same hard-stop dead-end the cutter overshoot above fixes.
- **Blocky joins last, cut-and-union sequencing matters.** `hex_bolt.py`
  cuts the thread out of the blank before unioning the hex head on,
  specifically because a flush blocky join (hex prism onto a cylinder end)
  is far friendlier to the EXACT solver than the thread cut — sequencing
  the riskier operation earlier means any failure surfaces before the
  cheap, reliable step, not after.
- **Re-normalize after union.** `hex_bolt.py`'s `_bool_union` calls
  `recalc_face_normals` on the result after applying the modifier and
  deleting the operand, because EXACT-solver unions of complex meshes can
  come back with inconsistent face winding even when topologically solid.
  `_bool_diff` (used for all thread cuts in both files) hasn't needed this
  in practice, but if you see winding issues after a difference, this is
  the first thing to try.

## Validation: draw() warnings are often looser than the execute() gate

`threaded_fastener.py`'s draw() shows ERROR-icon labels for degenerate
truncation/flank-angle combinations, but **nothing in its execute() ever
cancels** — the raw thread-helix operator always builds something, even
if the panel is warning you it looks wrong. `hex_bolt.py` and
`hex_nut.py`, by contrast, both **do** enforce a hard gate in execute()
(`self.report({'ERROR'}, ...)` + `CANCELLED`) — but the threshold used
there is looser than the one that triggers the draw() warning. E.g.
`hex_bolt.py` shows an ERROR-icon "Head too small for thread diameter"
warning once `head_wall <= 0.5mm`, but only actually cancels execution at
`head_wall <= 0`. A wall between 0 and 0.5mm will build successfully
despite the visible warning — read the number, not just the icon color.
