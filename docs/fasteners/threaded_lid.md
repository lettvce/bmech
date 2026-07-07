# Threaded Lid

`fasteners/threaded_lid.py` → `OBJECT_OT_threaded_lid` (`object.threaded_lid`, "Threaded Lid")

An inverted cup that screws onto a [threaded_container](threaded_container.md)
neck: solid cap at the top, open skirt below, with an **internal** thread
ADDED (unioned) onto the skirt's already-hollow inner wall, anchored at
the ceiling (plus a small axial gap) and extending DOWN for
`thread_length_mm`, above an unthreaded **guide zone** (down to the real
mouth) that lets the container's neck pass through freely before engaging
the thread. `thread_diameter_mm` plays double duty as both "OD of the
container this lid fits" and the thread's own major diameter, matching
hex_bolt.py/hex_nut.py's convention so `fastener_matching.sync_thread_dims`
works unmodified.

**[REDESIGN] Every earlier version of this file cut the thread
SUBTRACTIVELY out of a pre-built solid plug**, mirroring hex_nut.py's
recipe — which required the thread's own region of the skirt to be built
solid first, a separate pilot bore, and a careful overshoot/clip dance at
both ends of the cut. This version instead builds the WHOLE skirt hollow
from the start and ADDS the thread as a standalone helical ridge, unioned
onto the wall. See [Why additive, and the tradeoff that comes with it](#why-additive-and-the-tradeoff-that-comes-with-it)
below for why this is a deliberate choice, not an oversight.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | fastener objects with the OPPOSITE (external) orientation | Match Target; runs `fastener_matching.sync_thread_dims` |
| `thread_diameter_mm` | Float (mm) | 60.0 | 5–300 (soft) | OD of the container this lid fits, AND thread major diameter |
| `wall_thickness_mm` | Float (mm) | 3.0 | 0.4–20 (soft) | Skirt wall AND cap thickness |
| `thread_length_mm` | Float (mm) | 8.0 | 1–90 (soft), clamped up further — see Validation | How far DOWN from the ceiling the thread extends |
| `guide_length_mm` | Float (mm) | 4.0 | 1–90 (soft) | Unthreaded lead-in below the thread, down to the real mouth — lets the container's neck self-align before engaging the thread |
| `pitch_mm` | Float (mm) | 4.0 | 0.5–20 (soft) | Jar/bottle threads run much coarser than fastener threads |
| `flank_angle_deg` | Float (°) | 90.0 | 1–179 | Wider than the 60° fastener convention |
| `truncation` | Float | 0.25 | 0–0.3 | See [threaded_container.md](threaded_container.md#why-the-default-truncation-is-higher-than-the-fastener-convention) for why this is higher than the fastener default |
| `inner_compensation_mm` | Float (mm) | 0.0 | 0–0.5 (soft) | FDM: internal features print tight — added to thread major radius |
| `fit_offset_mm` | Float (mm) | 0.0 | 0–0.5 (soft) | FDM: added to thread diameter for a looser fit against a mating [threaded_container](threaded_container.md), whose own diameter is reduced by the same offset. Never synced by Match Target |
| `resolution` | Int | 64 | 8–256 (soft) | |

`height_mm` is **not a property** — it's a pure derived sum
(`wall_thickness_mm + thread_length_mm + guide_length_mm`), shown
read-only in the panel's info box. There is no longer a fixed "total
height" budget the other three have to fit inside; adding cap, thread, or
guide length simply grows the part.

`thread_diameter_mm`/`pitch_mm`/`flank_angle_deg`/`truncation` freeze
together in a driven column whenever a target is set, same all-four
freeze behavior as [hex_bolt.md](hex_bolt.md#match-target).
`wall_thickness_mm`/`thread_length_mm`/`guide_length_mm`/`resolution`/
`inner_compensation_mm`/`fit_offset_mm` are never driven — fit is this
part's own printing-tolerance choice, not something the mating container
could meaningfully specify.

`fit_offset_mm` folds into `major_r` (`_derived()` and `_clamp()` both add
`fit_offset_mm / 2.0` alongside `inner_compensation_mm`) before any of the
usual truncation/thread-length clamping runs, so a large fit offset is
subject to the same margin protections as a large `inner_compensation_mm`
— it can't push `minor_r` to zero or below any more than compensation can.

**The target picker only offers `EXTERNAL`-oriented objects** —
`hex_bolt`, `threaded_container`, or a `threaded_fastener.py` raw thread
currently built with `thread_type='EXTERNAL'`. See
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule).

## Why additive, and the tradeoff that comes with it

`hex_bolt.py`'s own docstring records that this codebase tried additive
(union) threading once before and abandoned it: unioning a separately-built
helical ridge onto an undersized core is a numerically harder case for the
`EXACT` boolean solver than differencing a cutter out of solid material,
because a plain difference against a solid blank always has genuine
material to remove, while a union of two independently-built,
nearly-coincident curved surfaces can read as a hairline touch and cancel
material instead of merging. See
[blender_extension_lessons.txt](../../blender_extension_lessons.txt)'s
"EXACT boolean solver" section for the general form of this lesson.

This file uses additive anyway, as a deliberate choice: the earlier
subtractive design's real complexity wasn't the cut direction itself, it
was the overshoot/clip dance needed because the cutter had to terminate
*flush with other pre-existing boundaries* (the ceiling above, the
clearance-zone wall below) without leaving a visible seam. An additive
ridge has no such boundary to land on — it only needs to touch the
surrounding wall at all, and is allowed to hard-start/hard-stop in open
bore air at both ends (see the module docstring's `[REDESIGN]` note). The
one piece of the old numerical-fragility problem that *does* still apply
— the ridge's root radius touching the wall at exactly `major_r` being a
hairline coincidence, not a genuine volumetric overlap — is handled the
same way `hex_bolt.py`'s old additive mode handled it: a depth-scaled
radial `overlap` pushes the ridge's root past `major_r`, so the union has
real volume to merge.

Swept empirically (972 configurations across thread diameter, wall
thickness, thread length, guide length, pitch, flank angle, and
truncation) with 0 crashes and 0 mesh defects (non-manifold edges,
zero-area faces, zero volume) after the fixes below — including the
`crest_flat`-vs-`root_flat` fix, which a manifold-only sweep like this one
cannot catch by itself (a mesh with the crest/root duty cycle inverted is
still perfectly manifold; see the profile-argument fix below, verified
instead via ray-casting and real thread-to-thread engagement).

## Build method

Base profile (2D, r vs z, revolved 360° around Z) — z=0 is the **ceiling**
(top of the physical lid), z=`height_mm` is the real **mouth** (the open
end the container's neck enters through), the reverse of this file's own
previous z convention, adopted so `height_mm` falls out as a pure sum of
its three parts without needing a separate "skirt depth minus cap"
subtraction anywhere:

1. `(0, 0)` pole — center of the solid cap's top exterior surface
2. → `(outer_r, 0)` — flat exterior top surface, pole to ring
3. → `(outer_r, height)` — down the full outer wall
4. → `(major_r, height)` — across the mouth's own annular rim (`outer_r`
   to `major_r`), the real opening the neck enters through
5. → `(major_r, wall_thickness_mm)` — up the already-hollow, unthreaded-
   in-the-base-mesh bore wall from the mouth rim to the ceiling's
   underside
6. → `(0, wall_thickness_mm)` pole — flat interior ceiling disc, closing
   the bottom of the solid cap
7. → `closed=True` back to point 1: both ends are poles on the same
   central axis, so no face is built there at all — the cap (z=0 to
   `wall_thickness_mm`, r=0 to `outer_r`) is one uniformly solid region.

This produces a solid cap sitting above a fully hollow skirt — there is
no separate "solid plug" region and no pre-hollowed pilot bore distinction
anymore, since the thread is added onto this one uniform wall rather than
cut out of a thicker one. Six checkpoints total, down from eight in the
previous subtractive design.

**The additive thread ridge**: a standalone helical solid built with
`_external_profile` (crest pointing outward — here that's the ridge's
ATTACHMENT radius, not a cutter's reach), `root_flat` swapped into the
`crest_flat` slot (same convention as every other thread cutter in this
family), root radius `major_r + overlap`, crest (tip) radius `minor_r`.
Built directly at `thread_length_mm` tall with no axial overshoot —
additive material terminating in open bore air needs no overshoot/clip
treatment, unlike a subtractive cutter that has to avoid leaving a seam in
real material. Positioned starting at `wall_thickness_mm + ceiling_eps`
(a small pitch-scaled gap below the interior ceiling disc, avoiding an
exact-coincidence hairline touch with that pre-existing face), then
unioned onto the base object.

`overlap = max(0.02, min(0.2 * depth, 0.15))` — the same depth-scaled
radial padding `hex_bolt.py`'s own (since-abandoned) additive mode used,
so the ridge's root has genuine volumetric overlap with the wall instead
of a hairline touch at exactly `major_r`.

**[BUG, fixed] `crest_flat` goes in `_external_profile`'s argument slot
here — not `root_flat`, despite every subtractive cutter in this family
passing `root_flat` into that exact slot.** `_build_helix` fills its
cross-section from a constant floor (here, `minor_r`) up to a ceiling
that peaks at `major_r` for the argument's own plateau length. For a
cutter subtracted from a solid plug, passing `root_flat` there makes the
*removed* (valley) region span `root_flat + 2*flank_dz` — correct, since
the valley should be the wide one — leaving a properly tapered tooth
behind as the complement. But this file doesn't cut a complement out of
solid material; it unions this exact swept shape directly as the ridge.
Passing `root_flat` there built a solid that's fully deployed (reaching
`minor_r`) across `root_flat`'s own span and absent across `crest_flat`'s
span — precisely inverted from a real ridge, which must be fully deployed
only across `crest_flat`'s short span and retract flush with the wall
(no protrusion at all) across `root_flat`'s long span. Confirmed two ways:
(1) ray-casting radially outward from the bore's own axis at a fixed
height across 360 angles showed the `root_flat` version blocking 75% of
the sweep (matching `2*flank_dz + root_flat`) and open for only 25%
(matching `crest_flat`) — backwards; swapping to `crest_flat` restored
the expected 50/50 split. (2) Building a matching `threaded_container`
and boolean-intersecting it with the lid at a fine sweep of relative
axial/angular phases: the `root_flat` version never dropped below ~5% of
its own peak interference volume at any phase (never truly "meshed"); the
`crest_flat` version reaches **exactly 0.0** interference at the correctly
threaded phase and rises smoothly away from it in both directions — the
signature of a real, functioning thread pair, not just a manifold mesh.

`ceiling_eps = max(0.05, 0.05 * pitch_mm)` — small, pitch-scaled axial gap
between the ridge's own start and the interior ceiling disc.

**Recalc face normals unconditionally after the union** — per
`blender_extension_lessons.txt`, EXACT-solver unions can come back with
inconsistent face winding even on a genuinely manifold/solid result;
`_bool_op` does this every time regardless of whether a defect is
suspected.

**[BUG, fixed] `_build_helix` crashes outright (not just a mesh defect) if
`thread_length_mm` is too small relative to the thread's own profile
span.** `_build_helix` computes
`steps = ceil((height - profile_span) * res / pitch) + 1`, where
`profile_span = 2*fdz + crest_flat` is the axial height one full
crest-to-crest repeat occupies. Once `thread_length_mm <= profile_span`
this goes to zero or negative rings and `_build_helix` throws
`IndexError` building `rings[0]` — confirmed empirically (e.g.
`thread_length_mm=4` with `pitch_mm=8` crashes every time, well within
both properties' own valid ranges). Fixed in `_clamp()` (see below) by
clamping `thread_length_mm` up to `profile_span + 0.5 * pitch_mm`
whenever it's smaller — a half-pitch margin comfortably guarantees
several real ring steps.

## Validation: clamped, not cancelled

`execute()` calls `self._clamp()` first — same
[clamp-over-cancel rule](../CONVENTIONS.md#conventions-that-apply-across-every-family-not-just-gears)
as [threaded_container.md](threaded_container.md#validation-clamped-not-cancelled).
Only two silent-correction steps remain (down from three in every earlier
version) plus the crash-prevention floor above — since `height_mm` is now
a pure derived sum rather than a fixed budget the other properties have to
fit inside, there is no longer any "wall_thickness_mm vs. total height"
conflict to reconcile at all:

1. `truncation` clamped UP (via `_min_truncation_for_max_depth`) whenever
   the resulting thread depth would push `minor_r` below a
   `max(1.0, major_r * 0.05)` margin. Depends only on `major_r` — wall
   thickness plays no part, same as every earlier version.
2. `guide_length_mm` clamped up to `max(1.0, 0.25 * pitch_mm)` — the same
   margin the earlier subtractive design used for its clearance zone.
   The original numerical mechanism (a razor-thin exact-coincidence wall)
   doesn't apply the same way to an additive ridge, but the margin is
   kept anyway as the only guard against a guide zone too thin for the
   container's neck to actually self-align in.
3. `thread_length_mm` clamped up to `(2*fdz + crest_flat) + 0.5 * pitch_mm`
   whenever smaller — see the `[BUG, fixed]` note above; this is a crash
   fix, not a quality-only floor.

**One case still cancels**: if `truncation` would need to exceed 0.3 to
keep `minor_r` positive with margin (a pitch too coarse for the thread
diameter at any truncation) — no meaningful single "closest valid value"
to fall back to.

`draw()` shows these as `INFO`-icon labels at the bottom of the panel
(purely defensive/informational post-clamp), except the thread-length
warning, which is a genuine, non-clamped, separate concern from the crash
floor above:

- `fdz <= 0` → "Truncation too high — no room for flanks at this pitch" (can't actually occur, kept defensively).
- `minor_r <= 0` → "Thread too deep for this diameter — minor Ø ≤ 0".
- `thread_length_mm < 1.5 * pitch_mm` (and not the above) → **"Thread length < 1.5 pitches — weak engagement..."** (`ERROR` icon, still non-blocking — confirmed in the earlier subtractive design that exactly 1.0 pitch is a mesh defect there; kept here as a weak-engagement heads-up independent of the crash-prevention clamp, which uses a different threshold based on the profile's own geometry rather than a flat pitch multiple).

## Output

One object, named `ThreadedLid`, stamped via
`fastener_matching.stamp_thread`: `bmech_kind="threaded_lid"`,
`bmech_thread_diameter`, `bmech_pitch`, `bmech_flank_angle_deg`,
`bmech_truncation`. `fastener_matching.fastener_orientation` maps this
kind to `INTERNAL`. Meshes with a
[threaded_container](threaded_container.md) of the same four thread
values.
