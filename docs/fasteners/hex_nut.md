# Hex Nut

`fasteners/hex_nut.py` → `OBJECT_OT_hex_nut` (`object.hex_nut`, "Hex Nut")

A hex thru-nut — internal thread cut all the way through a hex prism.
Built **subtractively**: a helical groove cutter differenced out of the
prism first, then a plain pilot bore differenced out second, then welded
clean with a Merge by Distance pass — no separate thread ridge is ever
unioned in. Reuses `threaded_fastener.py`'s thread math
(`_thread_params`, `_build_helix`, `_external_profile` as the cutter) by
duplication, same convention as [hex_bolt.md](hex_bolt.md) — see
[README.md](README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate).
This file contains the clearest statement in the codebase of *why* that
duplication exists:

> "Thread math is duplicated from threaded_fastener.py per this project's
> convention (each generator module is self-contained, no cross-file
> thread math imports)."

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | mesh objects with `bmech_thread_diameter` | Match Target; runs `fastener_matching.sync_thread_dims` — copies `thread_diameter_mm`/`pitch_mm`/`flank_angle_deg`/`truncation` all at once |
| `z_height_mm` | Float (mm) | 6.5 | 0.5–50 (soft) | |
| `across_flats_mm` | Float (mm) | 13.0 | 1–100 (soft) | |
| `thread_diameter_mm` | Float (mm) | 8.0 | 0.5–80 (soft) | Nominal **major** diameter of the internal thread — "the hole size" |
| `pitch_mm` | Float (mm) | 1.25 | 0.1–10 (soft) | |
| `flank_angle_deg` | Float (°) | 60.0 | 1–179 | 60° metric/UNC, 55° BSP, 29° ACME |
| `truncation` | Float | 0.125 | 0.0–0.3 | |
| `resolution` | Int | 32 | 8–128 (soft) | |
| `inner_compensation_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to thread major radius — printed holes come out tight |
| `fit_offset_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to thread diameter — a looser fit against a mating external thread whose own diameter is reduced by the same offset. Never synced by Match Target |

## Match Target

Same freeze behavior as [hex_bolt.md](hex_bolt.md#match-target):
`thread_diameter_mm`, `pitch_mm`, `flank_angle_deg`, and `truncation`
freeze together, all at once, whenever a valid target is set — a nut and
the bolt it fits need all four thread dimensions to match exactly, so
there's no partial-freeze case the way there is for gears. `z_height_mm`,
`across_flats_mm`, `resolution`, `inner_compensation_mm`, and
`fit_offset_mm` stay editable regardless — none of them are part of what
makes a nut fit a given bolt.

**The target picker only offers `EXTERNAL`-oriented objects** — a nut
(always `INTERNAL`) can target a `hex_bolt`, or a `threaded_fastener.py`
raw thread currently built with `thread_type='EXTERNAL'`, but never
another `hex_nut` or an `INTERNAL` raw thread. See
[hex_bolt.md](hex_bolt.md#match-target) and
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for why this diverges from the gear family's deliberately loose poll.

See [README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for why the Match Target machinery lives in a small shared
`fastener_matching.py` despite this family's own "no shared module" rule
for thread geometry math.

## Build method — two different coincidence problems, two different fixes

This construction produces two DIFFERENT kinds of coincident surface, and
conflating them — fixing both the same way — reintroduces whichever bug
that one approach doesn't cover. They need to be understood separately:

1. **Difference the thread cutter FIRST** (`_external_profile` +
   `_build_helix`, with `root_flat` passed into the profile's `crest_flat`
   slot — see the module docstring) out of the solid hex prism: **exactly**
   `major_r` radially (no outward padding), but **half a pitch** of
   overshoot past **both** `z=0` and `z=z_height_mm` axially.
2. **Cut a plain pilot bore SECOND** (after the thread cutter, not before)
   through the same prism, sized to `minor_r + BOOL_EPSILON` — a hairline
   OVER the true minor diameter, not exactly equal to it. Z is padded
   `±BOOL_EPSILON` past both ends of `z_height_mm` to guarantee a full
   through-cut.
3. **Merge by Distance** (`bmesh.ops.remove_doubles`, a small fixed
   tolerance) on the resulting mesh, which cleans up whatever coincident
   geometry remains from steps 1-2.

**Radial coincidence (`minor_r`):** the pilot bore's wall and the thread
cutter's own root-flat faces are both nominally at `minor_r`. An earlier
version of this file cut the pilot bore *first*, sized to *exactly*
`minor_r`, then differenced the thread cutter afterward — undersizing the
bore the way `hex_bolt.py`'s `overlap` pattern would (shrinking it further)
left a real, physical, uncut lip at the bore's mouth, narrower than the
true minor diameter, since undersizing the bore before the thread cut
leaves material the thread cutter doesn't reliably clear. Cutting the
thread first and the pilot bore second — sized a hairline *over* `minor_r`
rather than under it — avoids that lip while still breaking the exact
radial coincidence.

**Axial wind-up (`z=0` / `z=z_height_mm`):** a single-start helix cut
exactly flush to the real end faces starts its very first ring — also its
own flat end cap — before the thread profile has completed even a quarter
turn, leaving a flat UNTHREADED band at both mouths of the nut (every angle
reads a uniform `minor_r` there, not the oscillating crest/root pattern a
real thread has). This is a dimensional defect, not a topological one — it
can pass a min-radius-only check completely undetected, since the flat
band happens to sit exactly at `minor_r` too. It needs axial overshoot, so
the helix is already fully wound up by the time it reaches the real
boundary. The overshoot amount matters: a FULL pitch — the more obviously
"safe" choice — is actually the worst one, because it places the same
profile phase at `z=0` that would land there anyway (the crest's return to
`minor_r`), exactly coincident with the pilot bore's own wall over an
extended stretch, which produced a genuine TORN HOLE in one configuration
tested (real non-manifold boundary edges from a missing face — not
duplicate geometry, so Merge by Distance can't weld it shut). **Half** a
pitch lands a different, non-`minor_r` phase at `z=0` instead, avoiding
that extra coincidence while still providing wind-up room.

The cut ordering (thread first, bore second) and the `+ BOOL_EPSILON`
pilot-bore sizing were arrived at through direct iteration in Blender by
the maintainer, not independently re-verified in this repo's own headless
test suite before shipping — see
[README.md](README.md#boolean-solver-patterns-specific-to-this-family).

## Panel warnings

- `flank_dz <= 0` → **"Truncation too high — no room for flanks at this
  pitch"** (ERROR icon).
- `wall <= 0.5` (where `wall = across_flats/2 - major_r`) → **"Thin or
  negative wall — increase across flats or reduce thread Ø"** (ERROR
  icon).

**Both are enforced in execute()**, same pattern as `hex_bolt.py`:
`flank_dz <= 0 or wall <= 0` reports
`self.report({'ERROR'}, "Invalid geometry — check truncation and
across-flats vs thread diameter")` and cancels. Again the execute() gate
(`wall <= 0`) is looser than the draw() warning (`wall <= 0.5`) — a wall
between 0 and 0.5mm builds despite the visible warning.

## Output

One object, named `HexNut` directly (not a temp-prefixed intermediate
name), stamped via `fastener_matching.stamp_thread`: `bmech_kind="hex_nut"`,
`bmech_thread_diameter`, `bmech_pitch`, `bmech_flank_angle_deg`,
`bmech_truncation` — the same stamping `hex_bolt.py` uses, read back by a
`hex_bolt` (or another `hex_nut`) picking this one as its Match Target.
