# Threaded Container

`fasteners/threaded_container.py` → `OBJECT_OT_threaded_container` (`object.threaded_container`, "Threaded Container")

A screw-top jar body: solid floor, straight outer wall, open mouth at the
top, with an **external** thread cut directly into the existing outer
wall near the top — no separate raised neck boss. `thread_diameter_mm`
plays double duty as both the container's own OD and the thread's major
diameter, matching hex_bolt.py/hex_nut.py's naming convention so
`fastener_matching.sync_thread_dims` works unmodified. See
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for the family's Match Target system this participates in.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | fastener objects with the OPPOSITE (internal) orientation | Match Target; runs `fastener_matching.sync_thread_dims` |
| `thread_diameter_mm` | Float (mm) | 60.0 | 5–300 (soft) | Container OD AND thread major diameter |
| `wall_thickness_mm` | Float (mm) | 3.0 | 0.4–20 (soft) | Side wall AND floor thickness |
| `height_mm` | Float (mm) | 60.0 | 1–500 (soft) | Total outer height, floor to rim |
| `thread_length_mm` | Float (mm) | 10.0 | 1–100 (soft) | How far down from the rim the thread extends |
| `pitch_mm` | Float (mm) | 4.0 | 0.5–20 (soft) | Jar/bottle threads run much coarser than fastener threads |
| `flank_angle_deg` | Float (°) | 30.0 | 1–179 | Shallower buttress-style default than the 60° fastener convention |
| `truncation` | Float | 0.25 | 0–0.3 | Higher default than the 0.125 fastener convention — see Build method for why |
| `outer_compensation_mm` | Float (mm) | 0.0 | 0–0.5 (soft) | FDM: external features shrink — added to thread major radius |
| `fit_offset_mm` | Float (mm) | 0.0 | 0–0.5 (soft) | FDM: subtracted from thread diameter for a looser fit against a mating [threaded_lid](threaded_lid.md), whose own diameter is increased by the same offset. Never synced by Match Target |
| `resolution` | Int | 64 | 8–256 (soft) | |

`thread_diameter_mm`/`pitch_mm`/`flank_angle_deg`/`truncation` freeze
together in a driven column whenever a target is set — same all-four
freeze behavior as [hex_bolt.md](hex_bolt.md#match-target), since a
container and its mating lid need all four to match simultaneously.
`wall_thickness_mm`/`height_mm`/`thread_length_mm`/`resolution`/
`outer_compensation_mm`/`fit_offset_mm` are never driven — fit, like
compensation, is this part's own printing-tolerance choice, not something
a mating fastener could meaningfully specify.

`fit_offset_mm` folds into `outer_r` (`_derived()` and `_clamp()` both
subtract `fit_offset_mm / 2.0` alongside adding `outer_compensation_mm`)
before any of the usual wall-thickness/truncation/thread-length clamping
runs, so a large fit offset is subject to the same margin protections a
large negative OD change would be.

**The target picker only offers `INTERNAL`-oriented objects** —
`hex_nut`, `threaded_lid`, or a `threaded_fastener.py` raw thread
currently built with `thread_type='INTERNAL'`. See
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for why this family's poll is deliberately NOT loose the way the gear
family's is.

## Why the default truncation is higher than the fastener convention

Jar/bottle threads use a much coarser pitch than fastener threads (4mm
here vs. ~1.25mm for hex_bolt.py's default), and thread depth scales with
`pitch` for a given flank angle and truncation
(`depth = flank_dz / tan(half_angle)`, and `flank_dz` scales with pitch).
At the fastener family's usual `truncation=0.125`, a 4mm pitch at 30°
flank angle produces a thread over 4.6mm deep — deeper than any
reasonable container wall. `truncation=0.25` cuts that to about 1.9mm,
safely under the default 3mm wall. If you increase `pitch_mm`, raise
`truncation` too (up to its 0.3 maximum) or increase `wall_thickness_mm`
to keep `depth < wall_thickness_mm` — the panel's ERROR label
("Thread too deep for this wall — would break through to interior")
fires at `depth >= wall_thickness_mm`, with a warning starting at 70% of
that.

## Build method

Base body: the same "C"-shaped chained-revolve technique
`threaded_lid.py` uses (see that file's module docstring for the shared
`_add_chained_revolve` helper, including its `closed`/`cap_start`/
`cap_end` flags) — outer-bottom pole, up the outer wall, across the open
top rim, down the inner wall, inner-floor pole. Both ends are poles, so
no boolean is needed for the basic hollow shape.

Thread cut: identical technique to `hex_bolt.py`'s external thread — a
helical groove cutter (`_internal_profile`, crest pointing inward, used
subtractively), `root_flat` swapped into the `crest_flat` argument slot,
a depth-scaled radial `overlap` so the cut has genuine volume to remove
rather than a hairline touch at the wall's own true major radius, cut
into the EXISTING solid wall near the top — no base-shape rework needed,
unlike [threaded_lid.md](threaded_lid.md), because the container's outer
wall is already solid at exactly the thread's major radius.

**Cleanup pass required, unlike hex_bolt.py.** This cut leaves a small,
fixed number of zero-area sliver faces (two numerically-identical
vertices within one triangle) from the EXACT solver — confirmed
empirically to be independent of the cutter's axial overshoot amount
(tested 0 to 2 full pitches, defect count never changed) and independent
of exactly where the top boundary sits, ruling out a wind-up/coincidence
theory; only mesh resolution changed the count. `bmesh.ops.
dissolve_degenerate` (the dedicated tool for exactly this artifact class)
run once after the boolean removes them completely — verified 0
zero-area, 0 non-manifold, correct positive volume afterward across a
54-combination parameter sweep.

## Validation: clamped, not cancelled

`execute()` calls `self._clamp()` first, before any geometry is built —
following [CONVENTIONS.md's clamp-over-cancel
rule](../CONVENTIONS.md#conventions-that-apply-across-every-family-not-just-gears),
the same pattern `gear_matching.clamp_pressure_angle` and `hex_bolt.py`'s
own `tip_diameter_mm` clamp use. Rather than cancelling when a
combination is invalid, three properties get silently corrected in
order:

1. `wall_thickness_mm` is clamped down to `min(outer_r, height_mm) - 0.1`
   whenever it would leave no interior cavity.
2. `truncation` is clamped UP (via `_min_truncation_for_max_depth`,
   solving the closed-form relationship between thread depth and
   `pitch_mm`/`flank_angle_deg`/`truncation`) whenever the resulting
   thread depth would exceed 70% of the (now-valid) wall thickness.
   Truncation, not wall thickness or thread diameter, is what gets
   adjusted here — it specifically controls how much a thread's own
   crest/root flats blunt its depth, while the size properties are what
   the user actually came to set.
3. `thread_length_mm` is clamped down to fit within `height_mm - floor_z`.

**Only one case still cancels**: if `truncation` would need to exceed its
own max (0.3) to keep depth in check — a pitch too coarse for the wall
thickness at any truncation — `_clamp()` returns a message,
`self.report({'ERROR'}, ...)` fires, and `execute()` returns `CANCELLED`.
This is the one combination with no meaningful single "closest valid
value" to fall back to (matching `bevel_gear.py`'s own
`face_width_mm >= cone_dist` precedent for the same kind of genuinely
irreconcilable case).

`draw()` still shows the same conditions as `INFO`-icon labels at the
bottom of the panel, after the derived-values box — but they're now
purely defensive/informational, since `_clamp()` means they should rarely
or never actually fire once a build completes:

- `inner_r <= 0` or `floor_z >= height_mm` → "Wall too thick for this OD/height — no interior cavity".
- `fdz <= 0` → "Truncation too high — no room for flanks at this pitch" (can't actually occur — `truncation`'s own max of 0.3 keeps `fdz > 0` always; kept as a defensive display, costs nothing).
- `depth >= wall_thickness_mm * 0.7` → "Thread depth near wall thickness limit".
- `thread_length_mm > (height_mm - floor_z)` → "Thread length near container height limit".

## Output

One object, named `ThreadedContainer`, stamped via
`fastener_matching.stamp_thread`: `bmech_kind="threaded_container"`,
`bmech_thread_diameter`, `bmech_pitch`, `bmech_flank_angle_deg`,
`bmech_truncation`. `fastener_matching.fastener_orientation` maps this
kind to `EXTERNAL`. Meshes with a [threaded_lid](threaded_lid.md) of the
same four thread values.
