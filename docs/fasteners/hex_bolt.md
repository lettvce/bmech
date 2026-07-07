# Hex Bolt

`fasteners/hex_bolt.py` → `OBJECT_OT_hex_bolt` (`object.hex_bolt`, "Hex Bolt")

A complete bolt — hex head, optional shank, external thread, optional
tip. Built **subtractively**: a blank stack (shank → thread blank at the
full major diameter → tip) is cut with a helical groove cutter, and the
hex head is unioned on last. Reuses `threaded_fastener.py`'s thread math
(`_thread_params`, `_build_helix`, `_internal_profile` as the cutter)
**verbatim by duplication**, not by import — see
[README.md](README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate).

## Axial layout (Z, head at z=0)

```
hex head    [0, hex_length_mm]
shank       [hex_top, hex_top + shank_length_mm]              (if shank_enable)
thread      [shank_top, shank_top + thread_length_mm]
tip         [thread_top, thread_top + tip_length_mm]          (if tip_enable)
```

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | mesh objects with `bmech_thread_diameter` | Match Target; runs `fastener_matching.sync_thread_dims` — copies `thread_diameter_mm`/`pitch_mm`/`flank_angle_deg`/`truncation` all at once |
| `hex_length_mm` | Float (mm) | 5.5 | 0.5–30 (soft) | Head height |
| `hex_across_flats_mm` | Float (mm) | 13.0 | 1–100 (soft) | |
| `shank_enable` | Bool | True | | |
| `shank_length_mm` | Float (mm) | 10.0 | 0.1–200 (soft) | |
| `shank_diameter_mm` | Float (mm) | 8.0 | 0.1–80 (soft) | Usually equal to thread Ø |
| `thread_length_mm` | Float (mm) | 20.0 | 0.5–200 (soft) | |
| `thread_diameter_mm` | Float (mm) | 8.0 | 0.5–80 (soft) | Nominal **major** diameter |
| `pitch_mm` | Float (mm) | 1.25 | 0.1–10 (soft) | |
| `flank_angle_deg` | Float (°) | 60.0 | 1–179 | 60° metric/UNC, 55° BSP, 29° ACME |
| `truncation` | Float | 0.125 | 0.0–0.3 | |
| `resolution` | Int | 32 | 8–128 (soft) | |
| `outer_compensation_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to thread major radius — printed external features shrink |
| `fit_offset_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Subtracted from thread diameter — a looser fit against a mating internal thread (e.g. [hex_nut](hex_nut.md)) whose own diameter is increased by the same offset. Never synced by Match Target |
| `tip_enable` | Bool | True | | |
| `tip_length_mm` | Float (mm) | 3.0 | 0.1–30 (soft) | |
| `tip_diameter_mm` | Float (mm) | 0.0 | 0.0–80 (soft) | 0 = sharp point, >0 = flat dog-point tip. Silently clamped to just under the thread's minor Ø (see below) |

## Match Target

Picking a target freezes `thread_diameter_mm`, `pitch_mm`, `flank_angle_deg`,
and `truncation` together, all at once, in a driven column in `draw()` —
unlike the gear family, there's no partial-match case here (a spur-target
that only drives module/pressure-angle while leaving helix/hand editable,
say): a bolt and the nut that fits it need **all four** thread dimensions
to match exactly for the threads to physically engage, so all four freeze
or unfreeze together depending on whether any valid target is set.
`thread_length_mm`, `resolution`, `outer_compensation_mm`, and
`fit_offset_mm` are never driven — length/resolution/compensation/fit are
this part's own choices, not something the mating fastener could
meaningfully specify.

**Unlike the gear family, the poll here is deliberately NOT loose.** A
bolt (always `EXTERNAL` orientation) can only target `INTERNAL`-oriented
objects — `hex_nut` objects, or a `threaded_fastener.py` raw thread
currently built with `thread_type='INTERNAL'` — never another `hex_bolt`
or an `EXTERNAL` raw thread, since there's no physical case where two
external threads mate (unlike some cross-kind gear matches, which can be
mechanically legitimate). See
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for the full reasoning and the mechanism (`bpy.context.active_operator`
lookup inside the poll) this restriction depends on — and why that
mechanism can't be verified by a headless test. `press_fit_pin.py` uses a
separate, unrelated `bmech_kind`-based system for its own face-alignment
operator, not this one.

## Build method

1. **Blank stack** (`_add_chained_revolve`): shank → thread blank
   (cylinder at the full `major_r`, not an undersized core) → tip built
   as one continuous solid of revolution sharing vertices at every
   junction, not three independently-capped primitives touching end to
   end. Independently-capped segments meeting at the same radius would
   leave duplicate coincident faces (both segments' own caps covering the
   identical disk) — a fragile case for the `EXACT` boolean solver.
   Chaining through shared vertices means the case can't arise.
   `tip_diameter_mm = 0` collapses the tip's final ring to a single shared
   pole vertex (a true point) rather than a zero-radius ring.
   The tip segment is a **flat perpendicular step** down to `tip_r` at the
   thread's own end Z, then a straight non-tapered pin — not a cone. A
   cone would shrink the blank's diameter while the thread cutter (next
   step) is still cutting a constant-diameter helix through it, so the
   thread would run out against a shrinking target and dead-end right at
   the tip instead of stopping cleanly.
2. **Thread cut**: a helical groove cutter (`_internal_profile` +
   `_build_helix`, with `root_flat` passed into the profile's
   `crest_flat` slot — see the module docstring) is differenced out of
   the thread-blank segment. The cutter's crest reach pokes slightly past
   `major_r` (`overlap = max(0.02, min(0.2*depth, 0.15))`) so the
   difference has genuine volume to remove instead of a hairline touch
   the `EXACT` solver could read as a no-op, and it runs one full pitch
   past `thread_length_mm` so the last crest doesn't dead-end flush at the
   blank's flat step — see
   [README.md](README.md#boolean-solver-patterns-specific-to-this-family).
   `tip_diameter_mm` is clamped here too: silently reduced to just under
   the thread's minor Ø (mutating the property itself, like gear
   `pressure_angle_deg` clamping) so the tip always works as an
   unthreaded lead-in pin instead of building too fat to matter.
3. **Hex head** (`_add_hex_prism`) built separately and unioned on
   **last** — a flush blocky join is far friendlier to the `EXACT` solver
   than the thread cut, so the riskier step happens first and any failure
   surfaces before the cheap reliable one. `_bool_union` re-runs
   `recalc_face_normals` afterward, since EXACT-solver unions can come
   back with inconsistent winding even when topologically solid.

## Panel warnings

- `flank_dz <= 0` → **"Truncation too high — no room for flanks at this
  pitch"** (ERROR icon).
- `head_wall <= 0.5` (where `head_wall = across_flats/2 - thread_diameter/2`)
  → **"Head too small for thread diameter"** (ERROR icon).

**Unlike the raw threaded fastener, these ARE enforced** — `execute()`
re-checks `flank_dz <= 0 or head_wall <= 0` and, if true, reports
`self.report({'ERROR'}, "Invalid geometry — check truncation and head
size vs thread diameter")` and returns `CANCELLED`. Note the execute()
gate (`head_wall <= 0`) is looser than the draw() warning threshold
(`head_wall <= 0.5`) — a head wall between 0 and 0.5mm shows a visible
warning but still builds successfully.

## Output

One object, renamed `HexBolt`, stamped via `fastener_matching.stamp_thread`:
`bmech_kind="hex_bolt"`, `bmech_thread_diameter`, `bmech_pitch`,
`bmech_flank_angle_deg`, `bmech_truncation`. These properties are what a
`hex_nut` (or a future `hex_bolt` targeting this one) reads back via
Match Target — see the section above.
