# Raw Threaded Fastener

`fasteners/threaded_fastener.py` → `OBJECT_OT_add_threaded_fastener` (`object.add_threaded_fastener`, "Add Threaded Fastener")

Generates a single thread-helix solid, meant to be combined with other
geometry via a **manual** boolean in the same session — this operator
never does the union/difference itself. It's the primitive both
[hex_bolt.md](hex_bolt.md) and [hex_nut.md](hex_nut.md) build on (by
duplicating its math, not importing it — see
[README.md](README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate)).

## The four mode combinations

| `thread_type` | `operation` | Meaning | Output name |
|---|---|---|---|
| EXTERNAL | ADDITIVE | Bolt-type ridge, union onto a shaft | `ExternalThread` |
| EXTERNAL | SUBTRACTIVE | Cut external threads into a cylinder | `ExternalThreadCutter` |
| INTERNAL | ADDITIVE | Nut-type ridge, union into a tube bore | `InternalThread` |
| INTERNAL | SUBTRACTIVE | Tap a hole (difference from a block) | `TapCutter` |

All four share identical helix-sweep math — only which compensation field
applies and the output name differ. The profile shape is picked by a
compact XNOR: `if (thread_type == 'EXTERNAL') == (operation == 'ADDITIVE')`
use the external (ridge-out) profile, else the internal (ridge-in)
profile — i.e. External+Additive and Internal+Subtractive share a
profile, and External+Subtractive and Internal+Additive share the other.
That's because a cutter is always the *opposite* profile of its
same-named additive counterpart.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | fastener objects with the OPPOSITE orientation — see Match Target below | Match Target; runs `fastener_matching.sync_raw_thread` |
| `thread_type` | Enum | `EXTERNAL` | `EXTERNAL`/`INTERNAL` | Forced to the opposite of the target's orientation whenever a target is set — see below |
| `operation` | Enum | `ADDITIVE` | `ADDITIVE`/`SUBTRACTIVE` | |
| `diameter_mm` | Float (mm) | 8.0 | 0.5–100 (soft) | Nominal **major** diameter, crest-to-crest |
| `pitch_mm` | Float (mm) | 1.25 | 0.1–10 (soft) | Distance between thread crests |
| `flank_angle_deg` | Float (°) | 60.0 | 1–179 | 60° = metric/UNC, 55° = BSP, 29° = ACME |
| `truncation` | Float | 0.125 | 0.0–0.3 | Crest flat as a fraction of pitch (ISO metric = 1/8). Root flat is always 2× this. |
| `height_mm` | Float (mm) | 12.0 | 0.5–200 (soft) | Total thread length |
| `resolution` | Int | 32 | 8–128 (soft) | Steps per revolution |
| `outer_compensation_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to major diameter. Use for External+Additive (bolt) — printed external features shrink. |
| `inner_compensation_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to major diameter. Use for Subtractive or Internal+Additive — printed holes come out tighter than designed. |

`_derive()` applies whichever compensation field matches the active
mode — `outer_compensation_mm` only for External+Additive, else
`inner_compensation_mm` — and always adds it to `major_r` before
re-deriving `minor_r` from the (now-larger) major radius. See
[README.md](README.md#fdm-compensation-always-added-one-direction-per-field).

## Match Target

Unlike `hex_bolt.py`/`hex_nut.py` (each permanently one orientation), a
raw thread can be built as EITHER, so picking a target does two things at
once: copies `diameter_mm`/`pitch_mm`/`flank_angle_deg`/`truncation` (all
four, frozen together, same as the hex generators — see
[hex_bolt.md](hex_bolt.md#match-target)), **and** forces `thread_type` to
whichever orientation is opposite the target's own — an external target
needs this built as `INTERNAL`, an internal target needs `EXTERNAL`. Both
freeze in `draw()` when a target is set. `operation` (additive/
subtractive) is **never** frozen — it only controls how the resulting
external or internal thread gets built (union vs. difference), not
whether it fits, so it stays a free choice regardless of target.

**The target picker itself only offers objects of the OPPOSITE
orientation** — pick a target while `thread_type='EXTERNAL'` and the
dropdown only lists internal-kind objects (`hex_nut`, or another raw
thread currently built as `INTERNAL`), and vice versa. This is a
deliberate divergence from the gear family's loose-poll philosophy — see
[README.md](README.md#match-target-a-deliberate-exception-to-the-no-shared-module-rule)
for why an external/internal thread pairing has no legitimate same-
orientation case the way cross-kind gear matches sometimes do. Because
this operator's own orientation is a *live* property rather than a fixed
kind, the poll has to look up which operator is asking
(`bpy.context.active_operator`) to know which orientation to filter for —
this, like every other pick/reset/rebuild behavior in this pattern,
cannot be exercised by a headless test; verify manually in the GUI that
the dropdown actually excludes same-orientation objects before trusting
it, especially after editing `fastener_matching.fastener_target_poll`.

## Build method

`_build_helix(bm, profile, pitch, height, res)` sweeps the 4-point
trapezoidal thread profile (root → rising flank → crest → falling flank)
around Z. Step count is `ceil((height - profile_span) * res / pitch) + 1`
— rounded **up**, so the built helix can slightly overshoot the requested
`height_mm`. This operator does nothing to correct that overshoot itself
(unlike `hex_nut.py`, which clips it with an INTERSECT boolean) — if you
need an exact height, plan for a small margin or trim afterward.

Side walls are quads between consecutive swept rings, including a
closing root-flat quad; the first and last rings are capped (first
reversed) to seal the helix ends into a manifold solid.

## Panel warnings

- `flank_dz <= 0` (truncation leaves no room for flanks at this pitch) →
  **"Truncation too high — no room for flanks at this pitch"** (ERROR
  icon).
- `flank_angle_deg < 2.0` → **"Flank angle near zero — thread depth will
  be very large"** (ERROR icon).

**Neither of these blocks `execute()`.** This operator has no hard
validation gate at all — internal clamps (`max(flank_dz, 0.0)`,
`max(half_angle, radians(0.5))`) keep the math from crashing, but the
operator always builds *something*, even when the panel is telling you
the result looks wrong. Contrast with `hex_bolt.py`/`hex_nut.py`, which
both do enforce a hard gate on their versions of these checks.

## Output

One object per call, at the 3D cursor, stamped via
`fastener_matching.stamp_thread`: `bmech_kind` is `"external_thread"` or
`"internal_thread"` depending on `thread_type` alone — **not** on
`operation`. An `ExternalThreadCutter` (External+Subtractive) still
stamps `"external_thread"`, even though its own current mesh shape uses
the internal (ridge-in) profile as a cutter tool — the profile shape
describes how this specific object cuts material away, but the RESULT it
produces once you finish the boolean by hand is an external thread, which
is what other parts need to match against, not this object's current
shape. Same reasoning applies to `TapCutter` stamping `"internal_thread"`.
