# Hex Nut

`fasteners/hex_nut.py` → `OBJECT_OT_hex_nut` (`object.hex_nut`, "Hex Nut")

A hex thru-nut — internal thread cut all the way through a hex prism.
Reuses `threaded_fastener.py`'s Internal+Additive thread math by
duplication, same as [hex_bolt.md](hex_bolt.md) does for External+
Additive — see
[README.md](README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate).
This file contains the clearest statement in the codebase of *why* that
duplication exists:

> "Thread math is duplicated from threaded_fastener.py per this project's
> convention (each generator module is self-contained, no cross-file
> thread math imports)."

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `z_height_mm` | Float (mm) | 6.5 | 0.5–50 (soft) | |
| `across_flats_mm` | Float (mm) | 13.0 | 1–100 (soft) | |
| `thread_diameter_mm` | Float (mm) | 8.0 | 0.5–80 (soft) | Nominal **major** diameter of the internal thread — "the hole size" |
| `pitch_mm` | Float (mm) | 1.25 | 0.1–10 (soft) | |
| `flank_angle_deg` | Float (°) | 60.0 | 1–179 | 60° metric/UNC, 55° BSP, 29° ACME |
| `truncation` | Float | 0.125 | 0.0–0.3 | |
| `resolution` | Int | 32 | 8–128 (soft) | |
| `inner_compensation_mm` | Float (mm) | 0.0 | 0.0–0.5 (soft) | Added to thread major radius — printed holes come out tight |

## Build method — three steps, in this order

1. **Cut a plain round bore** through the hex prism, sized to the
   thread's **root diameter** (`major_r`, minus a small overlap — same
   `overlap = max(0.02, min(0.2*depth, 0.15))` formula as `hex_bolt.py`),
   not the minor diameter. Cutting to the minor diameter would leave the
   body already solid where the ridge needs to sit, making the next step
   a no-op — you'd get a hole with no threads. The cutter is padded
   `±BOOL_EPSILON` past both ends of `z_height_mm` (safe for a
   subtractive cutter — over-cutting past a boundary is harmless).
2. **Union the internal-thread ridge** (`_internal_profile` +
   `_build_helix`) onto that bore. Unlike the bore cutter, this ridge
   gets **no** `±BOOL_EPSILON` padding — the source draws this contrast
   explicitly: "This ridge gets UNIONed (added), so any extension past
   the nut's real z=0..z_height range would show up as a visible nub
   poking out the top/bottom faces." Additive and subtractive steps need
   opposite padding conventions; see
   [README.md](README.md#boolean-solver-patterns-specific-to-this-family).
3. **Intersect against a clean Z-bound** cylinder spanning exactly
   `[0, z_height_mm]`. This exists to correct `_build_helix`'s own
   overshoot — its step count always rounds up, so the ridge from step 2
   can protrude slightly past the nut's nominal height without this final
   clip.

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
name). Same custom-property stamping as `hex_bolt.py`:
`bmech_thread_diameter`, `bmech_pitch`, `bmech_flank_angle_deg`,
`bmech_truncation`.
