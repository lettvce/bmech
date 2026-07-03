# Hex Bolt

`fasteners/hex_bolt.py` → `OBJECT_OT_hex_bolt` (`object.hex_bolt`, "Hex Bolt")

A complete bolt — hex head, optional shank, external thread, optional
tip — assembled as one finished solid via a sequence of boolean unions.
Reuses `threaded_fastener.py`'s External+Additive thread math **verbatim
by duplication**, not by import — see
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
| `tip_enable` | Bool | True | | |
| `tip_length_mm` | Float (mm) | 3.0 | 0.1–30 (soft) | |
| `tip_diameter_mm` | Float (mm) | 0.0 | 0.0–80 (soft) | 0 = sharp point, >0 = flat dog-point tip |

## Build method

1. **Core stack** (`_add_chained_revolve`): shank → thread-core cylinder
   → tip built as one continuous solid of revolution sharing vertices at
   every junction, not three independently-capped primitives touching end
   to end. Independently-capped segments meeting at the same radius would
   leave duplicate coincident faces (both segments' own caps covering the
   identical disk) — a fragile case for the `EXACT` boolean solver.
   Chaining through shared vertices means the case can't arise.
   `tip_diameter_mm = 0` collapses the tip's final ring to a single shared
   pole vertex (a true point) rather than a zero-radius ring.
2. **Thread ridge** built separately via the duplicated
   `_external_profile` + `_build_helix`, then unioned onto the
   thread-core segment. The core is built slightly larger than the
   ridge's true minor radius first (`overlap = max(0.02, min(0.2*depth,
   0.15))`) so the union has real 3D volume to work with rather than two
   nearly-coincident curved surfaces — see
   [README.md](README.md#boolean-solver-patterns-specific-to-this-family).
3. **Hex head** (`_add_hex_prism`) built separately and unioned on
   **last** — a flush blocky join is far friendlier to the `EXACT` solver
   than a thin curved union, so the riskier step happens first and any
   failure surfaces before the cheap reliable one.
4. Each `_bool_union` re-runs `recalc_face_normals` afterward, since
   EXACT-solver unions can come back with inconsistent winding even when
   topologically solid.

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

One object, renamed `HexBolt`. Custom properties are stamped for other
tools to introspect the thread spec later: `bmech_thread_diameter`,
`bmech_pitch`, `bmech_flank_angle_deg`, `bmech_truncation`. This is
metadata only — there's no Match Target system here the way there is for
gears; nothing currently reads these properties back.
