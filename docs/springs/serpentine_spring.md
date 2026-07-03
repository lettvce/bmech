# Serpentine Spring

`springs/serpentine_spring.py` → `OBJECT_OT_add_serpentine_spring` (`object.add_serpentine_spring`, "Add Serpentine Spring")

A flat zigzag spring — straight legs connected by rounded U-bends,
compressing along one axis. See [README.md](README.md) for family-wide
notes.

## Termination convention — odd vs. even module count

```
Odd  module_count -> U-termination (both open ends on the same Y side)
Even module_count -> S-termination (open ends on opposite Y sides)
```

This falls directly out of how bends alternate direction (see Build
method below) — it isn't a separate setting, just a consequence of
`module_count`'s parity. The panel surfaces it as an informational note
when `module_count` is even.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `input_mode` | Enum | `PITCH_MODE` | `PITCH_MODE` (Module Count + Pitch → Length computed) / `LENGTH_MODE` (Module Count + Length → Pitch computed) | |
| `module_count` | Int | 5 | min 1 | Number of U-turn modules. **Always** a direct integer input in both modes — see below. |
| `spring_width` | Float (mm) | 20.0 | min 1.0 | Full Y-axis extent (center-to-center of bend centers) |
| `pitch` | Float (mm) | 4.0 | min 0.8 | Center-to-center distance between adjacent leg centerlines. Pitch Mode only. |
| `spring_length` | Float (mm) | 40.0 | min 1.0 | Total X-axis (compression-axis) extent. Length Mode only. |
| `strip_height` | Float (mm) | 2.0 | min 0.4 | Out-of-plane (Z) dimension — drives the Solidify modifier |
| `strip_thickness` | Float (mm) | 0.8 | min 0.0 | In-plane (X) cross-section — drives the outer/inner profile offset |

Note this is the one primitive in the whole library where **no property
has an upper bound** (no `max`/`soft_max` anywhere) — every other
generator sets at least a `soft_max` on its dimensional inputs.

## Why `module_count` is always typed in directly

Per the module's own header comment: deriving or flooring `module_count`
from a length+pitch combination would produce a mismatch between the
length/pitch you asked for and what the spring actually builds. Rather
than silently rounding, the generator always takes tooth — sorry, *module*
— count as a direct input in both modes, and computes whichever of
pitch/length is left over exactly.

- **PITCH_MODE**: `spring_length = module_count * pitch + strip_thickness`
  (computed, shown read-only).
- **LENGTH_MODE**: `pitch = (spring_length - strip_thickness) /
  module_count` (computed, shown read-only).

## Panel warnings — none of them actually block

- `leg_length <= 0` (derived from `spring_width - strip_thickness -
  2*bend_radius`) → **"Width too narrow — increase Width, decrease Pitch,
  or decrease Thickness"** (ERROR icon). Does **not** cancel —
  `execute()` clamps `leg_length = max(leg_length, 0.01)` instead.
- `pitch < strip_thickness` → **"Pitch must exceed strip thickness"**
  (ERROR icon). Does **not** cancel — `execute()` clamps
  `pitch = max(pitch, strip_thickness + 0.01)` instead.

The actual `CANCELLED` paths in this operator come from somewhere else
entirely: an exception raised inside `build_spring_quadstrip()`, or a
degenerate result with fewer than 3 vertices. Every ERROR-icon label in
this file's panel is a soft warning, not a gate — unusual among this
library's generators, most of which have at least one warning that
matches its blocking behavior.

## Build method

1. **Centerline**: `build_spring_centerline()` alternates straight legs
   (vertical runs at `y_bc_bot`/`y_bc_top`, inset from the true
   `spring_width` by `strip_thickness/2 + bend_radius` — a deliberate
   offset so that after the ribbon is offset outward in step 2, the
   *outer* edge lands exactly on `spring_width`, not the centerline)
   connected by semicircular bend arcs. Even-indexed legs bend `pi -> 0`;
   odd-indexed legs bend `pi -> 2*pi` — alternating which side each U-turn
   opens on is what produces the zigzag and drives the odd/even
   termination convention above. Bend arc segment count scales with bend
   radius (`max(8, round(pi*bend_radius/0.2))`, targeting ~0.2mm arc
   length per segment) rather than being a fixed constant.
2. **Offset**: `offset_centerline()` walks the centerline and offsets it
   by `±strip_thickness/2` along the local perpendicular (estimated by
   central-difference tangents), producing synchronized outer/inner point
   loops. At sharp corners the averaged-normal offset produces a slight
   chamfer rather than a sharp miter — a known, accepted approximation,
   documented in the source as staying under 1mm at typical strip sizes.
3. **Quad strip**: `build_spring_quadstrip()` builds flat quads directly
   between the outer and inner loops at Z=0, then triangulates and
   recalculates normals via a `bmesh` pass. `mesh.validate()` is checked
   afterward — if Blender had to auto-repair anything, a `print()`
   diagnostic is emitted (not a `self.report()`, so it won't show in the
   UI — check the system console if something looks subtly wrong).
4. **Thickness**: unlike `hairspring.py`, the Z dimension here **is** a
   Solidify modifier (`"SpringThickness"`, `thickness=strip_height`,
   `offset=0.0` symmetric, `use_even_offset=True`, `use_rim=True`) applied
   non-destructively to the flat quad-strip, not baked into the mesh.
5. Origin reset to `ORIGIN_GEOMETRY` / `BOUNDS` after creation.

## Output

One object per call, `SerpentineSpring` (mesh data also named
`SerpentineSpring`), with a live Solidify modifier attached. No
parenting, no `bmech_*` stamping. Success message:
`"Spring generated: %d modules, pitch=%.2f mm, bend_radius=%.2f mm"`.
