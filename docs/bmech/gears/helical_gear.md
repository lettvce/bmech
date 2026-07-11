# Helical Gear

`gears/external/helical_gear.py` → `OBJECT_OT_helical_gear` (`object.helical_gear`, "Helical Gear")

External involute gear with twisted (helical) teeth for quieter, stronger
meshing than a spur gear. See [README.md](README.md) for family-wide
conventions, especially the **hand convention** section — this is the
primitive that introduces `hand`.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_opposite` — copies module/PA/helix angle and **inverts** hand |
| `tooth_count` | Int | 20 | 5–200 (soft) | |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Transverse module |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | RIGHT = teeth twist CW looking from top |
| `width_mm` | Float (mm) | 10.0 | 1–50 (soft) | |
| `bore_enable` | Bool | True | | |
| `bore_diameter` | Float (mm) | 5.0 | 0.1–50 (soft) | |
| `bore_compensation` | Float (mm) | 0.2 | 0.0–1.0 (soft) | Added to bore radius |
| `n_slices` | Int | 16 | 2–64 (soft) | Z divisions — more = smoother helix |

## Build method

Unlike the spur gear (flat profile + Solidify), the helical gear is built
as a genuine 3D solid: one 2D involute profile is generated once, then
extruded across `n_slices` Z-layers from 0 to `width_mm`, each layer
rotated by

```
twist(z) = hand_sign * z * tan(helix_angle) / pitch_radius
hand_sign = +1 for RIGHT, -1 for LEFT
```

Side walls are quads between consecutive slices; end caps are the first
(reverse-wound) and last slice as flat n-gons. This is the entire hand
implementation — `hand` only ever flips the sign on this one formula.

Total twist across the full face width (`width_mm * tan(helix_angle) /
pitch_radius`) and the normal module (`module * cos(helix_angle)`) are
shown as read-only info lines in the panel.

Bore is cut the same way as the spur gear's (`EXACT`-solver boolean
DIFFERENCE, cutter extended `±BOOL_EPSILON` past both faces), but here as a
genuinely separate boolean step since there's no Solidify modifier to
apply first.

## Panel warnings

- `dedendum_radius <= 0` → **"Module too large — dedendum radius ≤ 0"**
  (ERROR, blocks — silent `CANCELLED`, no `self.report()`).
- `bore_r >= dedendum_radius` → **"Bore larger than dedendum radius"**
  (ERROR). Note the bore-skip logic in `execute()` uses strict `<`, so this
  threshold and the actual skip condition line up exactly.

Unlike the herringbone gear, this operator does **not** call
`self.report({'INFO'}, ...)` on success — no confirmation message beyond
the created object.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(obj, "helical", module, pressure_angle_deg,
                          tooth_count=tooth_count,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```

Meshes with: another helical gear (**opposite hand**, same module/PA/helix
angle), or a helical annulus gear (**same hand** — see
[helical_annulus_gear.md](helical_annulus_gear.md)).
