# Herringbone Gear

`gears/external/herringbone_gear.py` → `OBJECT_OT_herringbone_gear` (`object.herringbone_gear`, "Herringbone Gear")

External involute gear with V-shaped (double-helical) teeth — two mirrored
helical halves meeting at mid-width, canceling axial thrust so no thrust
bearing is needed on the shaft. See [README.md](README.md) for family-wide
conventions and the hand-convention rules.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_opposite` — copies module/PA/helix angle, **inverts** hand |
| `tooth_count` | Int | 20 | 5–200 (soft) | |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Transverse module |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Half-angle of the V; 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | RIGHT = bottom half twists CW from below |
| `width_mm` | Float (mm) | 14.0 | 2–80 (soft) | **Full** face width — each half is `width_mm/2` |
| `bore_enable` | Bool | True | | |
| `bore_diameter` | Float (mm) | 5.0 | 0.1–50 (soft) | |
| `bore_compensation` | Float (mm) | 0.2 | 0.0–1.0 (soft) | Added to bore radius |
| `n_slices` | Int | 12 | 2–48 (soft) | **Per half.** Total slices built = `2*n_slices - 1` |

## Hand doesn't cancel thrust — it still has to match

A herringbone gear has zero net axial thrust *regardless of which hand you
pick* — that's the whole point of the V shape. But `hand` still determines
which way the V opens, and mating herringbone gears still need **opposite**
hands (same external-external rule as helical gears) for the two V's to
nest correctly. Don't assume hand is irrelevant here just because thrust
isn't at stake.

## Build method

Same slice-and-twist approach as the helical gear, done twice and mirrored:

```
bottom half, z: 0 → width/2       twist rises 0 → peak
top half,    z: width/2 → width   twist falls peak → 0
peak_twist = (width/2) * tan(helix_angle) / pitch_radius
```

The top-half loop starts at slice index 1, not 0, because slice 0 of the
top half is the same vertex ring as the last slice of the bottom half (the
shared peak at mid-height) — skipping it avoids a duplicate ring. Total
slice count is therefore `2*n_slices - 1`, shown as a read-only info line.

End caps here use **center-fan triangulation** (an added center vertex per
face, fanned to the rim), unlike the helical gear's flat n-gon caps — a
structural difference worth knowing if you're editing the generator code,
though it makes no visible difference to the output.

Bore cutting is identical to the helical gear's approach.

## Panel warnings

Same two checks as helical gear: `dedendum_radius <= 0` (ERROR, blocks) and
`bore_r >= dedendum_radius` (ERROR). The info box additionally shows Half
width, Peak twist, and Total slices.

Unlike helical gear, this operator **does** call
`self.report({'INFO'}, "Herringbone: %d teeth, %.1f° helix, %.1f mm wide")`
on success.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(obj, "herringbone", module, pressure_angle_deg,
                          tooth_count=tooth_count,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```

Meshes with: another herringbone gear (**opposite hand**), or a herringbone
annulus gear (**same hand** — see
[herringbone_annulus_gear.md](herringbone_annulus_gear.md)).
