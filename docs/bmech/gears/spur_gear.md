# Spur Gear

`gears/external/spur_gear.py` → `OBJECT_OT_add_spur_gear` (`object.add_spur_gear`, "Add Spur Gear")

Straight-tooth external involute gear. The baseline gear primitive — every
other external-gear generator in this library (helical, herringbone, bevel)
is a variation on the same tooth-profile math built in this file. See
[README.md](README.md) for conventions shared across the whole family.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_module_pa` |
| `module` | Float (mm) | 1.0 | 0.1–50.0 | |
| `tooth_count` | Int | 20 | 5–500 | |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `width_mm` | Float (mm) | 6.0 | 0.1–100 (soft) | Solidify modifier depth |
| `bore_enable` | Bool | True | | |
| `bore_diameter` | Float (mm) | 5.0 | 0.1–50 (soft) | |
| `bore_compensation` | Float (mm) | 0.2 | 0.0–1.0 (soft) | Added to bore radius |

## Build method

Unlike every other gear primitive in the library, the spur gear is built as
a **flat 2D n-gon profile + Solidify modifier**, not a hand-extruded solid
mesh. `build_gear_profile()` produces a closed polygon (tooth flanks are
involute curves sampled at `INVOLUTE_POINTS=15` points, tooth spaces are
dedendum-circle arcs), which is filled as a single face and thickened with
a `SOLIDIFY` modifier (`thickness=width_mm`, `offset=0.0`).

If `bore_enable`, the Solidify modifier is applied first (so the bore
boolean operates on a real solid, not a modifier stack), then a cylinder
cutter is boolean-subtracted (`EXACT` solver) using the same
`bore_r = bore_diameter/2 + bore_compensation` convention as the rest of
the family.

## Panel warnings

- `pitch_r*cos(pressure_angle) > dedendum_radius` → **"Undercut likely"**
  (INFO-severity warning, not blocking) — thin tooth count / large
  pressure angle combinations can undercut the tooth root.
- `dedendum_radius <= 0` → **"Module too large — dedendum radius is zero
  or negative"** (ERROR, blocks — `execute()` returns `CANCELLED`).
- `bore_r >= dedendum_radius` (with bore enabled) → **"Bore too large for
  dedendum radius"** (ERROR).

## Output

One object, name `Gear` (or `Gear.001`, etc. via `unique_name`), stamped
`gear_matching.stamp_gear(obj, "spur", module, pressure_angle_deg,
tooth_count=tooth_count)`. No `hand` or `helix_angle_deg` fields — spur
gears are straight-toothed.
