# Herringbone Annulus Gear

`gears/ring/herringbone_annulus_gear.py` → `OBJECT_OT_herringbone_annulus_gear` (`object.herringbone_annulus_gear`, "Herringbone Annulus Gear")

Herringbone internal gear — V-shaped involute teeth cut into the bore of a
solid ring. See [README.md](README.md) for family-wide conventions and
[annulus_gear.md](annulus_gear.md) for the swapped addendum/dedendum
geometry shared by all three annulus generators.

## Hand convention

Same rule as the helical annulus gear: a herringbone annulus and its
mating herringbone pinion need the **SAME** hand (unlike external-external
herringbone pairs, which need opposite hands). Picking a target here runs
`sync_helical_same`. See
[helical_annulus_gear.md](helical_annulus_gear.md#this-is-where-the-hand-convention-flips)
for why this matters.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_same` |
| `tooth_count` | Int | 40 | 8–200 (soft) | Must exceed the mating pinion's tooth count |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Transverse module — must match the mating pinion |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Half-angle of the V; 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | Bottom half twists CW from below when RIGHT |
| `width_mm` (labeled "Total Width") | Float (mm) | 14.0 | 2–80 (soft) | Full face width — each half is `width_mm/2` |
| `ring_wall_mm` | Float (mm) | 5.0 | 0.5–30 (soft) | Radial wall thickness beyond the tooth root |
| `n_slices` (labeled "Slices per Half") | Int | 12 | 2–48 (soft) | Total slices built = `2*n_slices - 1` |
| `outer_segs` | Int | 64 | 16–256 (soft) | Facets on the outer cylindrical surface |

Note the property labels here ("Total Width", "Slices per Half") are more
explicit than the plain "Width"/"Slices" labels on the helical annulus
gear, because herringbone splits both properties' semantics across two
mirrored halves.

## Build method

The cutter is built the same way as an external herringbone gear's body —
bottom half twist rising `0 → peak_twist`, top half falling
`peak_twist → 0` — but as a boolean cutter rather than the final solid, and
extended `±BOOL_EPSILON` past the body at both ends.

`peak_twist = (width_mm/2) * tan(helix_angle) / pitch_radius`.

One deliberate accuracy trade-off, called out directly in the source: the
z-range extension for boolean safety means twist is technically `0` at
`z = -BOOL_EPSILON` and returns to `0` at `z = width_mm + BOOL_EPSILON`
rather than exactly at the nominal faces — a 0.001mm deviation, accepted
as negligible in exchange for boolean robustness.

The top-half loop skips its first slice (shared with the bottom half's
last slice, at the peak) — same duplicate-vertex-ring avoidance as the
external herringbone gear.

## Panel warnings

Only `tip_r <= 0` → **"Module too large — tip radius ≤ 0"** (ERROR,
blocks). Info box additionally shows normal module, peak twist, and total
slice count (`2*n_slices - 1`).

No print-in-place clearance logic — produces a rigid single-part ring
meant to be printed separately from its pinion.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(body, "herringbone_annulus", module, pressure_angle_deg,
                          tooth_count=tooth_count,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```

Meshes with a herringbone pinion
([herringbone_gear.md](herringbone_gear.md)) of the same module, pressure
angle, and helix angle, and the **same** hand.
