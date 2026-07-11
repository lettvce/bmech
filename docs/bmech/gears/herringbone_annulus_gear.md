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

**No boolean, no Solidify modifier** — same direct-bmesh rewrite as the
straight and helical annulus gears ([annulus_gear.md](annulus_gear.md#build-method),
[helical_annulus_gear.md](helical_annulus_gear.md#build-method)), extended
to a V-shaped (herringbone) twist. This used to be a solid outer cylinder
minus a boolean-DIFFERENCE herringbone cutter (`EXACT` solver).

`_build_herringbone_annulus_solid` builds:
- The **inner (toothed) wall**: `2*n_slices - 1` V-twisted Z-layers of the
  tooth profile — bottom half twist rising `0 → peak_twist` over
  `[0, width_mm/2]`, top half falling `peak_twist → 0` over
  `[width_mm/2, width_mm]`, sharing the mid-slice at the peak (the top-half
  loop starts at `k=1` to avoid rebuilding that shared slice). Twist is
  relative to `z=0`/`width_mm` directly now — the old boolean cutter had
  to pad its Z range by `±BOOL_EPSILON` to avoid a coincident-cap boolean
  artifact, which no longer applies since there's no boolean.
  `peak_twist = (width_mm/2) * tan(helix_angle) / pitch_radius`.
- The **outer (cylindrical) wall**: a PLAIN, UNTWISTED, independently-
  spaced circle (`outer_segs` points, only 2 Z-layers) — the outer surface
  doesn't twist, only the inner teeth do. As with the other two annulus
  generators, this is deliberately NOT built with one point per
  tooth-profile point at a matching angle (that reintroduces zero-area
  triangles at the tooth profile's collinear dedendum-circle point
  insertion).
- The **two end caps**: `bmesh.ops.triangle_fill` fed the boundary edges
  of both the (twisted) inner ring and the (untwisted) outer ring together
  at each end.

**Pressure angle clamp has extra margin here too**: `_derived()` clamps to
`gear_matching.max_pressure_angle_deg(...) - PA_TRIANGLE_FILL_MARGIN_DEG`
(0.2°), not the theoretical limit itself — `triangle_fill` is more fragile
right at that boundary than the old `EXACT`-solver boolean was. Swept
tooth counts 8-100 × pressure angles 10-45° × both hands: 96/96 clean.

`herringbone_planetary_gear_set.py`'s own ring gear is unaffected by this
rewrite — it's a separate file with its own cutter function
(`_make_herringbone_cutter_obj`), still boolean-based.

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
