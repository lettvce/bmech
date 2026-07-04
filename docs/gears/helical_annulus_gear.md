# Helical Annulus Gear

`gears/ring/helical_annulus_gear.py` → `OBJECT_OT_helical_annulus_gear` (`object.helical_annulus_gear`, "Helical Annulus Gear")

Helical internal gear — twisted involute teeth cut into the bore of a solid
ring. See [README.md](README.md) for family-wide conventions and
[annulus_gear.md](annulus_gear.md) for the swapped addendum/dedendum
geometry shared by all three annulus generators.

## This is where the hand convention flips

**Read this even if you've already read the helical/herringbone external
gear docs.** External-external helical pairs need opposite hands; a
helical annulus and its mating helical pinion need the **SAME** hand.
Right annulus meshes with a right pinion, not a left one. This is the
single most common mistake this library's Match Target system exists to
prevent — which is why picking a target here runs `sync_helical_same`, not
`sync_helical_opposite`.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_same` — copies module/PA/helix angle and matches hand **exactly** |
| `tooth_count` | Int | 40 | 8–200 (soft) | Must exceed the mating pinion's tooth count |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Transverse module — must match the mating pinion |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | Must match the pinion's hand — see above |
| `width_mm` | Float (mm) | 10.0 | 1–80 (soft) | |
| `ring_wall_mm` | Float (mm) | 5.0 | 0.5–30 (soft) | Radial wall thickness beyond the tooth root |
| `n_slices` | Int | 16 | 2–64 (soft) | Z divisions for the twisted inner (toothed) wall |
| `outer_segs` | Int | 64 | 16–256 (soft) | Facets on the outer cylindrical surface |

## Build method

**No boolean, no Solidify modifier** — same direct-bmesh rewrite as the
straight annulus gear ([annulus_gear.md](annulus_gear.md#build-method)),
extended across twisted Z-slices. This used to be a solid outer cylinder
minus a boolean-DIFFERENCE helical cutter (`EXACT` solver); measured
~10-40x slower in testing (330ms-5.0s for tooth counts 8-100 vs 9-133ms
direct).

`_build_helical_annulus_solid` builds:
- The **inner (toothed) wall**: `n_slices` twisted Z-layers of the tooth
  profile, twist computed **relative to `z=0`**
  (`hand_sign * z * tan(helix_angle) / pitch_radius`) — simpler than the
  old boolean cutter's twist formula, which had to be relative to the
  cutter's own padded bottom face to avoid a boolean seam artifact that no
  longer exists here. `hand_sign` is `+1` for `RIGHT`, `-1` for `LEFT`,
  same convention as every other helical primitive.
- The **outer (cylindrical) wall**: a PLAIN, UNTWISTED, independently-
  spaced circle (`outer_segs` points, only 2 Z-layers). The outer surface
  of a helical annulus doesn't twist — only the inner teeth do. As with
  the straight annulus gear, this is deliberately NOT built with one
  point per tooth-profile point at a matching angle (that reintroduces
  zero-area triangles at the tooth profile's collinear dedendum-circle
  point insertion).
- The **two end caps**: `bmesh.ops.triangle_fill` fed the boundary edges
  of both the (twisted) inner ring and the (untwisted) outer ring
  together at each end — see [annulus_gear.md](annulus_gear.md#build-method)
  for the fuller rationale.

**Pressure angle clamp has extra margin here too**: `_derived()` clamps to
`gear_matching.max_pressure_angle_deg(...) - PA_TRIANGLE_FILL_MARGIN_DEG`
(0.2°), not the theoretical limit itself — `triangle_fill` is more fragile
right at that boundary than the old `EXACT`-solver boolean was. Swept
tooth counts 8-100 × pressure angles 10-45° × both hands: 96/96 clean.

Total twist (`width_mm * tan(helix_angle) / pitch_radius`) and normal
module (`module * cos(helix_angle)`) are shown as read-only info lines.

## Panel warnings

Only `tip_r <= 0` → **"Module too large — tip radius ≤ 0"** (ERROR,
blocks). No rim-thickness warning, matching the straight annulus gear.

No print-in-place clearance logic in this generator — it produces a rigid
single-part ring meant to be printed separately from its mating pinion.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(body, "helical_annulus", module, pressure_angle_deg,
                          tooth_count=tooth_count,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```

Meshes with a helical pinion ([helical_gear.md](helical_gear.md)) of the
same module, pressure angle, and helix angle, and the **same** hand.
