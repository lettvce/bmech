# Herringbone Rack

`gears/rack/herringbone_rack.py` → `OBJECT_OT_herringbone_rack` (`object.herringbone_rack`, "Herringbone Rack")

A straight rack sheared into a V along X: the bottom half shears one way
from `Z=0` to `Z=width/2`, the top half shears back the other way from
`Z=width/2` to `Z=width` — the gear-of-infinite-radius limit of
[herringbone_gear.md](herringbone_gear.md)'s V-twist, the same way
[helical_rack.md](helical_rack.md) is the limit of
[helical_gear.md](helical_gear.md)'s twist.

## Hand convention

Same rule as the helical rack: mating with a herringbone pinion needs the
**same** hand — **not** the opposite hand the external-external gear-gear
rule would suggest, because a rack contacts its pinion at a different
relative angular position than two side-by-side external gears do. See
[README.md](README.md#hand-convention--the-one-rule-most-likely-to-bite-you)
and [helical_rack.md](helical_rack.md#hand-fix--why-this-needed-correcting-after-initial-release)
for the full derivation and empirical verification. Herringbone parts have
no net axial thrust regardless of hand, but hand still determines which
way the V opens and must match the mating pinion.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_same`, and if the target has `bmech_tooth_count`, also updates `tooth_count_rack` |
| `module` | Float (mm) | 1.0 | 0.1–50.0 | Must match the mating pinion's module |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Half-angle of the V; 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | Bottom half shears toward +X as Z increases when `RIGHT` |
| `length_mode` | Enum | `TOOTH_COUNT` | `TOOTH_COUNT` / `MATCH_GEAR` | Same semantics as [straight_rack.md](straight_rack.md#length_mode) |
| `tooth_count_rack` | Int | 10 | 2–1000 | Used directly in `TOOTH_COUNT` mode; fallback in `MATCH_GEAR` mode if no target is set |
| `width_mm` (labeled "Total Width") | Float (mm) | 14.0 | 2–100 (soft) | Full extent along the V axis — each half is `width_mm/2` |

No bore properties, no `n_slices` — see Build method below for why.

## Hand and Match Target freezing

Same freeze pattern as [herringbone_gear.md](herringbone_gear.md):
`module`/`pressure_angle_deg` freeze whenever any target is set;
`helix_angle_deg` freezes only when the target stamps
`bmech_helix_angle_deg`; `hand` freezes with `helix_angle_deg` **except**
when `gear_matching.hand_target_ambiguous(True, target)` is true — i.e.
when this rack (herringbone-style) is matched against a plain-helical
target, since a plain helical pinion only meshes one half of this rack's V
at a time, and which hand is correct depends on which half. See
[README.md](README.md#the-match-target-system-gear_matchingpy) for the
full rationale.

## Build method

**No `n_slices` property, unlike `herringbone_gear.py`.** Each half of the
V is its own linear (affine) shear of Z within its own range — see
[helical_rack.md](helical_rack.md#build-method) for the full derivation of
why an affine shear needs no intermediate Z-layers to be exact, unlike a
gear's rotational twist. A herringbone rack therefore needs exactly
**three** layers total: `Z=0` (shear 0), `Z=width/2` (peak shear), and
`Z=width` (shear back to 0) — not `2*n_slices-1` layers approximating a
curved surface the way `herringbone_gear.py` needs.

```
peak_shear = (width/2) * tan(helix_angle)
```

applied as `x' = x + hand_sign * shear` at each of the three layers, with
`hand_sign = +1` for `RIGHT`, `-1` for `LEFT` — same sign convention as
every other hand-aware generator in this family.

Reuses `straight_rack.py`'s tooth-profile math verbatim
(`build_rack_tooth_profile`/`build_rack_profile`, duplicated here per this
family's per-file mesh-helper convention), tiled across X. Two straight
side-quad strips connect bottom→middle and middle→top, each exact for the
same affine reason described in [helical_rack.md](helical_rack.md).

Swept tooth counts 2–50 × pressure angles 14.5–30° × helix angles 5–44° ×
both hands: 0 non-manifold edges, 0 zero-area faces on the evaluated mesh.

## Hand fix — why this needed correcting after initial release

Same root cause and fix as [helical_rack.md](helical_rack.md#hand-fix--why-this-needed-correcting-after-initial-release):
`bmech_sync_target` originally called `sync_helical_opposite` by analogy
with the gear-gear external-external rule. Verified wrong by building a
30°-helix herringbone pinion with a same-hand rack (0 mm³ mesh
interpenetration, boolean `EXACT` `INTERSECT`) versus an opposite-hand
rack (~388 mm³ of gross overlap) at matching module/PA/helix. Fixed by
swapping to `sync_helical_same` — the shear math itself (each half's
`peak_shear` formula) was already correct; only the hand-sync direction
and the property/doc wording needed correcting.

## Phase alignment

Same cosmetic X phase-align as every rack in this family — see
[helical_rack.md#phase-alignment](helical_rack.md#phase-alignment) for
the full derivation. Confirmed unaffected by the V-shear: alignment is
computed at the Z=0 slice, and the same-hand fix above keeps that phase in
sync with the gear's own twist across the whole width once aligned there.

## Output

One object, name `HerringboneRack` (or `HerringboneRack.001`, etc.),
stamped:
```python
gear_matching.stamp_gear(obj, "herringbone_rack", module, pressure_angle_deg,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```
Meshes with a herringbone pinion
([herringbone_gear.md](herringbone_gear.md)) of the same module, pressure
angle, and helix angle, and the **same** hand.
