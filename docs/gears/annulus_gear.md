# Annulus (Ring) Gear

`gears/ring/annulus_gear.py` → `OBJECT_OT_annulus_gear` (`object.annulus_gear`, "Annulus Gear")

Straight-tooth internal gear — involute teeth cut into the bore of a solid
ring, meshing with a spur pinion running inside it. See
[README.md](README.md) for family-wide conventions.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_module_pa` — copies module/PA only (no hand/helix on a straight gear) |
| `tooth_count` | Int | 40 | 8–200 (soft) | Must exceed the mating pinion's tooth count |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Transverse module — must match the mating pinion |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `width_mm` | Float (mm) | 10.0 | 1–80 (soft) | |
| `ring_wall_mm` | Float (mm) | 5.0 | 0.5–30 (soft) | Radial wall thickness beyond the tooth root |
| `outer_segs` | Int | 64 | 16–256 (soft) | Facets on the outer cylindrical surface |

## Meshing rule

Same module and pressure angle as the pinion, and avoid
`tooth_count(annulus) - tooth_count(pinion) < 12` — too few teeth of
difference risks interference between the ring's addendum and the pinion's
addendum with standard tooth proportions.

## Addendum/dedendum roles are swapped

An internal gear's teeth point inward, so the usual addendum/dedendum roles
invert relative to an external gear:

```
pitch_r      = module * tooth_count / 2
tip_r        = pitch_r - module            # ADDENDUM_COEFF role, now shrinks the radius
root_r_inner = pitch_r + 1.25*module       # DEDENDUM_COEFF role, now grows the radius
outer_r      = root_r_inner + ring_wall_mm
```

The cutter that carves these teeth is built by the exact same profile
code as an external gear's tooth (`_build_annulus_cutter_profile`), just
with `ADDENDUM_COEFF` and `DEDENDUM_COEFF` swapped in role — `ded_r`
becomes what reaches inward to the bore (tip), and `add_r` becomes what
stays out near the ring body (root).

## Build method

Boolean-only, **no Solidify modifier** (unlike the spur gear): a solid
outer cylinder is built at `outer_r` × `width_mm`, then the annulus cutter
profile is extruded into its own solid (`-BOOL_EPSILON` to
`width_mm + BOOL_EPSILON`) and boolean-DIFFERENCEd out (`EXACT` solver,
applied, cutter object deleted). Cap triangulation uses the family-wide
center-fan convention, not `triangle_fill` (see [README.md](README.md)).

No hand or helix concept anywhere in this generator — it's a pure
straight-tooth internal gear.

## Panel warnings

Only one check across all three annulus generators (straight, helical,
herringbone): `tip_r <= 0` → **"Module too large — tip radius ≤ 0"**
(ERROR, blocks). There is no rim-thickness or undercut-specific warning for
annulus gears, unlike the external-gear family — if you set `ring_wall_mm`
too thin, nothing in the panel will flag it.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(body, "annulus", module, pressure_angle_deg,
                          tooth_count=tooth_count)
```

Meshes with a spur pinion of the same module and pressure angle.
