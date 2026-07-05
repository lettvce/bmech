# Helical Rack

`gears/rack/helical_rack.py` → `OBJECT_OT_helical_rack` (`object.helical_rack`, "Helical Rack")

A straight rack whose teeth are sheared along X as a linear function of Z
— the gear-of-infinite-radius limit of [helical_gear.md](helical_gear.md)'s
twist. Meshes with a helical pinion of the same module, pressure angle,
and helix angle, and the **same** hand — **not** the opposite hand the
external-external gear-gear rule would suggest. See
[README.md](README.md#hand-convention--the-one-rule-most-likely-to-bite-you)
for why a rack is the exception: it contacts its pinion at a different
relative angular position (directly below, θ=-90°) than two side-by-side
external gears do (θ=180° apart), and that's what flips the sign.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_helical_same`, and if the target has `bmech_tooth_count`, also updates `tooth_count_rack` |
| `module` | Float (mm) | 1.0 | 0.1–50.0 | Must match the mating pinion's module |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Tooth shear angle; 15–30° typical for FDM |
| `hand` | Enum | `RIGHT` | `RIGHT` / `LEFT` | Teeth shear toward +X as Z increases when `RIGHT` |
| `length_mode` | Enum | `TOOTH_COUNT` | `TOOTH_COUNT` / `MATCH_GEAR` | Same semantics as [straight_rack.md](straight_rack.md#length_mode) |
| `tooth_count_rack` | Int | 10 | 2–1000 | Used directly in `TOOTH_COUNT` mode; fallback in `MATCH_GEAR` mode if no target is set |
| `width_mm` | Float (mm) | 10.0 | 1–100 (soft) | Full extent along the shear (Z) axis |

No bore properties, no `n_slices` — see Build method below for why.

## Hand and Match Target freezing

Same freeze pattern as [helical_gear.md](helical_gear.md): `module`/
`pressure_angle_deg` freeze whenever any target is set; `helix_angle_deg`
freezes only when the target itself stamps `bmech_helix_angle_deg` (a
spur-gear target never drives it, matching `sync_helical_same`'s own
guard); `hand` freezes with `helix_angle_deg` **except** when
`gear_matching.hand_target_ambiguous(False, target)` is true — i.e. when
this rack (plain-helical style) is matched against a herringbone target,
since a plain helical rack only meshes one half of a herringbone pinion's
V, and which hand is correct depends on which half. See
[README.md](README.md#the-match-target-system-gear_matchingpy) for the
full rationale — this rack follows exactly the same logic
`helical_gear.py` uses for gear-gear pairs.

## Build method

**No `n_slices` property, unlike every helical/herringbone *gear*.** A
helical gear twists by *rotating* each Z-slice's profile by an angle that
grows nonlinearly relative to straight-line interpolation between slices,
so more slices measurably improve the approximation to the true curved
helical surface. A helical rack instead *shears* the profile —
`x' = x + hand_sign * z * tan(helix_angle)`, a transform that is **linear
in z**. A straight 3D edge between the bottom-layer vertex `(x, y, 0)` and
the top-layer vertex `(x + shear(width), y, width)` passes exactly through
`(x + shear(z), y, z)` for every z in between (parametrize `t = z/width`:
both the edge and `shear(z)` scale identically with `t`). This is an exact
affine identity, not an approximation, so **exactly two Z-layers** —
bottom and top — reproduce the sheared surface with no faceting error at
any resolution. Adding an `n_slices` property here would misleadingly
suggest a smoothness/accuracy tradeoff that doesn't exist for this shape.

`shear(z) = z * tan(helix_angle)` — note there's no division by pitch
radius, unlike the gear's `twist_angle(z) = z * tan(helix_angle) /
pitch_radius`. The rack is the pitch-radius-→-∞ limit of the gear case:
the *arc-length* displacement of any profile point (`radius *
twist_angle(z)`) converges to the radius-independent constant
`z * tan(helix_angle)` as radius grows without bound, since displacement
at any fixed radius on the pitch circle is what a rack tooth's true
straight-line motion represents.

Otherwise reuses `straight_rack.py`'s tooth-profile math verbatim
(`build_rack_tooth_profile`/`build_rack_profile`, duplicated here per this
family's per-file mesh-helper convention — see
[README.md](README.md#the-match-target-system-gear_matchingpy)), tiled
across X, then two straight-side-quad layers connect the sheared bottom
and top n-gon faces.

Swept tooth counts 2–50 × pressure angles 14.5–30° × helix angles 5–44° ×
both hands: 0 non-manifold edges, 0 zero-area faces (checked on the
evaluated mesh — see [straight_rack.md](straight_rack.md#build-method) for
why the raw pre-modifier/pre-eval mesh isn't the right thing to check; this
generator has no modifiers at all, everything is baked bmesh geometry, but
the same "evaluate before trusting" habit caught this reliably).

## Hand fix — why this needed correcting after initial release

The first version of this file reused `sync_helical_opposite` (the
external-external gear-gear rule) on the assumption that a rack, having
external tooth geometry, follows the same hand rule as two side-by-side
external gears. It doesn't. The two cases contact at *different relative
angular positions* on the pinion — two side-by-side gears contact at a
θ=180°-apart pair of points (each gear's contact point faces the other),
while a rack sitting directly below its pinion contacts at the pinion's
own θ=-90° point (a 90°, not 180°, offset from a θ=0° reference). That
difference changes the sign of the tooth-flank-slope relationship the
hand rule depends on.

This was caught by direct empirical testing, not just re-deriving the
angle math: building a 30°-helix pinion together with a rack of the *same*
module/PA/helix at each hand value and computing the actual mesh
intersection volume (`bmesh.ops.boolean` `INTERSECT`, `EXACT` solver) gave
**0 mm³** for same-hand and **~376 mm³** of gross tooth interpenetration
for opposite-hand. Independently, extracting real mesh vertices from the
gear confirmed its tooth flank at the rack-facing contact point shifts by
measured `dx/dz ≈ +tan(helix_angle)` for `hand='RIGHT'` — exactly matching
this file's own `shear(z)` formula with no sign flip needed there. The fix
was entirely in `bmech_sync_target` (swap `sync_helical_opposite` for
`sync_helical_same`) and the hand-property/docs wording — the shear math
itself was already correct.

## Phase alignment

Beyond the Y-drop-by-pitch-radius and X-centering every rack does when a
target is set (see [straight_rack.md](straight_rack.md#length_mode)), a
rack also applies `gear_matching.rack_phase_align_x(target,
tooth_count_rack)` — an extra X shift so the rack spawns with a gear
tooth centered in a rack *gap* (and vice versa) rather than an arbitrary
phase.

**This is cosmetic, not a correctness fix.** Standard (zero-profile-shift)
involute teeth are in conjugate contact at every point along the path of
contact — every rotational phase of a correctly-dimensioned pair is
interference-free. Without this shift, whether the rack spawns looking
"nicely meshed" (tooth-into-gap) or "tooth stabbing into tooth" (not
actually wrong, just a confusing snapshot to hand a user) is arbitrary,
driven by whichever tooth-count/tooth-count-parity combination the user
happened to pick.

**Why `tooth_count_rack`'s parity has to be part of the calculation, not
just the target's tooth count** — a mistake made and caught in this
file's own first draft: the *existing* X-centering formula
(`half_rack_length - tooth_pitch/2`) already puts either a rack
tooth-center or a rack gap-center at the target's local x=0, depending on
whether `tooth_count_rack` is odd or even (a rack tooth sits there for
odd counts, a gap for even). A first version of `rack_phase_align_x`
computed a shift purely from the gear's own tooth phase, blind to that
parity. Hand-checking a concrete case caught it: a module=2, 40-tooth
gear's own tooth phase happens to sit exactly at the meshing point
(θ=-90° is an exact multiple of that gear's tooth-pitch angle) — but for
a 10-tooth rack (even), the *pre-existing* centering formula independently
puts a rack *gap* at that same point already, meaning the two effects
already canceled into perfect gap-alignment with **zero** shift needed.
The naive gear-only formula didn't know that, and would have added a
half-tooth-pitch shift that moved this already-correct pairing into
tooth-on-tooth instead — while doing nothing wrong for an 11-tooth (odd)
rack against the same gear, where the parity-blind and parity-aware
formulas happen to agree. Verified after the fix across gear tooth counts
{17, 20, 39, 40, 41} × rack tooth counts {5, 10, 11, 20, 21} (a mix of odd/
even/coprime-with-40 combinations chosen to stress exactly this parity
interaction): all 25 combinations land with a rack tooth center exactly
`tooth_pitch/2` away from the nearest gear tooth center — true gap
alignment, not tooth-on-tooth, at every single tooth count parity tested.
Confirmed unchanged for the helical and herringbone variants too (checked
at their own θ=0 reference slice, since the same-hand fix keeps the shear
in sync with the gear's twist across the full width once aligned at one
slice).

## Output

One object, name `HelicalRack` (or `HelicalRack.001`, etc.), stamped:
```python
gear_matching.stamp_gear(obj, "helical_rack", module, pressure_angle_deg,
                          helix_angle_deg=helix_angle_deg, hand=hand)
```
Meshes with a helical pinion ([helical_gear.md](helical_gear.md)) of the
same module, pressure angle, and helix angle, and the **same** hand.
