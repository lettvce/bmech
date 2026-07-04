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

**No boolean, no Solidify modifier** — the ring is built directly with
bmesh. This used to be a solid outer cylinder minus a boolean-DIFFERENCE
tooth cutter (`EXACT` solver); that was ~80-180x slower in testing
(295ms-5.8s for tooth counts 8-100, scaling badly with tooth count) and
was the most sluggish generator in the whole gear family. The direct
construction produces the identical shape in 3.7-35ms.

`_build_annulus_solid` builds four pieces directly:
- The **inner (toothed) wall**: consecutive points of the tooth profile
  connected vertically between the two Z layers — same idea as an
  external gear's own tooth-wall construction.
- The **outer (cylindrical) wall**: a separate, independently-spaced
  circle (`outer_segs` points) connected the same way. This is
  deliberately **not** built with one point per tooth-profile point at a
  matching angle — a matched 1:1 correspondence looks like it would let
  the two loops bridge directly into a clean cap, but doesn't work: the
  tooth profile inserts a dedendum-circle point at the same angle as its
  neighbor wherever `base_r > ded_r` (a genuine straight undercut-flank
  segment, not a mistake), and bridging to a same-angle outer point puts
  three points on the same ray through the origin — an unavoidably
  zero-area triangle regardless of how the resulting quad gets split.
  Confirmed by testing: a matched-angle outer ring reintroduces
  non-manifold edges and zero-area faces at low tooth counts (8/15/20)
  that treating the outer ring as fully independent does not.
- The **two end caps**, each built with `bmesh.ops.triangle_fill` fed the
  boundary edges of **both** loops together in one call. `triangle_fill`
  treats the inner loop as a hole in the outer loop's polygon and
  triangulates the actual annular region directly, with no trouble from
  the profile's collinear-point segment, since it isn't assuming any
  point correspondence between the loops at all — see
  [README.md](README.md) for the fuller history (a fan-from-a-center-
  vertex approach was tried and produced degenerate zero-area cap faces;
  a naive index-matched bridge was tried next and produced the same
  degenerate faces for a different reason; `triangle_fill` fed both loops
  together is what actually works).

### Pressure angle clamp has extra margin, specifically for this generator

`_derived()` clamps `pressure_angle_deg` to
`gear_matching.max_pressure_angle_deg(...) - PA_TRIANGLE_FILL_MARGIN_DEG`
(0.2°), not the theoretical self-intersection limit itself the way other
gear generators do via `gear_matching.clamp_pressure_angle`. Right at that
limit the tooth tip's flank points become near-coincident — the old
boolean-based `EXACT` solver tolerated a profile sitting exactly there,
but `bmesh.ops.triangle_fill`'s constrained triangulation does not, and
produced real non-manifold edges (292-844 of them, in testing) without
this margin. 0.2° reliably clears it for tooth counts up to ~100 — the
range that covers virtually every practical FDM design.

**Known limitation, not fixed**: at very high tooth counts (100+, already
far beyond typical printable annulus gear sizes) combined with a
pressure angle that gets clamped, an occasional tiny residual (2-6
non-manifold edges, out of tens of thousands) can still appear. It does
not correlate cleanly with pressure-angle margin, `outer_segs`, or their
ratio in testing — it behaves like floating-point noise in
`triangle_fill`'s own triangulation at extreme profile complexity, not a
parameter with an obvious closed-form fix. Swept 8-100 teeth × the full
pressure-angle range: 166/168 combinations (98.8%) built completely clean;
the 2 failures were both `tooth_count=100` with a *requested* pressure
angle (40-45°) far above what actually gets used after clamping (~31°) —
i.e., only reachable by deliberately asking for an unrealistic value at
an already-extreme tooth count.

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
