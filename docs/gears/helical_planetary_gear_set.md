# Helical Planetary Gear Set

`gears/planetary/helical_planetary_gear_set.py` → `OBJECT_OT_helical_planetary_gear_set` (`object.helical_planetary_gear_set`, "Helical Planetary Gear Set")

Same topology and assembly math as
[planetary_gear_set.md](planetary_gear_set.md) — read that first. This
document only covers what's different: helical teeth on all three members,
which introduces a hand-derivation rule specific to three-body epicyclic
sets.

## Properties

Same as the spur planetary set, plus:

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Applies to all three members |
| `hand` (labeled "Sun Hand") | Enum | `RIGHT` | `RIGHT` / `LEFT` | Sets the **sun's** hand only — planet and ring are derived |
| `n_slices` | Int | 16 | 2–64 (soft) | Z divisions for the helical twist, applies to all three members |

`sun_teeth`, `planet_teeth`, `planet_count`, `module`,
`pressure_angle_deg`, `width_mm`, `ring_wall_mm`, `pip_gap`, `outer_segs`
are identical to the spur set.

## Hand is chosen once, for the sun — everything else follows

You only ever set the hand of the **sun** gear. Planet and ring hands are
derived automatically from the two mesh rules in
[README.md](README.md#hand-convention--the-one-rule-most-likely-to-bite-you):

```
sun-planet is external-external  → planet = OPPOSITE of sun
planet-ring is external-internal → ring   = SAME as planet
                                  → ring   = OPPOSITE of sun
```

In code this is `sun_sign = ±1` from the `hand` enum, then
`planet_sign = -sun_sign`, `ring_sign = planet_sign`. The panel shows this
as a one-line summary: `"R→L→L"` when sun is RIGHT, `"L→R→R"` when sun is
LEFT.

## Planet phasing is unchanged by the helix

The planet orbital-rotation formula
(`-theta_i * sun_teeth/planet_teeth + pi/planet_teeth`) and the ring's
fixed `-pi/ring_teeth` phase correction are **identical** to the spur set
— helix twist doesn't change the z=0 phase of any member, only how each
member's teeth wind along Z above that phase.

## Build method

Sun and planet: same per-slice twist extrusion as a standalone helical gear
(`helical_gear.py`), just parameterized by `sun_sign` / `planet_sign`
instead of a user-facing `hand` property. Ring: solid cylinder minus a
twisted annulus cutter (same approach as
[helical_annulus_gear.md](helical_annulus_gear.md)), using `ring_sign`.

Info box additionally shows normal module (`module*cos(helix_angle)`) and
the sun's total twist across its face width, in degrees.

## Output

**`2 + planet_count` objects per call**: `HelPlanetaryRing`,
`HelPlanetarySun`, and `HelPlanetaryPlanet.001`…`.00N` (linked-mesh
copies, same as the spur set). Success message:
`"Helical planetary: %d/%d/%d teeth (sun/planet/ring), %.1f° helix, %d planets"`.
