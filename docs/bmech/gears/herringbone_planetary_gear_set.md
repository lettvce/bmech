# Herringbone Planetary Gear Set

`gears/planetary/herringbone_planetary_gear_set.py` → `OBJECT_OT_herringbone_planetary_gear_set` (`object.herringbone_planetary_gear_set`, "Herringbone Planetary Gear Set")

Same topology and assembly math as
[planetary_gear_set.md](planetary_gear_set.md), same hand-derivation rule
as [helical_planetary_gear_set.md](helical_planetary_gear_set.md) — read
both first. This document only covers what's different: V-shaped teeth on
all three members.

## Properties

Same as the spur planetary set, plus:

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `helix_angle_deg` | Float (°) | 20.0 | 1–45 | Half-angle of the V, applies to all three members |
| `hand` (labeled "Sun Hand") | Enum | `RIGHT` | `RIGHT` / `LEFT` | Sets the sun's hand; planet/ring derived (`R→L→L` / `L→R→R`) |
| `width_mm` (labeled "Total Width") | Float (mm) | 14.0 | 2–80 (soft) | Full face width — each half is `width_mm/2`, shared by all three members |
| `n_slices` (labeled "Slices per Half") | Int | 12 | 2–48 (soft) | Total slices built per member = `2*n_slices - 1` |

`sun_teeth`, `planet_teeth`, `planet_count`, `module`,
`pressure_angle_deg`, `ring_wall_mm`, `pip_gap`, `outer_segs` are
identical to the spur set.

## What's identical to the other two planetary sets

The tooth-count rule (`ring = sun + 2*planet`), the assembly condition
(`(sun+ring) % planet_count == 0`, warning-only), the planet orbital-phase
formula, the ring's `-pi/ring_teeth` phase correction, and the hand
derivation (sun chosen, planet = opposite, ring = same as planet) are all
**identical** to the spur and helical planetary sets — only the mesh
builders differ. If you already understand those two docs, the only new
thing here is the V-shaped tooth geometry itself.

## Build method

Sun and planet: each built as two mirrored helical halves sharing a
mid-slice, same approach as a standalone herringbone gear
(`herringbone_gear.py`) — bottom half twist `0 → peak`, top half
`peak → 0`, `2*n_slices - 1` total slices, `peak_twist = (width_mm/2) *
tan(helix_angle) / pitch_radius` (computed per-member using that member's
own pitch radius). Ring: solid cylinder minus a herringbone-twisted
annulus cutter (same approach as
[herringbone_annulus_gear.md](herringbone_annulus_gear.md)).

Info box shows normal module, the **sun's** peak twist in degrees, and
total slice count (`2*n_slices - 1`).

## Output

**`2 + planet_count` objects per call**: `HbPlanetaryRing`,
`HbPlanetarySun`, and `HbPlanetaryPlanet.001`…`.00N` (linked-mesh
copies). Success message:
`"Herringbone planetary: %d/%d/%d teeth (sun/planet/ring), %.1f° helix, %d planets"`.
