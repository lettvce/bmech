# Planetary Gear Set

`gears/planetary/planetary_gear_set.py` → `OBJECT_OT_planetary_gear_set` (`object.planetary_gear_set`, "Planetary Gear Set")

Builds a complete, correctly-meshed epicyclic gear set — sun, N planets,
and ring — in one operator call. Straight (spur) teeth. See
[README.md](README.md) for family-wide conventions; this and the two other
gear-set primitives are **self-contained** and don't use the Match Target
/ `stamp_gear` system at all (there's nothing external to mesh with — the
set is already fully meshed internally).

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `sun_teeth` | Int | 12 | 4–100 (soft) | |
| `planet_teeth` | Int | 16 | 4–100 (soft) | Ring tooth count is derived: `ring = sun + 2*planet` |
| `planet_count` | Int | 3 | 2–8 | `(sun + ring)` must be divisible by this |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Shared by all three members |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `width_mm` | Float (mm) | 10.0 | 1–80 (soft) | Shared face width |
| `ring_wall_mm` | Float (mm) | 5.0 | 0.5–30 (soft) | Ring gear's radial wall beyond the tooth root |
| `pip_gap` | Float (mm) | 0.2 | 0.0–2.0 (soft) | Radial clearance at tooth tips for print-in-place |
| `outer_segs` | Int | 64 | 16–256 (soft) | Facets on the ring's outer surface |

## The two governing equations

```
ring_teeth = sun_teeth + 2 * planet_teeth                # exact, not optional
(sun_teeth + ring_teeth) % planet_count == 0              # even spacing
```

The first is geometric fact — three meshed gears on those center distances
have no other valid ring size. The second is the classic planetary
assembly condition: if it doesn't hold, the planets can't all be
simultaneously in mesh with both sun and ring at evenly-spaced angular
positions.

Violating the second condition is a **warning, not a block** — the panel
shows an ERROR-icon label
(`"(%d + %d) / %d not integer — planets won't space evenly"`), and
`execute()` calls `self.report({'WARNING'}, ...)`, but still builds the
geometry. Take the warning seriously; the set will look assembled but the
planets will not actually be correctly phased.

## Planet placement

```
angle_step = 2*pi / planet_count
theta_i    = i * angle_step                                    # i = 0..planet_count-1
position   = center_dist * (cos(theta_i), sin(theta_i))        # center_dist = r_sun + r_planet
rotation_i = -theta_i * (sun_teeth/planet_teeth) + pi/planet_teeth
```

The rotation term is what keeps every planet correctly meshed with the sun
regardless of where around the circle it sits — each planet's own spin is
tied to its orbital angle by the sun/planet tooth ratio.

The ring is then given a fixed phase correction,
`rotation_euler.z = -pi / ring_teeth`, independent of planet count: the
boolean cutter used to carve the ring's teeth has slot centers at
`k * 2*pi/ring_teeth`, putting ring teeth at `(2k+1) * pi/ring_teeth`;
`-pi/ring_teeth` is exactly the offset needed to land a ring tooth at each
planet's mesh valley.

## `pip_gap` — not the same thing as bore compensation

`pip_gap` adds radial clearance at tooth flanks so a planetary set printed
fully pre-assembled (print-in-place) doesn't fuse its own meshing teeth
together. It's implemented as two different geometric effects depending on
which side it's applied to: it **thins** an external gear's tooth (sun,
planet), and **widens** the annulus cutter's slot (ring) — same parameter,
opposite direction, because both need to open up the same physical gap.
This is unrelated to `bore_compensation` on other primitives, which deals
with hole tolerance, not tooth backlash.

Note this generator does **not** nest sun/planet/ring into a single
print-in-place assembly the way `pip_gap`'s name might suggest for other
mechanisms in this library (see the ratchet/hinge generators for that
pattern) — sun, planets, and ring are always separate objects; `pip_gap`
only controls backlash between them if you do print them pre-assembled.

## Build method

Sun and planets are built directly via bmesh extrusion (flat profile,
straight extrude — no boolean, no twist). The ring is a solid cylinder
(`outer_r = r_ring + 1.25*module + ring_wall_mm`) with an annulus cutter
boolean-subtracted (`EXACT` solver), same pattern as
[annulus_gear.md](annulus_gear.md). All `planet_count` planets are linked
copies of a single shared mesh datablock — editing one planet's mesh in
Edit Mode will edit all of them.

## Output

**`2 + planet_count` objects per call** (5 by default): `PlanetaryRing`,
`PlanetarySun`, and `PlanetaryPlanet.001`…`PlanetaryPlanet.00N`. The ring
is the active object after creation. Success message:
`"Planetary: %d/%d/%d teeth (sun/planet/ring), module %.1f, %d planets"`.

No `hand`, no helix — spur teeth only. For twisted-tooth versions, see
[helical_planetary_gear_set.md](helical_planetary_gear_set.md) and
[herringbone_planetary_gear_set.md](herringbone_planetary_gear_set.md),
which share every formula in this document except the mesh builders
themselves.
