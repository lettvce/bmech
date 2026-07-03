# Gear Family — Shared Conventions

This document covers everything that's true for *every* gear generator in
`mechanisms_core/`. Primitive-specific docs (one file per generator, in this
same folder) only cover what's different about that primitive — they assume
you've read this first.

All 13 gear primitives ship as separate operators, one Blender object (or,
for gear sets, several) per call, appearing under `Shift-A > Mechanisms > Gears`.

## Primitives in this family

`mechanisms_core/gears/` splits into four subfamilies, matching the
`Mechanisms > Gears` menu structure in `menu.py`: `external/`, `ring/`,
`planetary/`, `bevel/`. `gear_matching.py` (the Match Target system, see
below) lives at `gears/` root since it's shared by every subfamily.

| Primitive | File | kind stamp |
|---|---|---|
| Spur gear | `gears/external/involute_gear_rack.py` | `spur` |
| Rack | `gears/external/involute_gear_rack.py` | `rack` |
| Helical gear | `gears/external/helical_gear.py` | `helical` |
| Herringbone gear | `gears/external/herringbone_gear.py` | `herringbone` |
| Bevel gear | `gears/bevel/bevel_gear.py` | `bevel` |
| Annulus (ring) gear | `gears/ring/annulus_gear.py` | `annulus` |
| Helical annulus gear | `gears/ring/helical_annulus_gear.py` | `helical_annulus` |
| Herringbone annulus gear | `gears/ring/herringbone_annulus_gear.py` | `herringbone_annulus` |
| Planetary gear set | `gears/planetary/planetary_gear_set.py` | *(none — see below)* |
| Helical planetary gear set | `gears/planetary/helical_planetary_gear_set.py` | *(none)* |
| Herringbone planetary gear set | `gears/planetary/herringbone_planetary_gear_set.py` | *(none)* |
| Cluster gear | `gears/external/cluster_gear.py` | *(none)* |
| Compound gear train | `gears/external/compound_gear.py` | *(none)* |

The single-gear and ring-gear primitives (top 8 rows) are building blocks
meant to mesh with each other and participate in the Match Target system
below. The gear-set primitives (bottom 5 rows) are self-contained assemblies
— they build several already-meshed gears in one call and don't stamp or
read `bmech_*` properties themselves.

## Units and core parameters

Everything is millimeters. Every single-gear primitive shares two core
parameters:

- **`module`** (mm) — sets tooth size. Two gears only mesh if their modules
  match. For helical/herringbone gears, `module` is the **transverse**
  module (measured in the cross-section view); the normal module (what a
  real hob/cutter would be specified as) is `module * cos(helix_angle)` and
  is shown as a read-only info line in the operator panel.
- **`pressure_angle_deg`** — standard range 10–45°, default 20°. Must match
  between meshing gears. Larger pressure angles give stronger, wider-based
  teeth but need more radial clearance.

Tooth proportions are standard throughout the family:

```
ADDENDUM_COEFF = 1.0    # tooth height above pitch circle, in units of module
DEDENDUM_COEFF = 1.25   # tooth depth below pitch circle, in units of module
```

so for an external gear, `addendum_radius = pitch_radius + module` and
`dedendum_radius = pitch_radius - 1.25*module`, where
`pitch_radius = module * tooth_count / 2`.

**Annulus (internal/ring) gears swap these roles.** The cutter that carves
inward-pointing teeth into a ring is built with the same profile-builder
code as an external tooth, but `ADDENDUM_COEFF` and `DEDENDUM_COEFF` swap
which one is "outward" and which is "inward" — so for a ring gear,
`tip_r = pitch_r - module` (teeth point inward) and
`root_r_inner = pitch_r + 1.25*module` (tooth base meets the solid ring
body). See [annulus_gear.md](annulus_gear.md) for the full derivation.

## Hand convention — the one rule most likely to bite you

Helical and herringbone gears have a `hand` property (`RIGHT`/`LEFT`).
The rule for which hand a mating gear needs depends on whether the mesh is
external-external or external-internal:

- **External ↔ external** (two spur/helical/herringbone gears meshing on
  parallel shafts, or a helical rack): mating gears need **OPPOSITE**
  hands. This is `gear_matching.sync_helical_opposite`.
- **External ↔ internal** (a helical/herringbone pinion inside a helical/
  herringbone annulus/ring gear): mating gears need the **SAME** hand.
  This is `gear_matching.sync_helical_same` — and it's the opposite of
  the external-external rule, which is exactly the mistake this convention
  exists to prevent.

Herringbone gears specifically have *no net axial thrust* regardless of
hand (that's the point of the herringbone V-shape) — but hand still has to
match per the rules above, because it determines which way the V opens,
not just which way it twists.

Straight (non-helical) gears — spur, rack, bevel, straight annulus — have
no `hand` property at all.

## The Match Target system (`gear_matching.py`)

Every gear operator that finishes successfully stamps custom properties
(`bmech_kind`, `bmech_module`, `bmech_pressure_angle_deg`, and where
applicable `bmech_tooth_count`, `bmech_helix_angle_deg`, `bmech_hand`) onto
the object it creates, via `gear_matching.stamp_gear(...)`.

Every gear operator also exposes a **Match Target** object picker
(`target`, a `PointerProperty`). Its poll function
(`gear_matching.gear_target_poll`) accepts any mesh object carrying a
`bmech_module` property — it does **not** check `bmech_kind`, so nothing
stops you from picking a bevel gear as the target for a rack. Meshing
correctness is on you; the system only saves you from retyping numbers.

Picking a target runs one of four sync helpers, chosen by what kind of
pair the operator represents:

| Helper | Copies | Used by |
|---|---|---|
| `sync_module_pa` | module, pressure angle | spur, rack, straight annulus |
| `sync_helical_opposite` | + helix angle, **inverted** hand | helical/herringbone external gears |
| `sync_helical_same` | + helix angle, **same** hand | helical/herringbone annulus gears |
| `sync_bevel` | module, pressure angle, target's tooth count → this gear's `mate_teeth` | bevel gears |

Gear-set primitives (planetary/cluster/compound) don't participate in this
system at all — they build their own internally-consistent meshes in one
shot and never call `stamp_gear` or read a `target`.

## Bore holes and FDM compensation

Most single-gear primitives (not bevel, not gear-set shafts) expose
`bore_enable` / `bore_diameter` / `bore_compensation`. **Compensation is
always added to the bore radius, never subtracted** — printed holes come
out undersized, and this project's convention is that all FDM dimensional
compensation (external or internal) is additive, matching every other
generator in this library. Default compensation is `0.2mm`.

```
bore_r = bore_diameter / 2 + bore_compensation
```

The operator silently skips cutting the bore (not clamps it) if
`bore_r >= dedendum_radius` — check the panel's error label rather than
trusting the mesh will just come out with a smaller hole.

Bevel gears have **no bore feature at all**: their center-fan end-cap
construction already fills the hub area, so there's nothing for a boolean
cutter to remove without also removing tooth material. Don't expect a
`bore_diameter` field on that operator.

Planetary/cluster/compound gear-set primitives use a related but distinct
parameter, **`pip_gap`** — radial clearance added at tooth flanks so
gears printed pre-meshed (print-in-place) don't fuse together. This is
backlash clearance between teeth, not a bore-hole allowance; don't confuse
the two.

## Boolean/geometry implementation notes

These matter if you're reading or modifying the generator code, less so if
you're just using the operators:

- Internal-tooth features (annulus bores, planetary ring gears) are always
  cut with a **boolean DIFFERENCE modifier, `solver='EXACT'`**, applied
  immediately and the cutter object deleted — never left as a live
  modifier.
- Cutter meshes are extruded `BOOL_EPSILON = 0.001mm` past the body's end
  faces on both sides, specifically so the cutter's cap faces are never
  exactly coplanar with the body's cap faces (coplanar faces are a common
  cause of EXACT-solver boolean failures).
- Concave/star-shaped end caps (annulus cross-sections, cluster-gear
  shoulders) are triangulated with a **center-fan** from one added center
  vertex, not `bmesh.ops.triangle_fill`. Triangle-fill can produce
  self-intersecting triangles on a concave polygon that Blender's CGAL
  EXACT solver then rejects; a fan from a single interior point can't
  self-intersect.
- Helical/herringbone twist is built by slicing the extrusion into
  `n_slices` (or `n_slices` per half, for herringbone) Z-layers and
  rotating each slice by `hand_sign * z * tan(helix_angle) / pitch_radius`,
  where `hand_sign` is `+1` for `RIGHT` and `-1` for `LEFT`. This is the
  entire hand implementation — there's no separate mirroring step.
- `unique_name("Gear")` pre-computes a Blender-style `.001`/`.002` suffix
  before object creation, because the addon needs the final name up front
  for scene bookkeeping rather than relying on Blender's own on-creation
  renaming.

## Validation conventions

- A hard geometric impossibility (dedendum/tip radius ≤ 0, bevel face
  width ≥ cone distance) is a **CANCELLED** operator with an ERROR-icon
  label in the panel; most of these cancel silently without an
  `self.report()` call, so check the panel, not just the status bar.
- A soft engineering guideline being violated (bevel `face_width_mm >
  cone_dist/3`, planetary tooth-count/planet-count not dividing evenly) is
  a **WARNING** — the operator still runs and still reports via
  `self.report({'WARNING'}, ...)`, but the geometry is built anyway. Read
  the panel warnings; they're not blocking, but they're not decorative
  either.

## Naming, units, and defaults at a glance

- Default `pressure_angle_deg` is **20.0°** everywhere.
- Default `module` is **2.0mm** for helical/herringbone/bevel/annulus
  families, **1.0mm** for spur/rack/cluster/compound.
- Minimum tooth count is **5** for most external gears, **8** for bevel
  gears (keeps cone-angle math and small-end teeth sane) and for annulus
  gears (`min=8` on `tooth_count`).
- Gear-set primitives (planetary/cluster/compound) create **multiple
  objects per operator call** — see each primitive's doc for the exact
  count and naming scheme.
