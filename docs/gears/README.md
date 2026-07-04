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

Every gear operator also exposes a **Match Target** object picker, drawn
via `layout.prop(context.window_manager, "bmech_gear_target", ...)`. This
lives on `WindowManager`, **not** as a `PointerProperty` on the operator
itself — `bpy.types.Operator` can't hold a `PointerProperty` to an
`Object` (Blender silently drops that property and everything declared
after it in the class on registration), which is exactly the mistake this
system exists to avoid repeating. Each operator instead implements
`bmech_sync_target(self, context)`, and a shared `update` callback on the
WindowManager property (`gear_matching._on_target_change`) dispatches to
whichever operator `context.active_operator` currently points at. See
[CONVENTIONS.md#the-match-target-pattern--letting-one-parts-output-drive-anothers-properties](../CONVENTIONS.md#the-match-target-pattern--letting-one-parts-output-drive-anothers-properties)
for the full list of Blender-API gotchas this pattern runs into — that
section is written to generalize to any future family that wants the same
kind of cross-object matching, not just gears.

The poll function (`gear_matching.gear_target_poll`) accepts any mesh
object carrying a `bmech_module` property — it does **not** check
`bmech_kind`, so nothing stops you from picking a bevel gear as the target
for a rack. This is deliberate, not an oversight: cross-kind matches are
sometimes mechanically real (a helical gear meshing with one half of a
herringbone gear, both sharing module/pressure-angle/helix/hand) and
sometimes represent a shared-hub relationship rather than a direct mesh
(see the spur-gear case below) — the system only saves you from retyping
numbers, meshing correctness is on you either way.

Picking a target runs one of four sync helpers, chosen by what kind of
pair the operator represents:

| Helper | Copies | Used by |
|---|---|---|
| `sync_module_pa` | module, pressure angle | spur, rack, straight annulus |
| `sync_helical_opposite` | + helix angle, **inverted** hand | helical/herringbone external gears |
| `sync_helical_same` | + helix angle, **same** hand | helical/herringbone annulus gears |
| `sync_bevel` | module, pressure angle, target's tooth count → this gear's `mate_teeth` | bevel gears |

**A spur-gear target never drives helix angle or hand**, even when the
gear being created is helical or herringbone. `stamp_gear` never writes
`bmech_helix_angle_deg`/`bmech_hand` for a `"spur"` kind, so
`sync_helical_opposite`'s `if "bmech_helix_angle_deg" in target.keys()`
guards simply skip for a spur target — only module and pressure angle get
pulled in. This isn't a gap to fix: a plain spur gear and a helical gear
can't correctly mesh on a standard parallel-shaft arrangement regardless
of matched module/pressure-angle (the tooth angles don't align), so
matching a helical/herringbone gear to a spur target represents "shares a
pitch circle" (e.g. a compound gear with a spur section on the same hub),
not "these teeth mesh directly" — helix angle and hand are the twisted
section's own properties to set, not something a non-twisted target could
meaningfully specify. `helical_gear.py`/`herringbone_gear.py`'s `draw()`
reflects this directly: `module`/`pressure_angle_deg` are frozen whenever
any target is set, but `helix_angle_deg`/`hand` are only frozen when the
target actually has `bmech_helix_angle_deg` — i.e. when it's a
helical/herringbone target, not a spur one.

**Even when a helical/herringbone target *does* drive `helix_angle_deg`,
`hand` specifically stays editable if the two sides are different
styles** (one plain helical, one herringbone) — `gear_matching.
hand_target_ambiguous(self_is_herringbone, target)`. A herringbone tooth
is two mirrored helical halves (bottom twists one way, top the other); a
plain helical tooth is a single constant-angle line. A plain helical gear
can only mesh correctly against **one** half of a herringbone gear at a
time, and which hand is correct depends on which half — the sync can only
assume one convention (matching the target's bottom half, per
`herringbone_gear.py`'s own `hand` property being defined relative to its
bottom half) as a default, so it's wrong to lock the field when the
design might actually engage the top half instead. This is NOT the same
condition as the spur-target case above — `helix_angle_deg`'s *magnitude*
stays frozen in the cross-style case (it's unambiguous regardless of
which half is engaged), only `hand` reopens. Same-style pairs (helical↔
helical, herringbone↔herringbone) have no such ambiguity — a herringbone
meshes a herringbone across its full width by design — and freeze `hand`
normally, including across the external/annulus boundary (an external
helical pinion matching a helical annulus target is still same-style, so
`hand` stays frozen there too).

Gear-set primitives (planetary/cluster/compound) don't participate in this
system at all — they build their own internally-consistent meshes in one
shot and never call `stamp_gear` or read the target.

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

- Internal-tooth features are cut with a **boolean DIFFERENCE modifier,
  `solver='EXACT'`**, applied immediately and the cutter object deleted —
  never left as a live modifier — **except all three annulus generators**
  (`annulus_gear.py`, `helical_annulus_gear.py`,
  `herringbone_annulus_gear.py`), which were rewritten to skip the boolean
  entirely (see below). Planetary ring gears
  (`herringbone_planetary_gear_set.py`'s own ring cutter, and its plain/
  helical siblings) still use the boolean approach.
- Cutter meshes (where a boolean cutter is still used) are extruded
  `BOOL_EPSILON = 0.001mm` past the body's end faces on both sides,
  specifically so the cutter's cap faces are never exactly coplanar with
  the body's cap faces (coplanar faces are a common cause of EXACT-solver
  boolean failures).
- **All three annulus generators were rewritten to skip the boolean
  entirely** — direct bmesh construction (inner toothed wall, outer
  cylindrical wall, two caps), no cutter object, no `EXACT` solver call.
  Measured 10-180x faster across the three (295ms-5.8s boolean-based vs
  3.7-208ms direct, for tooth counts 8-100) — the annulus family was the
  slowest in the whole gear family before this change. The two caps are
  the interesting part: they're built with `bmesh.ops.triangle_fill` fed
  the boundary edges of **both** the inner (toothed) loop and the outer
  (circular) loop together in one call, which triangulates the annular
  region between them directly (treating the inner loop as a hole),
  rather than hand-bridging the two loops with an index-matched strip of
  quads. A matched-angle bridge looks like the obvious approach (one outer
  point per inner-profile point, at the same angle) but doesn't work: the
  tooth profile inserts a dedendum-circle point at the same angle as its
  neighbor wherever `base_r > ded_r` (a genuine straight undercut-flank
  segment, not a bug), and bridging to a same-angle outer point puts three
  points on the same ray through the origin — an unavoidably zero-area
  triangle regardless of how the resulting quad gets split. Confirmed by
  testing: a matched-angle outer ring reintroduces non-manifold edges and
  zero-area faces at low tooth counts (8/15/20); treating the outer ring
  as a fully independent, separately-spaced circle and letting
  `triangle_fill` reconcile the two loops does not. For the two twisted
  variants, the outer ring additionally stays PLAIN and UNTWISTED (only 2
  Z-layers) — only the inner teeth twist. See
  [annulus_gear.md](annulus_gear.md#build-method) for the full writeup and
  [helical_annulus_gear.md](helical_annulus_gear.md#build-method) /
  [herringbone_annulus_gear.md](herringbone_annulus_gear.md#build-method)
  for the twisted extensions.
- All three annulus generators' pressure-angle clamps carry an extra
  `PA_TRIANGLE_FILL_MARGIN_DEG` (0.2°) below the theoretical
  self-intersection limit, not the limit itself. Right at that limit the
  tooth tip's flank points become near-coincident — the old boolean-based
  `EXACT` solver tolerated a profile sitting exactly there, but
  `triangle_fill`'s constrained triangulation does not, and produced
  hundreds of non-manifold edges without the margin.
- Where a boolean cutter is STILL used (planetary rings), concave/
  star-shaped end **caps on that cutter** use `bmesh.ops.triangle_fill` on
  its own single boundary loop, not a center-fan. This used to be
  reversed — center-fan from an added interior vertex, on the theory that
  triangle-fill could produce self-intersecting triangles on a concave
  polygon that the CGAL EXACT solver would then reject. That theory didn't
  hold up: `triangle_fill` was tested directly against these exact
  profiles (tooth counts 8 through 100) and produced 0 self-intersections,
  while the center-fan approach had a real, confirmed defect the fan
  reasoning missed — the same collinear dedendum-circle point described
  above degenerates a fan triangle to exactly zero area. That looked
  harmless (a zero-area triangle covers no area) but wasn't: its short
  edge is shared with a side-wall face, so the mesh depends on that face
  existing to stay edge-manifold, even though it contributes nothing to
  visible volume. Dropping it outright (an intermediate fix, since
  discarded) turned the edge into a non-manifold boundary instead —
  confirmed to produce hundreds of non-manifold edges on some tooth
  counts. Cluster-gear shoulders and bevel gears still use the center-fan
  approach — untouched, since they weren't confirmed to hit the same
  degenerate-collinear-point condition, and bevel gears in particular have
  no boolean run against their own mesh, so a degenerate cap face there is
  cosmetic at worst, not a build failure.
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
