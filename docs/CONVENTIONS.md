# bmech Conventions — Single Source of Truth

This is the entry point for understanding how `mechanisms_core/` is
organized and documented. Read this first, then follow the links into
whichever family you're touching.

## Doc structure: family → primitive

Documentation for each part family lives under `docs/<family>/`, split two
ways:

- **`docs/<family>/README.md`** — everything true for the whole family:
  shared parameters, shared math/constants, shared validation behavior,
  shared implementation patterns (e.g. how booleans are built, how FDM
  compensation is applied). If a convention would otherwise be repeated
  identically across every primitive doc in the family, it belongs here
  instead.
- **`docs/<family>/<primitive>.md`** — one file per generator operator.
  Only what's specific to that primitive: its own properties, its own
  panel warnings, its own build method, its own stamped output. Every
  primitive doc assumes you've already read the family README and won't
  re-explain family-wide conventions.

This split exists because the gear family alone has 13 generators sharing
a large, easy-to-drift set of conventions (module/pressure-angle
semantics, the `bmech_*` stamping and Match Target system, FDM bore
compensation direction, boolean-safety patterns). Writing those once in a
family README and linking to them from every primitive doc is what keeps
them from being re-typed — and re-drifting — 13 times over.

**When adding a new family**, follow the same shape: a family README
covering what's shared, one file per primitive covering what's not, and
an entry added to the index below.

## Documentation is not optional — write it alongside the code

**Every time a new mechanism generator is added to `mechanisms_core/`
(a new operator, in an existing family or a brand new one), write its
documentation in the same change** — not as separate follow-up work.
Concretely, that means:

- **New primitive in an existing family**: add
  `docs/<family>/<primitive>.md` following the shape of the other files
  in that folder (properties table, build method, panel warnings, output/
  stamping). Update that family's `README.md` if the new primitive
  introduces a convention shared with — or that breaks an assumption
  held by — the rest of the family (a new hand/direction convention, a
  new validation pattern, a new compensation field, etc.). Add a row to
  that family's primitive table if one exists.
- **New family**: create `docs/<family>/README.md` plus one
  `docs/<family>/<primitive>.md` per operator it ships with, and add a row
  to the Families table below with status `Documented`.
- **Editing an existing generator's properties, validation, or build
  method**: update its doc in the same change. A stale primitive doc
  (documenting properties/behavior that no longer exist) is worse than no
  doc, since it actively misleads.

This mirrors the pattern this project already uses for graduating
prototypes — see the repo `README.md`'s Development section — except the
doc now graduates with the code, not after it. If you're an AI agent
working in this repo: treat "add docs for the new mechanism" as an
implicit, unstated part of any task that adds or changes a generator
operator, the same way you'd treat "keep the tests passing."

## Families

| Family | Status | Entry point |
|---|---|---|
| Gears | Documented | [docs/gears/README.md](gears/README.md) |
| Fasteners (`fasteners/threaded_fastener.py`, `fasteners/hex_bolt.py`, `fasteners/hex_nut.py`, `fasteners/press_fit_pin.py`) | Documented | [docs/fasteners/README.md](fasteners/README.md) |
| Bearings (`bearings/ball_bearing.py`) | Documented | [docs/bearings/README.md](bearings/README.md) |
| Springs (`springs/hairspring.py`, `springs/serpentine_spring.py`) | Documented | [docs/springs/README.md](springs/README.md) |
| Ratchets (`ratchets/ratchet_pawl.py`, `ratchets/internal_ratchet.py`) | Documented | [docs/ratchets/README.md](ratchets/README.md) |

## Conventions that apply across every family (not just gears)

These were established while documenting gears but aren't gear-specific —
carry them forward into every future family doc:

- **All dimensions are millimeters.**
- **FDM compensation is always additive**, on both external and internal
  features — never subtract compensation from a nominal dimension, even
  for a feature that's expected to print oversized. This project's prints
  shrink, so every compensation value in this codebase widens a hole or
  otherwise adds material, never removes it. (See
  [gears/README.md#bore-holes-and-fdm-compensation](gears/README.md#bore-holes-and-fdm-compensation)
  for the gear-family instance of this rule.)
- **Nominal size is always the outer/major dimension**, with any
  inner/minor dimension derived from it — never the reverse. (Established
  for thread nomenclature: nominal size = major diameter for both bolt and
  nut, minor diameter is always derived.)
- **Validation severity is two-tier**: a hard geometric impossibility
  cancels the operator with an ERROR-icon panel label (and usually, but
  not always, an explicit `self.report({'ERROR'}, ...)`); a soft
  engineering guideline being violated is a WARNING that still builds the
  geometry. Don't assume every ERROR-icon label is blocking — check
  whether `execute()` actually returns `CANCELLED` for that specific
  condition, since some primitives use the ERROR icon for non-blocking
  warnings too (see
  [gears/bevel_gear.md](gears/bevel_gear.md#panel-warnings) for a primitive
  with both kinds).
- **Boolean cutters get deleted after use.** Every generator that cuts a
  feature with a boolean modifier builds a temporary cutter object,
  applies the modifier, and deletes the cutter — never leaves a live
  boolean modifier or an orphaned cutter object in the scene.
- **A "compensation" field and a "clearance" field are not the same
  thing, even though both exist to make FDM prints fit together.**
  Compensation (`*_compensation_mm`) always widens a single hole/feature
  to counteract print shrinkage — always added, one direction. Clearance
  (`pip_gap`, `gap_mm`, `clearance_mm`, depending on the family) reserves
  a running gap between two independently-moving or pre-meshed printed
  surfaces, and is sometimes subtracted from a maximum allowed size
  rather than added to a hole. Don't genericize these into one concept
  when writing a new primitive doc — check which one a given property
  actually is before describing it. (See
  [gears/planetary_gear_set.md#pip_gap--not-the-same-thing-as-bore-compensation](gears/planetary_gear_set.md#pip_gap--not-the-same-thing-as-bore-compensation)
  and
  [ratchets/README.md#fdm-compensation-vs-running-clearance--two-different-things-dont-conflate-them](ratchets/README.md#fdm-compensation-vs-running-clearance--two-different-things-dont-conflate-them).)

## Two different philosophies for sharing code between primitives — both are intentional

The gear family centralizes shared logic in one helper module
(`gears/gear_matching.py`) that every gear primitive imports from. The
fastener and ratchet families do the opposite: every generator module is
fully self-contained, and shared math (thread profile generation, ratchet
tooth profiles) is **duplicated verbatim** between files rather than
imported. `hex_nut.py`'s source states the fastener family's reasoning
directly: "each generator module is self-contained, no cross-file thread
math imports." `internal_ratchet.py` gives the matching rationale for
ratchets: its copied helpers exist "so this module has no inter-module
dependency."

Both are deliberate, not inconsistency to clean up. Before adding a new
primitive, check which convention the rest of its family already follows
— see [gears/README.md#the-match-target-system-gear_matchingpy](gears/README.md#the-match-target-system-gear_matchingpy)
for the shared-module approach and
[fasteners/README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate](fasteners/README.md#this-family-does-not-share-a-helper-module--and-thats-deliberate)
for the duplicate-on-purpose approach.
