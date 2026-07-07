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
- **Prefer clamping over cancelling whenever a natural "closest valid
  value" exists — don't default to a `self.report({'ERROR'})` +
  `CANCELLED` banner just because a combination is currently invalid.**
  `gear_matching.clamp_pressure_angle` is the original instance of this:
  rather than blocking the operator when `pressure_angle_deg` would
  self-intersect the tooth profile for the current `tooth_count`, it
  silently mutates `pressure_angle_deg` down to the exact self-intersection
  limit and lets the build proceed — the redo panel shows the corrected
  value instead of a red error banner. `hex_bolt.py`'s `tip_diameter_mm`
  (clamped to just under the thread's minor radius) and
  `threaded_container.py`/`threaded_lid.py`'s own `_clamp()` methods
  (wall thickness, truncation, and thread length clamped against each
  other in the container; truncation, guide length, and thread length
  clamped independently in the lid, since its additive thread no longer
  shares a fixed height budget with the other properties) follow the
  identical pattern.
  Concretely:
    - Write the clamp as its own method (or a shared helper, if the
      family centralizes logic — gears put theirs in `gear_matching.py`;
      fasteners duplicate a bespoke `_clamp()`/`_min_*_for_*()` pair per
      file, per that family's own convention), called once at the very
      top of `execute()`, **before** `_derived()` or any mesh building —
      mutate `self.<property>` directly so the value the user sees in the
      redo panel is the corrected one, not the one they typed.
    - `draw()`'s existing "is this combination invalid" checks don't go
      away — keep them, but they become defensive/informational rather
      than the primary gate, since `_clamp()` should mean they rarely if
      ever actually fire once wired up. Downgrading their icon from
      `'ERROR'` to `'INFO'` (still shown at the bottom of the panel, after
      the derived-values box) signals that distinction to a future reader:
      an `INFO` label here means "this used to be possible, `_clamp()`
      should have already fixed it" as opposed to a live blocking gate.
    - **Still cancel for genuinely irreconcilable combinations** — cases
      with no meaningful single "closest valid value" to fall back to
      (`bevel_gear.py`'s `face_width_mm >= cone_dist` is this family's
      original example: there's no sensible auto-shrink target, the
      geometry is just asked to be physically impossible). Have the clamp
      method return a message (or `None`) rather than always succeeding,
      and only `CANCELLED` + `self.report({'ERROR'}, message)` in that
      genuinely-unfixable case.
    - **When a clamp depends on multiple properties interacting, solve
      for the one property whose role is "how much this construction
      technique compromises," not the one that's the user's deliberate
      size choice.** `threaded_container.py`/`threaded_lid.py` clamp
      `truncation` (not `wall_thickness_mm` or `thread_diameter_mm`) to
      fix a thread that's too deep for the available material, since
      `truncation` specifically controls how much a thread's crest/root
      flats blunt its depth — the size properties are what the user
      actually came to set.
    - **Verify a clamp helper doesn't clamp its own return value before
      the caller gets to check it against the boundary that would make
      the case irreconcilable.** Confirmed real bug in this project: a
      first version of `_min_truncation_for_max_depth` clamped its result
      to `[0, 0.3]` (truncation's own valid range) internally, which made
      the caller's `if min_trunc > 0.3: return "irreconcilable"` check
      impossible to ever trigger — the value being compared had already
      been silently capped at exactly the boundary it was being tested
      against. Return the raw, unclamped solve from a helper like this;
      let the caller do the boundary check, then clamp for actual use.
    - **A margin picked for one constraint doesn't automatically work for
      a different one, even in the same file.** `threaded_lid.py` reused
      a 0.1mm margin (already validated for its wall-thickness clamp) for
      an unrelated thread-length clamp, and a parameter sweep dedicated to
      exactly that margin (0.1/0.2/0.5/1.0/1.5/2.0/3.0mm) found it
      reintroduced a real mesh defect below ~1mm — a razor-thin
      clearance-zone wall segment is numerically fragile for the EXACT
      solver independent of any other fix already in place. Test the
      actual margin a new clamp needs; don't assume one that worked
      elsewhere transfers.
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

## The "Match Target" pattern — letting one part's output drive another's properties

Gears are the only family that has this today (`gears/gear_matching.py`),
but the underlying technique isn't gear-specific — anything where one
generated part's dimensions should drive another's (a bolt matched to a
nut's thread, a bearing matched to a shaft's OD) would hit the same set of
Blender API gotchas. If you build this for another family, these are the
things that actually bit us getting it right for gears, not obvious from
reading the finished code:

- **The picker has to live on `WindowManager`, not the operator.**
  `bpy.types.Operator` cannot hold a `PointerProperty` to an `Object` —
  Blender silently drops that property **and everything declared after it
  in the class** on registration. No error, no warning; properties just
  stop showing up in the panel. Put the picker on `WindowManager` instead,
  and have each participating operator implement its own
  `bmech_sync_target(context)` method that a single shared `update`
  callback (via `context.active_operator`) dispatches to.
- **Freeze only what the sync actually drives, and only for target kinds
  that drive it** — not blanket-freeze every matchable property whenever
  any target is picked. Check whether *this specific* target carries the
  custom-property keys your sync function reads before disabling the
  corresponding field in `draw()`. A target that doesn't stamp a given
  property should leave the matching field editable, since the sync
  function will never touch it for that target and the user still needs
  to set it by hand.
- **Reset the picker in `invoke()`, never in `execute()`.** A
  `WindowManager` property persists across unrelated operator
  invocations, so a target picked for one part would silently carry over
  to the next, unrelated part unless you clear it. But `execute()` also
  re-runs on every redo-panel property tweak — resetting there wipes out
  the user's own selection the moment they touch any other field.
  `invoke()` only fires once, on a genuinely fresh creation, so it's the
  only place to clear it.
- **Treat a reset-to-`None` as a no-op in the update callback — don't let
  it trigger a sync or rebuild.** This one caused a real, confirmed bug:
  clearing the picker back to empty (from the `invoke()` reset above) is
  itself a value change, and fires the same `update` callback a genuine
  pick does. If that callback doesn't special-case `None`, it tries to
  sync/rebuild using whatever `context.active_operator` still points at —
  which, at that exact moment, is the *previous* part, not the new one
  being created. The result was the previous part getting deleted and
  rebuilt at wherever the 3D cursor happened to be by then. Bail out
  immediately when the new value is `None`; there's nothing meaningful to
  sync from an empty target regardless of why it became empty.
- **Rebuilding the object live when a target is picked needs to avoid
  `bpy.ops.ed.undo()`.** Picking a target doesn't trigger Blender's own
  redo-panel auto-rerun — that only watches the operator's *own* bound
  properties for changes, and the picker lives elsewhere. `ed.undo()`
  looks like the natural way to mirror what a real redo-panel edit does
  internally, but calling it from inside a property update callback is a
  fragile combination — confirmed to delete the object being edited
  without rebuilding it. The safer approach leans on a convention this
  whole codebase already follows: every generator's `execute()` ends by
  deselecting everything else and selecting + activating exactly what it
  built. That means the update callback can just delete whatever's
  currently selected + active (plus its now-orphaned mesh data, if any)
  and call `execute()` again directly — no undo stack involved.
- **This whole feature category is invisible to headless testing.**
  `context.active_operator` is always `None` in `blender --background`
  scripts, and invoking an operator with `'INVOKE_DEFAULT'` collapses to
  calling `execute()` directly rather than dispatching to a custom
  `invoke()` — there's no real window/event system in background mode for
  either to hook into. Neither the sync-on-pick, the reset-on-invoke, nor
  the rebuild-on-pick behavior can be exercised by any headless script.
  If you build this pattern elsewhere, budget for manual GUI verification
  of these specific behaviors — a clean headless test run does not cover
  them.
- **A loose target poll (accept any object stamped with the right custom
  property, without checking its specific "kind") is a feature, not a
  gap.** For gears this means nothing stops picking a bevel gear as the
  target for a rack — and that turned out to be mechanically correct to
  allow, not just permissive: cross-kind matches can be legitimate
  (sharing a pitch circle across a compound-gear hub, for instance) even
  when the two parts wouldn't mesh directly. Don't tighten a target poll
  to "same kind only" without a concrete reason — see
  [gears/README.md#the-match-target-system-gear_matchingpy](gears/README.md#the-match-target-system-gear_matchingpy)
  for the specific case this came up in.

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
