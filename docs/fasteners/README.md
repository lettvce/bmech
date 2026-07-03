# Fastener Family — Shared Conventions

Covers everything true for every generator in `mechanisms_core/fasteners/`.
Primitive-specific docs (one per generator, in this folder) assume you've
read this first. See [../CONVENTIONS.md](../CONVENTIONS.md) for the
cross-family rules this family follows, and [../gears/README.md](../gears/README.md)
for a contrasting example of how a different family organizes its shared
code.

## Primitives in this family

| Primitive | File | Builds |
|---|---|---|
| Raw threaded fastener | `fasteners/threaded_fastener.py` | A thread helix solid, for manual boolean combination |
| Hex bolt | `fasteners/hex_bolt.py` | Head + shank + external thread + tip, unioned into one part |
| Hex nut | `fasteners/hex_nut.py` | Hex prism with an internal thread cut through it |
| Press-fit pin | `fasteners/press_fit_pin.py` | A tapered friction-fit pin + matching hole cutter, plus a face-alignment operator |

## This family does NOT share a helper module — and that's deliberate

Unlike the gear family (which centralizes shared logic in
`gears/gear_matching.py` and has every gear import it), **every fastener
generator module is fully self-contained.** `hex_bolt.py` and `hex_nut.py`
each carry their own copies of `_thread_params`, `_build_helix`, and
either `_external_profile` or `_internal_profile` — logic that
originates in `threaded_fastener.py` and is duplicated **verbatim**, not
imported. `hex_nut.py`'s source states the reason directly:

> "Thread math is duplicated from threaded_fastener.py per this project's
> convention (each generator module is self-contained, no cross-file
> thread math imports)."

`hex_bolt.py` and `hex_nut.py` also independently duplicate their own
hex-prism/cylinder/boolean-union helpers between each other, extending the
same convention beyond just thread math.

**If you're adding a new fastener generator that needs thread geometry,
copy `_thread_params`/`_build_helix`/the relevant profile function from
`threaded_fastener.py` into your new file rather than importing them.**
This is the opposite instinct from the gear family, where reuse via
import is preferred — know which family you're in before reaching for
either pattern.

## Thread nomenclature: nominal size is always major diameter

Every thread-based generator (`threaded_fastener.py`, `hex_bolt.py`,
`hex_nut.py`) treats the user-facing diameter property as the **major**
diameter, for both external (bolt) and internal (nut) threads — e.g. an
M8 property means an 8.000mm major diameter on both the bolt and the nut
that fits it. `hex_nut.py`'s source states this explicitly:

> "Standard thread nomenclature: the nominal/basic size IS the major
> diameter, for both external (bolt) and internal (nut) threads... The
> minor diameter... is DERIVED from major_r via pitch + flank angle, not
> the other way around."

The minor diameter is **never** a user input anywhere in this family —
always computed by `_thread_params(major_r, pitch_mm, flank_angle_deg,
truncation)`:

```
half_angle  = max(radians(flank_angle_deg / 2), radians(0.5))   # guards tan(0)
crest_flat  = truncation * pitch_mm
root_flat   = 2 * truncation * pitch_mm
flank_dz    = max((pitch_mm - crest_flat - root_flat) / 2, 0.0)  # clamped, not negative
depth       = flank_dz / tan(half_angle)   if flank_dz > 0  else 0
minor_r     = major_r - depth
```

`press_fit_pin.py` has no threads and doesn't use this convention — see
its own doc for the analogous "nominal is a centerline reference, pin and
hole are derived symmetrically around it" rule.

## FDM compensation: always added, one direction per field

Every compensation property in this family is **added** to a radius or
diameter, never subtracted — consistent with the rest of this library.
Which field you set depends on which side of the part is printed:

- **External features that print undersized** (a bolt's thread, a
  press-fit pin): compensation is added to grow the feature back out.
- **Internal features that print tight** (a nut's thread bore, a cut
  hole, a press-fit hole cutter): compensation is added to open the hole
  back up.

`threaded_fastener.py` exposes both `outer_compensation_mm` (External +
Additive mode) and `inner_compensation_mm` (Subtractive or Internal +
Additive modes) on the same operator, since one operator covers all four
mode combinations. `hex_bolt.py` only exposes `outer_compensation_mm`
(it only ever builds External + Additive). `hex_nut.py` only exposes
`inner_compensation_mm`. `press_fit_pin.py` exposes both
`pin_diameter_compensation_mm` and `hole_diameter_compensation_mm`
independently, since it builds both parts in one operator call.

`press_fit_pin.py`'s interference math additionally documents *why* this
matters for a two-part fit: total interference is split symmetrically
around the nominal diameter (half added to the pin, half subtracted from
the hole) — that symmetric split is not FDM compensation, it's the
mechanical fit spec. FDM compensation is a *separate*, always-additive
term layered on top of each side of that split:

```
half_interference = interference_mm / 2
pin_diameter_mm  = nominal_diameter_mm + half_interference + pin_diameter_compensation_mm
hole_diameter_mm = nominal_diameter_mm - half_interference + hole_diameter_compensation_mm
```

## Boolean-solver patterns specific to this family

Fastener geometry leans on Blender's `EXACT` boolean solver far more
heavily than the gear family (whole bolts and nuts are assembled from
several unioned/differenced solids, not just one bore cut). Some patterns
that show up repeatedly and are worth knowing before editing this code:

- **Union overlap is depth-scaled, not a fixed hairline epsilon.**
  `hex_bolt.py` and `hex_nut.py` both compute
  `overlap = max(0.02, min(0.2 * depth, 0.15))` and build the underlying
  solid slightly larger than the thread ridge's true minor radius before
  unioning the ridge onto it. Two nearly-coincident curved surfaces along
  a full helix length are numerically fragile for the EXACT solver — a
  hairline gap can be misread as a shared boundary and cancel material
  instead of merging. A small but real volume of overlap avoids this.
- **Chained revolves, not stacked independently-capped primitives.**
  `hex_bolt.py`'s `_add_chained_revolve` builds the shank→thread-core→tip
  stack as one continuous solid sharing vertices at each junction, rather
  than three separately-capped cylinders touching end to end. Two
  independently-capped segments meeting at the same radius would produce
  duplicate coincident faces (each segment's own cap covering the
  identical disk) — a fragile degenerate case for the EXACT solver.
  Chaining through shared vertices means there's no internal face for
  this to happen to.
- **Blocky joins last, curved unions first.** `hex_bolt.py` unions the
  thread ridge onto the core before unioning the hex head on, specifically
  because a flush blocky join (hex prism onto a cylinder end) is far
  friendlier to the EXACT solver than a thin curved union — sequencing
  the riskier union earlier means any failure surfaces before the cheap,
  reliable step, not after.
- **Re-normalize after every union.** `_bool_union` in both files calls
  `recalc_face_normals` on the result after applying the modifier and
  deleting the operand, because EXACT-solver unions of complex meshes can
  come back with inconsistent face winding even when topologically solid.
- **Padding direction depends on additive vs. subtractive.** Subtractive
  cutters (a bore, a hole) are over-extended `±BOOL_EPSILON` past the
  target's real boundary — safe, since removing a little extra material
  past an edge is harmless. Additive ridges/features get **no** such
  padding — extending an additive union past the part's real boundary
  would leave a visible nub poking out the other side. `hex_nut.py`'s
  source draws this contrast explicitly at the ridge-union step.
- **`_build_helix`'s step count rounds up**, so a thread helix can slightly
  overshoot its requested height. `hex_nut.py` corrects for this with a
  final `INTERSECT` boolean against a clean Z-bounded cylinder, guaranteeing
  a flush top/bottom face regardless of the overshoot.

## Validation: draw() warnings are often looser than the execute() gate

`threaded_fastener.py`'s draw() shows ERROR-icon labels for degenerate
truncation/flank-angle combinations, but **nothing in its execute() ever
cancels** — the raw thread-helix operator always builds something, even
if the panel is warning you it looks wrong. `hex_bolt.py` and
`hex_nut.py`, by contrast, both **do** enforce a hard gate in execute()
(`self.report({'ERROR'}, ...)` + `CANCELLED`) — but the threshold used
there is looser than the one that triggers the draw() warning. E.g.
`hex_bolt.py` shows an ERROR-icon "Head too small for thread diameter"
warning once `head_wall <= 0.5mm`, but only actually cancels execution at
`head_wall <= 0`. A wall between 0 and 0.5mm will build successfully
despite the visible warning — read the number, not just the icon color.

`press_fit_pin.py` is the outlier: its `execute()` cancels silently on
either a failed `validate_press_pin_parameters()` check or a caught
exception during mesh building, in both cases **without** calling
`self.report()`. If a press pin operator returns `CANCELLED` with no
visible error message, check the panel's validation labels — nothing
will appear in the status bar to tell you what went wrong.
