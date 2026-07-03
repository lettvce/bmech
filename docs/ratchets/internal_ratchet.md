# Internal Freewheel Ratchet

`ratchets/internal_ratchet.py` → `OBJECT_OT_add_internal_ratchet` (`object.add_internal_ratchet`, "Add Internal Freewheel Ratchet")

A bicycle-freewheel-style mechanism: an outer ring with inward-pointing
teeth, an inner hub, and `pawl_count` pawls mounted on the hub pointing
radially outward. **CW rotation of the inner hub (viewed from +Z) =
LOCK; CCW = FREE** — the opposite convention from
[ratchet_pawl.md](ratchet_pawl.md)'s external wheel, see
[README.md](README.md#the-lock-direction-convention-flips-between-the-two-files--read-this-first)
for why.

## Shared constants

`HOLE_SEGMENTS = 32`, `RING_SEGMENTS_OUT = 64` (the outer ring's own
circle resolution — finer than hole segments since it's the visible
outer silhouette), `PIVOT_PAD_MM = 2.0`.

Some geometry helpers (`finalize_mesh_object`, `build_pawl_profile_points`,
a variant of `create_filled_profile_with_hole`) are copied from
`ratchet_pawl.py` — see
[README.md](README.md#why-internal_ratchetpy-duplicates-code-from-ratchet_pawlpy).

## Properties

**Ring:**

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `sizing_mode` | Enum | `MODULE` | `MODULE` / `OUTER_DIAMETER` | |
| `tooth_count` | Int | 24 | 4–200 (soft) | |
| `module` | Float (mm) | 2.0 | 0.1–20 (soft) | |
| `outer_diameter_mm` | Float (mm) | 50.0 | 5–500 (soft) | |
| `ring_wall_thickness_mm` | Float (mm) | 4.0 | 0.5–50 (soft) | Radial thickness from outer surface to tooth roots |
| `tooth_depth_auto` | Bool | True | | `tooth_depth_mm = 0.6 * module` |
| `tooth_depth_mm` | Float (mm) | 1.2 | 0.1–20 (soft) | Only used when auto is off |

**Hub:**

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `hub_outer_diameter_mm` | Float (mm) | 20.0 | 1–200 (soft) | Must be smaller than tip diameter minus clearance |
| `bore_diameter_mm` | Float (mm) | 5.0 | 0–100 (soft) | |
| `bore_compensation_mm` | Float (mm) | 0.0 | -2.0–2.0 (soft) | Added to bore diameter |
| `clearance_mm` | Float (mm) | 0.3 | 0–5 (soft) | Running gap between hub surface and tooth tips — see [README.md](README.md#fdm-compensation-vs-running-clearance--two-different-things-dont-conflate-them). Not a hole compensation. |

**Pawls:**

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `pawl_count` | Int | 3 | **min=1 (hard)**, soft_max 12 | The only property in this family (or the gear family) with a hard rather than soft minimum |
| `pawl_arm_length_auto` | Bool | True | | Computes arm length so the tip just reaches the tooth valley |
| `pawl_arm_length_mm` | Float (mm) | 8.0 | 1–100 (soft) | Only used when auto is off |
| `pawl_arm_width_mm` | Float (mm) | 4.0 | 1–30 (soft) | |
| `pawl_tip_width_mm` | Float (mm) | 1.5 | 0.1–20 (soft) | |
| `tip_engagement_depth_mm` | Float (mm) | 0.6 | 0–10 (soft) | How far the tip projects past the tooth tips into the valley |
| `pivot_hole_diameter_mm` | Float (mm) | 2.0 | 0–20 (soft) | |
| `pivot_hole_compensation_mm` | Float (mm) | 0.0 | -2.0–2.0 (soft) | |

**Shared:**

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `width_mm` | Float (mm) | 6.0 | 0.5–200 (soft) | Shared by ring, hub, and all pawls |
| `parent_under_empty` | Bool | True | | Groups everything under a `FreewheelRatchet` empty |

## Pawl positioning — radial, not tangent-biased

Unlike `ratchet_pawl.py`'s tangent-biased heuristic solve, an internal
ratchet's pawls are mounted radially outward from the hub, so the
positioning math is much simpler:
`auto_pawl_arm_length() = max(contact_r - pivot_r, 1.0)` where both the
pivot and the tip contact point lie on the same radial line through the
hub center. No angular bias term is needed.

Pawls are distributed at `angle = i * 2*pi/pawl_count + pi/tooth_count` —
the `+pi/tooth_count` term is a half-tooth-pitch phase offset, likely so a
pawl doesn't spawn straddling a tooth root/tip boundary by coincidence
(not explicitly commented in the source, but the pattern is consistent
across all `pawl_count` placements).

## Ring teeth: a non-convex boundary needs a different tessellation method

`build_inner_teeth_points()` produces the same sawtooth pattern as the
external wheel's tooth builder, with root/outer radius roles swapped so
teeth point inward. But because the inner boundary is **non-convex**,
`build_ring_object()` can't use `bmesh.ops.triangle_fill` the way every
other ring-shaped generator in this library does — it uses
`mathutils.geometry.tessellate_polygon` instead, with the outer contour
wound CCW and the inner (tooth) contour wound CW as a hole. See
[README.md](README.md#why-internal_ratchetpy-duplicates-code-from-ratchet_pawlpy)
for why this matters if you're writing a new non-convex ring generator.

Drive/ramp faces per tooth (from the builder's own docstring): **drive
face** `root_i -> tip_i` (radial, catches the pawl on CW hub rotation);
**ramp face** `tip_i -> root_(i+1)` (angled chord, pawl slides over it on
CCW/FREE rotation).

## Validation

`validate_freewheel()` returns a list of error strings covering: non-positive
ring/tip radius, `tooth_depth_mm >= ring_inner_r`, `hub_outer_r >= tip_r -
clearance_mm` ("Hub outer radius must be < tip radius - clearance —
reduce hub diameter or increase ring size"), `bore_r >= hub_outer_r`,
`tooth_count < 4`, `pawl_count < 1`, pivot-hole-vs-arm-width, and
pawl-arm-length-vs-pivot-hole. `draw()` shows every returned error at
once in an "Issues:" box; `execute()` re-runs the same full check and
cancels (`CANCELLED`, no `self.report()` message) if anything comes back
non-empty — matching this family's general no-clamping discipline (see
[README.md](README.md#validation-no-clamping-ever--but-only-in-ratchet_pawlpy)).

## Output

**`2 + pawl_count` objects** per call: `FreewheelRing`, `FreewheelHub`,
and `pawl_count` pawls named `FreewheelPawl.000`…`FreewheelPawl.%03d`
(zero-padded). Default `pawl_count=3` gives 5 objects. If
`parent_under_empty`, all are parented under a new `FreewheelRatchet`
empty (`PLAIN_AXES`, sized to `ring_outer_r * 0.3`). Ring is set active.
Success message: `"Internal freewheel ratchet: %d teeth, %d pawls, ring Ø
%.1f mm"`.
