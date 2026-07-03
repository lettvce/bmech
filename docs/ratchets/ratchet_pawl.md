# External Ratchet & Pawl

`ratchets/ratchet_pawl.py` — three operators sharing one set of geometry
helpers: `MESH_OT_add_ratchet_wheel` (`mesh.add_ratchet_wheel`, "Add
Ratchet Wheel"), `MESH_OT_add_ratchet_pawl` (`mesh.add_ratchet_pawl`,
"Add Ratchet Pawl"), and `OBJECT_OT_add_ratchet_mechanism`
(`object.add_ratchet_mechanism`, "Add Ratchet & Pawl") — the combined
operator that builds a matched, auto-positioned pair.

Rigid, no-spring: **+Z (CCW from above) wheel rotation = LOCK**, -Z (CW) =
FREE. See [README.md](README.md) for this family's shared conventions,
especially the lock-direction convention (which flips for the internal
ratchet) and the "no clamping, ever" validation policy.

## Shared constants

`HOLE_SEGMENTS = 32` (hole circle resolution), `PIVOT_BIAS_ANGLE_DEG =
35.0` (tangent-vs-radial blend for the auto pivot solve, see below),
`PIVOT_PAD_MM = 2.0` (minimum wall thickness kept around any pivot/axle
hole).

## Operator 1: Add Ratchet Wheel

### Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `sizing_mode` | Enum | `MODULE` | `MODULE` (module + tooth count, matches the gear family's convention) / `OUTER_DIAMETER` (outer diameter + tooth count, module back-solved) | |
| `tooth_count` | Int | 12 | 4–200 (soft) | |
| `module` | Float (mm) | 3.0 | 0.1–20 (soft) | |
| `outer_diameter_mm` | Float (mm) | 40.0 | 2–500 (soft) | |
| `tooth_depth_auto` | Bool | True | | Derives `tooth_depth_mm = 0.6 * module` |
| `tooth_depth_mm` | Float (mm) | 1.8 | 0.1–50 (soft) | Only used when auto is off |
| `width_mm` | Float (mm) | 6.0 | 0.5–200 (soft) | |
| `bore_enable` | Bool | True | | |
| `axle_hole_diameter_mm` | Float (mm) | 5.0 | 0.1–100 (soft) | |
| `axle_hole_compensation_mm` | Float (mm) | 0.0 | -2.0–2.0 (soft) | Added to hole diameter |

The panel shows a read-only **"Back Face Angle (derived)"** label — see
[README.md](README.md#tooth-ramp-angle-is-derived-never-a-free-input).

### Tooth profile

`build_wheel_profile_points()` builds a closed CCW loop with exactly 2
vertices per tooth (root, tip) — an earlier version used a two-segment
back face to independently satisfy uniform spacing, a radial drive face,
and a free-standing ramp angle at once; v1.1 dropped the third constraint
(ramp angle became derived) and simplified to a single straight back-face
segment per tooth, tip_i → root_(i+1).

### Validation (raises, always cancels — see family README)

`validate_wheel_params()`: `tooth_count >= 4`; `tooth_depth_mm <
root_radius` (else "would collapse the wheel through its own center");
`axle_hole_diameter_mm < 2*root_radius` (else "axle hole is bigger than
the wheel"). All three are checked live in `draw()` (shown as an ERROR
label if any fails) and re-checked in `execute()`, where a failure
returns `CANCELLED`.

### Output

One object, `RatchetWheel`. Several read-only geometry values are stashed
as ID properties on the object for the combined operator (and anything
else) to read back later: `root_radius`, `outer_radius`,
`tooth_depth_mm`, `sector_angle_deg`, `tooth_count`,
`back_face_angle_deg`.

## Operator 2: Add Ratchet Pawl

### Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `pawl_arm_length_mm` | Float (mm) | 25.0 | 2–300 (soft) | |
| `pawl_arm_width_mm` | Float (mm) | 6.0 | 1–50 (soft) | |
| `pawl_tip_width_mm` | Float (mm) | 3.0 | 0.1–50 (soft) | |
| `pivot_hole_diameter_mm` | Float (mm) | 5.0 | 0–50 (soft) | |
| `pivot_hole_compensation_mm` | Float (mm) | 0.0 | -2.0–2.0 (soft) | |
| `width_mm` | Float (mm) | 6.0 | 0.5–200 (soft) | |
| `tip_engagement_depth_mm` | Float (mm) | 1.0 | 0–20 (soft) | How far the tip projects into the tooth valley at full engagement. Informational only here — only the combined operator's auto-positioning solve actually consumes it. |
| `pivot_location` | Vector (mm) | (0,0,0) | | World-space pivot placement. Not part of the original spec — added so this operator is independently usable; the combined operator overrides it with its own solve. |
| `pivot_rotation_deg` | Float (°) | 0.0 | -360–360 (soft) | |

### Pawl shape

`build_pawl_profile_points()` builds a 7-point wedge in the pawl's local
frame, pivot at the origin, arm along +X: a pivot pad
(`hole_radius + PIVOT_PAD_MM` wide) so the pivot hole has material on all
sides, a taper starting at 30% of arm length, and a wedge tip
(`max(tip_width_mm, 1.0)` long, guarding against a degenerate
zero-length wedge) ending at the true tooth-valley contact point.

### Validation

Checks `pawl_arm_length_mm > pivot_hole_diameter_mm +
pivot_hole_compensation_mm` and `hole_dia < pawl_arm_width_mm`, both shown
in `draw()` and enforced in `execute()` (`CANCELLED` on failure).

**One check is deliberately NOT performed here**: whether
`pawl_tip_width_mm` fits the wheel's tooth-valley width. That requires
wheel geometry this standalone operator doesn't have — it's only checked
inside the combined operator below.

### Output

One object, `Pawl`. ID properties `pawl_arm_length_mm`,
`pivot_hole_diameter_mm` stashed on it.

## Operator 3: Add Ratchet & Pawl (combined)

Builds a matched wheel + pawl pair with the pawl's pivot **auto-solved**
so its tip lands correctly on a real tooth drive face.

### Properties

All the wheel and pawl properties above, minus `width_mm` (unified into a
single shared field — "one field, can't diverge" per the source comment),
plus:

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `center_distance_solve_mode` | Enum | `AUTO` | `AUTO` (solve pivot from `pawl_arm_length_mm` + `engagement_side`) / `MANUAL` (use `pawl_pivot_location` directly) | |
| `pawl_pivot_location` | Vector (mm) | (0,0,0) | | Manual mode only |
| `engagement_side` | Enum | `+X` | `+X`/`-X`/`+Y`/`-Y` (pawl sits on that side of the wheel) / `CUSTOM` (uses `engagement_angle_deg`) | |
| `engagement_angle_deg` | Float (°) | 0.0 | -360–360 (soft) | |
| `parent_under_empty` | Bool | True | | Groups both parts under a new `RatchetMechanism` empty |

Note `tip_engagement_depth_mm`'s minimum here is **0.01**, not the 0.0
used by the standalone pawl operator — the auto-solve divides by/compares
against this value, so it can't be exactly zero here.

### The auto-positioning solve — a heuristic, not a rigorous statics derivation

The module's own design-decision log is direct about this: it's a
tangent-biased heuristic, not a moment/statics calculation. `solve_pawl_pivot()`:

1. Maps `engagement_side` to a target angle around the wheel, then
   **snaps to the nearest actual tooth drive-face angle** — "so the
   contact point lands exactly on a real drive face instead of in mid-air
   between teeth."
2. Computes the lock-direction tangent at that contact point (per the
   +Z=LOCK convention) and the outward radial direction.
3. Blends them: `pivot_dir = normalize(-tangent_lock*cos(bias) +
   radial_dir*sin(bias))` with `bias = PIVOT_BIAS_ANGLE_DEG = 35°` — the
   pivot sits mostly *against* the lock-direction surface velocity (so the
   wheel's push rotates the pawl arm further into engagement rather than
   out of it) plus a constant outward lean so the pivot clears the wheel
   body.
4. `pivot_world = contact_world + pawl_arm_length_mm * pivot_dir`.

Raises `ValueError` (→ `CANCELLED`) if `tip_engagement_depth_mm >=
tooth_depth_mm` (tip can't project deeper than the tooth itself), or if
the resulting pivot still ends up inside/at the wheel's outer radius
(`pawl_arm_length_mm` too short).

### Validation — one genuine non-blocking warning in this whole file

Almost everything here follows the raise/cancel discipline described in
[README.md](README.md#validation-no-clamping-ever--but-only-in-ratchet_pawlpy):
wheel geometry checks, pawl structural checks, and the **cross-part check
unique to this operator** — `pawl_tip_width_mm >=
estimate_valley_width_mm(...)` ("Tip too wide for tooth valley") — all
cancel on failure.

The one exception: in **MANUAL** pivot mode, if the manually-placed pivot
ends up more than 0.5mm away from where `pawl_arm_length_mm` says it
should be, `execute()` issues `self.report({'WARNING'}, ...)` but still
builds the mechanism — "the tip won't land exactly on the drive face" is
a warning you can act on or ignore, not a blocking error, since you
explicitly chose manual placement.

### Output

**2 or 3 objects**: `RatchetWheel` + `Pawl`, plus (default on) a
`RatchetMechanism` empty parenting both. Pawl is set active. Success
message reports tooth count, derived back-face angle, and which side the
pawl engages on.
