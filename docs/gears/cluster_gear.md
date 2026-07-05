# Cluster Gear

`gears/external/cluster_gear.py` → `OBJECT_OT_add_cluster_gear` (`object.add_cluster_gear`, "Add Cluster Gear")

Two involute spur gears of different sizes, stacked on one shared axle and
printed as a single monolithic piece (smaller gear on top of the larger
one). Common in gear trains where two shafts need to be replaced by one
compound shaft. Self-contained like the planetary sets — no `stamp_gear`,
no Match Target. See [README.md](README.md) for family-wide conventions.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `bottom_teeth` | Int | 36 | 4–200 (soft) | Gear at the base |
| `top_teeth` | Int | 12 | 4–200 (soft) | Gear on top |
| `module` | Float (mm) | 1.0 | 0.1–50.0 | Shared by both gears |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `width_bottom` | Float (mm) | 6.0 | 0.5–50 (soft) | |
| `width_top` | Float (mm) | 6.0 | 0.5–50 (soft) | |
| `bore_enable` | Bool | True | | |
| `axle_hole_mm` (labeled "Bore Ø") | Float (mm) | 5.0 | 0.1–50 (soft) | |
| `axle_compensation_mm` | Float (mm) | 0.2 | 0.0–1.0 (soft) | Added to hole radius |

## The geometric constraint that makes this work

The two gears are fused at their junction plane (`Z = width_bottom`) by
bridging the bottom gear's top edge loop to the top gear's bottom edge loop
with a single triangulated "shoulder" face. **This only produces valid
geometry if the top gear's outer diameter is smaller than the bottom
gear's dedendum diameter** — i.e. the top gear must sit entirely within
the bottom gear's top face at that junction, not overhang it. The panel
doesn't check this directly; if you set `top_teeth`/`module` large enough
to violate it, expect a broken or self-intersecting shoulder face rather
than an error message.

The one constraint the panel *does* check is the axle bore against both
gears' dedendum radii: if `bore_r >= min(dedendum radii of top and bottom
gears)`, the panel shows an ERROR-icon label,
`"Axle hole too large — max Ø %.2f mm for smallest gear"`. This is a
warning-style label — it doesn't block `execute()`.

## Build method

Reuses `spur_gear.build_gear_profile()` for both gear profiles
(unlike the planetary-set files, which keep their own duplicate copies of
the profile builder). The whole part — bottom gear, shoulder, top gear —
is assembled as **one continuous bmesh solid**, not two separate objects
boolean-unioned; no coincident/duplicate faces at the junction. If a bore
is enabled, it's cut afterward as a single boolean-DIFFERENCE cylinder
through the entire height (`EXACT` solver, `±BOOL_EPSILON` extension —
same pattern as the rest of the family).

## Output

**One object per call**, `ClusterGear` / `ClusterGearMesh`. Success
message: `"Cluster gear: %d / %d teeth, %.2f mm tall"`. Straight teeth
only — no hand, no helix.
