# Rack

`gears/external/involute_gear_rack.py` → `OBJECT_OT_add_rack` (`object.add_rack`, "Add Gear Rack")

Straight involute rack — the "gear of infinite radius" that converts a
mating spur/helical pinion's rotation into linear motion. See
[README.md](README.md) for family-wide conventions.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_module_pa`, and if the target has `bmech_tooth_count`, also updates `tooth_count_rack` |
| `module` | Float (mm) | 1.0 | 0.1–50.0 | Must match the mating gear's module |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `length_mode` | Enum | `TOOTH_COUNT` | `TOOTH_COUNT` / `MATCH_GEAR` | See below |
| `tooth_count_rack` | Int | 10 | 2–1000 | Used directly in `TOOTH_COUNT` mode; fallback in `MATCH_GEAR` mode if no target is set |
| `width_mm` | Float (mm) | 6.0 | 0.1–100 (soft) | Solidify modifier depth |

No bore properties — a rack has no hub to bore.

## `length_mode`

- **`TOOTH_COUNT`** — rack length is exactly `tooth_count_rack` teeth,
  independent of any target.
- **`MATCH_GEAR`** — if a target gear is set (and stamped with
  `bmech_tooth_count`), the rack is sized to span **one full pitch
  circumference** of that gear (i.e. `tooth_count_rack` teeth = the
  target's own tooth count), and the rack is positioned/centered against
  the target automatically: `obj.location = target.location`, offset down
  by the target's pitch radius (Y) and left by half the rack length minus
  half a tooth pitch (X). If no target is set, this mode silently falls
  back to using `tooth_count_rack` as typed.

## Build method

Same flat-profile-plus-Solidify approach as the spur gear:
`build_rack_profile()` tiles `build_rack_tooth_profile()` (straight flanks
at `pressure_angle_deg` from vertical, with a quarter-circle root fillet of
radius `ROOT_FILLET_COEFF * module = 0.38 * module` — the rack tooth
profile is the only builder in this library that fillets the root; gear
teeth elsewhere use a bare dedendum-circle arc) across X, then closes the
profile with a rectangular base extending `dedendum + module` below the
root line for a solid foundation.

## Output

One object, name `Rack` (or `Rack.001`, etc.), stamped
`gear_matching.stamp_gear(obj, "rack", module, pressure_angle_deg)` — note
no `tooth_count` is stamped on a rack (a rack's "tooth count" isn't a
meshing constraint the way a gear's is).
