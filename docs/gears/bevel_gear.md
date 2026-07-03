# Bevel Gear

`gears/bevel/bevel_gear.py` → `OBJECT_OT_bevel_gear` (`object.bevel_gear`, "Bevel Gear")

Straight-tooth bevel gear — a toothed frustum for shafts meeting at 90°,
built via **Tredgold's approximation** (each axial slice is the standard
flat involute profile, uniformly scaled toward the cone apex — not a true
spherical involute). See [README.md](README.md) for family-wide
conventions.

Bevel gears in this library have **no `hand` property** (straight teeth
only, no spiral-bevel support) and **no bore feature** — the two biggest
differences from every other single-gear primitive.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `target` | Object pointer | — | gears with `bmech_module` | Match Target; runs `sync_bevel` — copies module/PA, and sets `mate_teeth` to the target's own `tooth_count` |
| `tooth_count` | Int | 16 | 8–120 (soft) | This gear's own tooth count |
| `mate_teeth` | Int | 16 | 8–120 (soft) | Tooth count of the **mating** gear — sets this gear's cone angle |
| `module` | Float (mm) | 2.0 | 0.1–20.0 (soft) | Module at the large end (back face) |
| `pressure_angle_deg` | Float (°) | 20.0 | 10–45 | |
| `face_width_mm` | Float (mm) | 8.0 | 1–60 (soft) | Tooth length along the cone slant — keep ≤ cone distance / 3 |
| `n_slices` | Int | 12 | 2–64 (soft) | Axial divisions — more = smoother taper |

Note the minimum tooth count here is **8**, not the 5 used by
spur/helical/herringbone — keeps the cone-angle math and small-end tooth
size sane at the apex.

## Making a mating pair

Two bevel gears mesh at 90° with coincident apices when their cone angles
are complementary: `delta_1 + delta_2 = 90°`. This library derives cone
angle from `atan(tooth_count / mate_teeth)`, so **to build the mating
gear, create a second bevel gear with `tooth_count` and `mate_teeth`
swapped** — or just point its `target` at the first gear and let
`sync_bevel` do the swap for you automatically.

```
delta         = atan(tooth_count / mate_teeth)      # this gear's cone half-angle
pitch_r       = module * tooth_count / 2             # at the large end
cone_dist L   = pitch_r / sin(delta)                 # apex-to-back-face slant distance
z_apex        = L * cos(delta) = pitch_r * mate_teeth / tooth_count
z_top         = face_width_mm * cos(delta)           # axial height of the small end
scale_top     = (L - face_width_mm) / L               # uniform XY scale at the small end
```

## Build method

One 2D involute profile is built once at full size (the large end). For
each of `n_slices` axial layers from `z=0` to `z=z_top`, every vertex of
that profile is uniformly scaled by `(z_apex - z) / z_apex` and placed at
height `z` — this uniform per-slice scaling **is** Tredgold's
approximation. No twist, no hand.

End caps use center-fan triangulation (single center vertex per face,
fanned to the rim) at both `z=0` and `z=z_top`. This is explicitly why
there's no bore feature: the fan fill already covers the hub area under
the root circle, so there's no boolean step to hook a bore into without
also cutting into tooth material.

## Panel warnings

- `face_width_mm >= cone_dist` → **"Face width ≥ cone distance — gear
  vanishes"** (ERROR, blocks — `execute()` reports
  `self.report({'ERROR'}, ...)` and returns `CANCELLED`. This is the only
  bevel-gear check that produces an explicit error report rather than a
  silent cancel).
- `face_width_mm > cone_dist / 3` (and not the above) → **"Face width > L/3
  — small-end teeth very small"** (ERROR-icon label, but **not
  blocking** — `execute()` calls `self.report({'WARNING'}, ...)` and still
  builds the gear). `cone_dist / 3` is called out in the source as "the
  standard engineering limit" for bevel gears.

The info box also shows: cone angle, mate cone angle (`90° - delta`),
pitch/tip/root diameter at the large end, cone distance, height, tip
scale, pitch diameter at the small end, and the `N:Nm` ratio.

## Output

One object, stamped:
```python
gear_matching.stamp_gear(obj, "bevel", module, pressure_angle_deg,
                          tooth_count=tooth_count)
```

Note `mate_teeth` and `hand` are **not** stamped — unlike helical/
herringbone gears, the bevel-matching system relies entirely on
`sync_bevel` reading the target object's live `bmech_tooth_count` at
pick-time, not on any stamped mate/cone data on the gear itself.

On success: `self.report({'INFO'}, "Bevel gear: %d/%d teeth, δ=%.1f°,
module %.1f, face %.1f mm")`.
