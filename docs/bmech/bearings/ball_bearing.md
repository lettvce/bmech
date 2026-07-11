# Ball Bearing

`bearings/ball_bearing.py` → `OBJECT_OT_add_ball_bearing` (`object.add_ball_bearing`, "Add Ball Bearing")

Generates an inner race, an outer race, and `ball_count` truncated-sphere
balls — a complete, FDM-assemblable ball bearing. See
[README.md](README.md) for family-wide notes.

## Pitch diameter is derived from ball packing, not typed in

```
ball_center_r = (2*ball_radius_mm + gap_mm) / (2*sin(pi / ball_count))
```

You specify bore ID and outer OD directly; everything else (pitch circle,
groove radius, wall thicknesses) is derived from those two plus the ball
geometry.

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `bore_diameter_mm` | Float (mm) | 6.0 | 0.1–500 (soft) | Inner race ID (axle bore) |
| `outer_diameter_mm` | Float (mm) | 28.0 | 0.2–600 (soft) | Outer race OD |
| `ball_sizing_mode` | Enum | `RADIUS` | `RADIUS` (ball radius is the input, height shown read-only) / `HEIGHT` (truncated height is the input, radius shown read-only) | |
| `ball_radius_mm` | Float (mm) | 3.0 | 0.5–50 (soft) | |
| `ball_height_mm` | Float (mm) | 5.196 | 0.1–100 (soft) | Flat-to-flat height of the truncated ball |
| `overhang_angle_deg` | Float (°) | 30.0 | 1–89 | Maximum printable overhang from horizontal — ball caps are cut at this latitude so no supports are needed |
| `ball_count` | Int | 8 | min 3, soft 6–40 | More balls = larger derived pitch circle |
| `gap_mm` | Float (mm) | 0.5 | 0.05–5 (soft) | Clearance between adjacent ball surfaces |
| `clearance_mm` | Float (mm) | 0.2 | 0.0–2 (soft) | Groove radius = `ball_r + clearance/2`. Increase if the bearing binds after printing. |
| `parent_under_empty` | Bool | True | | |

## Truncated balls — why, and how they're sized

Balls aren't full spheres — both poles are cut flat at `overhang_angle_deg`
from horizontal, specifically so the top/bottom caps sit flush with the
race faces and print without support material. The ball zone itself is
open at top and bottom (not enclosed by the races) so balls can physically
be dropped in during FDM assembly.

```
z_cut  = ball_radius_mm * cos(overhang_rad)     # truncation half-height
r_flat = ball_radius_mm * sin(overhang_rad)     # radius of the flat cap
```

`ball_sizing_mode` just picks which of `ball_radius_mm` / `ball_height_mm`
is the driving input — in HEIGHT mode, `ball_radius_mm = ball_height_mm /
(2 * cos(overhang_rad))` is solved backward from the requested flat-to-flat
height.

## Auto-corrected wall thickness

Unlike most validation in this library, `_compute()` doesn't just flag a
too-thin wall — it **silently widens the geometry to fix it**: if the
inner or outer wall would fall under `MIN_WALL_MM = 0.8mm`, the bore is
shrunk or the outer diameter is expanded (whichever is needed) until both
walls clear that floor, before any geometry is built. The panel shows this
happening via an INFO-icon note (`"Bore reduced to %.2f mm to maintain
%.1f mm inner wall"` / `"OD expanded to %.2f mm to maintain %.1f mm outer
wall"`) — if your bearing's bore or OD comes out different from what you
typed, this is why; check the panel note rather than assuming the input
was ignored.

The panel's wall-thickness rows use a checkmark/error icon per wall
(`>= MIN_WALL_MM` = checkmark, else error), but because of the
auto-correction above, the error state is effectively unreachable through
normal use — it would only show if some other code path bypassed
`_compute()`'s correction.

There's also a separate `validate_bearing()` function defined in the
module (raw error-string checks: ball radius ≤ 0, degenerate truncation,
non-positive gap/bore, OD ≤ bore, wall-too-thin) that is **never called**
by either `draw()` or `execute()` — it's dead code, superseded by the
auto-scaling approach. Don't rely on it running; it doesn't.

## Build method

Both races are 2D `(r, z)` profiles (bore/OD wall, then a semicircular
groove arc spanning `±alpha_cut` around the ball contact latitude) revolved
360° into a bmesh (`RACE_SEGMENTS = 64` segments). Each ball is a
UV-sphere-style mesh (`BALL_LAT = 16` rings × `BALL_LON = 32` segments)
with both poles collapsed to a single flat-cap vertex ring at `±z_cut`
instead of a true pole point — that's the "truncated sphere."

Balls are placed evenly around the derived pitch circle:
`(ball_center_r*cos(angle), ball_center_r*sin(angle))` for
`angle = 2*pi*i/ball_count`.

The only real cancellation path is a caught `ValueError`/`RuntimeError`
around the three build calls — silent, no `self.report()` message.

## Output

**`2 + ball_count` objects per call**: `BearingInnerRace`,
`BearingOuterRace`, and `BearingBall.000`…`BearingBall.%03d` (zero-padded,
sharing no mesh data — each ball is its own independent mesh, unlike the
gear family's linked-copy planets). If `parent_under_empty`, all are
parented under a new `BallBearing` empty (`PLAIN_AXES`, sized to
`outer_r * 0.3`). Inner race is set active. Success message:
`"Ball bearing: bore Ø%.2f mm, OD Ø%.2f mm, %d balls, width %.2f mm"`.

No `bmech_*` stamping — this generator has no Match Target system.
