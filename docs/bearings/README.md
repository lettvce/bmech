# Bearing Family — Shared Conventions

Currently one primitive: `bearings/ball_bearing.py`. This README exists so
the family/primitive doc pattern is already in place when a second bearing
type (roller, thrust, etc.) is added — see
[../CONVENTIONS.md](../CONVENTIONS.md) for the pattern itself.

## Primitives in this family

| Primitive | File |
|---|---|
| Ball bearing | `bearings/ball_bearing.py` |

## Conventions

- Fully self-contained — imports only `bpy`, `bmesh`, `math`. No shared
  helper module (there's nothing to share yet with only one primitive).
- Wall-thickness safety is **auto-corrected, not just validated**: the
  bore and outer diameter are silently nudged (bore shrunk / OD expanded)
  to guarantee both races keep at least `MIN_WALL_MM = 0.8mm` of material,
  and the panel shows an INFO-icon note when this happens. This is a
  different pattern from most other families' geometry validation, which
  cancels or warns rather than auto-adjusting the user's inputs — see
  [ball_bearing.md](ball_bearing.md#auto-corrected-wall-thickness) for
  detail.
- `clearance_mm` and `gap_mm` here are **fit clearances baked directly
  into geometry** (added to a groove radius or a ball-packing gap), not a
  print-shrinkage compensation offset in the sense used elsewhere in this
  library (compare to the fastener family's `*_compensation_mm` fields).
  There is no separate "shrink compensation" property on this generator.
