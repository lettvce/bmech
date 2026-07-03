# Hairspring

`springs/hairspring.py` → `OBJECT_OT_add_hairspring` (`object.add_hairspring`, "Add Hairspring")

A flat Archimedean-spiral ribbon — the kind of spring used in mechanical
clock balance wheels. See [README.md](README.md) for family-wide notes.

## The spiral formula

```
r(theta) = r_inner + (gap / 2*pi) * theta,   theta in [0, 2*pi*turns]
```

sampled at `N = round(resolution * turns) + 1` points (minimum 2).

## Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `input_mode` | Enum | `MODE_A` | `MODE_A` (Inner/Turns/Gap) / `MODE_B` (Inner/Outer/Turns) | |
| `r_inner` | Float (mm) | 10.0 | 0.1–500 (soft) | |
| `turns` | Float | 5.0 | 0.5–50 (soft) | |
| `gap` | Float (mm) | 2.0 | 0.1–50 (soft) | Radial gap between adjacent coil passes. Mode A only. |
| `r_outer` | Float (mm) | 20.0 | 0.2–500 (soft) | Radius of the outermost coil. Mode B only. |
| `strip_width` | Float (mm) | 1.0 | 0.4–20 (soft) | Ribbon dimension in Z. FDM floor: 0.4mm. |
| `strip_thickness` | Float (mm) | 0.4 | 0.2–10 (soft) | Ribbon dimension radially. FDM floor: 0.2mm. |
| `resolution` | Int | 128 | 8–512 (soft) | Sample points per full turn |

## Mode A vs Mode B

- **MODE_A** (Inner / Turns / Gap): `gap_mm = self.gap` used directly;
  outer radius is shown as a computed read-only label.
- **MODE_B** (Inner / Outer / Turns): `gap_mm = (r_outer - r_inner) /
  turns` is derived, then clamped to a 0.1mm floor. The derived gap is
  shown as a read-only label.

## Panel warnings — only one actually blocks

- Mode B, `r_outer <= r_inner` → **"Outer radius must exceed inner
  radius"** (ERROR icon). **This one does cancel** — `execute()` returns
  `CANCELLED` on this exact condition in Mode B.
- Mode B, derived gap `< 0.1mm` → **"Derived gap below 0.1 mm — increase
  outer radius or reduce turns"** (ERROR icon). This one does **not**
  cancel — `execute()` silently clamps `gap_mm = max(gap_mm, 0.1)` instead
  of stopping. The warning is real (your spiral will be denser than
  requested) but not blocking.

## Build method

`build_manual_ribbon()` walks the sampled centerline points, computes a
tangent at each (central difference, forward/backward at the ends), and a
binormal `tangent × Z` as the local "width" direction — a hand-rolled
sweep frame, not Blender's built-in curve-bevel machinery (hence
"manual" in the function name). Each point gets 4 verts (outer/inner ×
top/bottom in the local frame); the ribbon is closed with start and end
cap quads. `strip_width` becomes ribbon depth in Z; `strip_thickness`
becomes ribbon width radially — there's no Solidify modifier involved,
the thickness is built directly into the mesh.

## Output

One object per call, `Hairspring` (mesh data `HairspringMesh`), placed at
the 3D cursor. No success report message is printed (unlike the other two
generators in this family). No `bmech_*` stamping.
