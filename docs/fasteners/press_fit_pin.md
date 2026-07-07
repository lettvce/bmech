# Press-Fit Pin

`fasteners/press_fit_pin.py` — two operators:
`OBJECT_OT_add_press_pin` (`object.add_press_pin`, "Add Press-Fit Pin")
and `OBJECT_OT_align_press_pin_to_face` (`object.align_press_pin_to_face`,
"Align Press Pin to Face").

A tapered friction-fit pin plus a matching undersized hole cutter, for
manual Boolean Difference assembly. No threads — has no relationship to
the thread-based generators in this family, and doesn't duplicate or
share code with them.

## Nominal size here means something different from the thread files

There's no major/minor diameter concept for a press fit. Instead,
`nominal_diameter_mm` is a **centerline reference diameter**, and pin OD
/ hole ID are derived symmetrically around it, split by the interference
fit:

```
half_interference = interference_mm / 2
pin_diameter_mm  = nominal_diameter_mm + half_interference + pin_diameter_compensation_mm
hole_diameter_mm = nominal_diameter_mm - half_interference + hole_diameter_compensation_mm
```

The symmetric `± half_interference` split is the mechanical fit spec, not
FDM compensation — FDM compensation is a separate, always-additive term
layered on top of each side (see
[README.md](README.md#fdm-compensation-always-added-one-direction-per-field)).
Both compensation terms are added regardless of which side they're on,
because FDM parts shrink either way — a positive value grows whichever
feature (pin or hole) it's attached to, counteracting that shrinkage.

## Operator 1: Add Press-Fit Pin

### Properties

| Property | Type | Default | Range | Notes |
|---|---|---|---|---|
| `nominal_diameter_mm` | Float (mm) | 5.0 | 0.0001 min, 1–50 (soft) | Centerline reference diameter |
| `interference_preset` | Enum | `MEDIUM` | `LIGHT` (0.10mm) / `MEDIUM` (0.20mm) / `TIGHT` (0.30mm) / `CUSTOM` | Picking a preset overwrites `interference_mm`; `CUSTOM` leaves it untouched |
| `interference_mm` | Float (mm) | 0.20 | 0.0001 min, 1.0 (soft) | Total diametral interference (pin OD minus hole ID) |
| `pin_diameter_compensation_mm` | Float (mm) | 0.1 | 0.0 min, 1.0 (soft) | Added to modeled pin diameter |
| `hole_diameter_compensation_mm` | Float (mm) | 0.2 | **no min/max set** | Added to modeled hole diameter — the one property in this family with no bounds at all; every sibling compensation field has `min=0.0` |
| `pin_length_mm` | Float (mm) | 10.0 | 0.0001 min, 100 (soft) | Back face to tip apex, including taper |
| `taper_length_mm` | Float (mm) | 2.0 | 0.0001 min, 20 (soft) | Axial length of the tip lead-in |
| `taper_angle_deg` | Float (°) | 30.0 | 0.0001–89.9999 | **Half-angle** from the central axis |
| `hole_depth_mm` | Float (mm) | 10.0 | 0.0001 min, 100 (soft) | |
| `hole_extend_margin_mm` | Float (mm) | 0.0 | 0.0 min, 10 (soft) | Extra cutter length past hole depth, for clean through-cuts |
| `radial_segments` | Int | 32 | 8–128 (soft) | Shared by both pin and cutter |

`interference_preset`'s `CUSTOM` option doesn't reset anything — it just
stops overwriting `interference_mm`, so whatever value was last set
(from a preset or typed manually) stays in effect.

### Build method

Pin: a three-ring revolve — straight cylindrical shaft, then a tapered
cone section narrowing to `tip_diameter_mm` (0 = a true point). Hole
cutter: a plain non-tapered cylinder (`bmesh.ops.create_cone` with equal
radii) sized to `hole_diameter_mm`, extended by `hole_extend_margin_mm` —
note the cutter's own shape does **not** taper to match the pin's tip;
it's a straight bore the full way down.

### Panel warnings

`validate_press_pin_parameters()` returns a list of error strings shown
live in the panel (ERROR icon per line): non-positive interference, hole
diameter, or pin diameter; `taper_length_mm >= pin_length_mm`; tip
diameter below the printable floor `NOZZLE_WIDTH_MM = 0.4mm`;
`radial_segments < 8`.

**These do cancel execute()** — `if errors: return {'CANCELLED'}` — but
unlike `hex_bolt.py`/`hex_nut.py`, **no `self.report()` call accompanies
the cancellation.** If this operator silently does nothing, check the
panel's ERROR-icon labels; nothing will appear in the status bar to
explain why. The same silent-cancel pattern applies to the
`try/except Exception` wrapped around the actual mesh-building calls —
any build exception also cancels with no reported message.

### Output

Two objects per call, both placed at the same 3D cursor location:
`PressPin` and `PressPin_HoleCutter`. Both are tagged with a `bmech_kind`
custom property (`"press_pin"` / `"press_pin_cutter"` respectively) — not
the gear family's Match Target system, but a similar mechanism: it's how
the second operator's object pickers filter to only pins/cutters this
tool produced.

## Operator 2: Align Press Pin to Face

Only runs in Edit Mode on a mesh with an active face selected
(`poll()` requires `context.mode == 'EDIT_MESH'`). Orients a pin and/or
cutter object to stick out of (pin) or recess into (cutter) that face
along its normal — meant to be run after generating a pin/cutter pair,
to place them without doing the trig by hand.

### Properties

| Property | Type | Default | Notes |
|---|---|---|---|
| `bmech_press_pin_align_pin` | Object pointer (WindowManager) | — | Filtered to objects tagged `bmech_kind == "press_pin"` |
| `bmech_press_pin_align_cutter` | Object pointer (WindowManager) | — | Filtered to objects tagged `bmech_kind == "press_pin_cutter"` |
| `standoff_mm` | Float (mm) | 0.001 | 0.0 min, 0.1 (soft) | Small deliberate overlap so a later boolean doesn't land on a knife-edge coincident face |

**[BUG, fixed] The pin/cutter pickers used to be `PointerProperty`s directly
on the operator** (`pin_object`/`cutter_object`) — Blender rejects that
outright for any ID-type property ("this type doesn't support data-block
properties"), throwing a registration error on every addon load. Moved
onto `WindowManager` instead (`bmech_press_pin_align_pin`/
`bmech_press_pin_align_cutter`), the same fix this family's own Match
Target pickers already use — `execute()`/`draw()` read them from
`context.window_manager` rather than `self`.

### Behavior

Face normal is transformed correctly under non-uniform scale, via the
inverse-transpose technique (`obj.matrix_world.inverted_safe().transposed().to_3x3()
@ face.normal`) rather than a naive `matrix_world @ normal`, which would
give the wrong direction if the mesh has non-uniform scale applied.
Rotation around the normal is left unconstrained (`to_track_quat`'s
arbitrary up-axis choice) since both the pin and cutter are surfaces of
revolution around their own Z — there's no visible "roll" to control.

`standoff_mm`'s sign differs by object, and both directions push the mesh
*further into* whatever it needs to boolean against, not away from it:
- Pin: placed at `center - normal * standoff` — pulled slightly into the
  face, so its base overlaps the surface for a clean union.
- Cutter: placed at `center + normal * standoff` — pushed slightly past
  the face on the material side, so it overlaps the solid for a clean
  difference.

### Panel warnings

`execute()` reports and cancels (with an explicit `self.report({'ERROR'},
...)`, unlike the sibling operator above) if there's no active selected
face, or if neither `pin_object` nor `cutter_object` is set.
