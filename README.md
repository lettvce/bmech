# bmech — Blender Mechanism Library

A free Blender extension for generating mechanical parts — gears, springs,
ratchets, bearings, and fasteners — parametric and ready to 3D print.

Website: [lettvce.com](https://lettvce.com)

## Install

1. Download the latest `mechanisms_core.zip` from the
   [Releases page](https://github.com/lettvce/bmech/releases/latest).
2. In Blender 5.1+: `Edit > Preferences > Get Extensions`, dropdown in the
   top-right corner → `Install from Disk...`.
3. Select the zip. Generators appear under `Shift-A > Mechanisms`.

## What's inside

- **Gears** — spur, helical, herringbone (external + ring), planetary sets
  (spur/helical/herringbone), straight bevel, all with print-in-place
  clearance support
- **Springs** — hairspring, serpentine spring
- **Ratchets** — external ratchet & pawl, internal freewheel ratchet
- **Fasteners** — threaded fasteners
- **Bearings** — ball bearings

## Development

`mechanisms_core/` is the packaged addon. `Prototype Scripts/` holds
work-in-progress generators run via Alt+P in Blender's Text Editor before
graduating into `mechanisms_core/`.

## Releasing a new version

```
git tag v1.0.1
git push origin v1.0.1
```

A GitHub Action zips `mechanisms_core/` and attaches it to a new Release
automatically — no manual packaging needed.

## License

GPL-3.0-or-later
