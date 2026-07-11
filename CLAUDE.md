# bmech — Blender Mechanism Library

Before making changes to `mechanisms_core/`, read
[docs/bmech/CONVENTIONS.md](docs/bmech/CONVENTIONS.md) — it's the single
source of truth for how documentation and conventions are organized across
part families, and lists cross-family rules (units, FDM compensation
direction, validation severity, boolean-cutter cleanup,
compensation-vs-clearance).

**When you add or change a mechanism generator, update its documentation
in the same change.** This is not optional follow-up work — see
[docs/bmech/CONVENTIONS.md#documentation-is-not-optional--write-it-alongside-the-code](docs/bmech/CONVENTIONS.md#documentation-is-not-optional--write-it-alongside-the-code)
for exactly what that means for a new primitive vs. a new family vs. an
edit to an existing one.

Every family is documented:

- [docs/bmech/gears/](docs/bmech/gears/README.md) — 15 primitives, split into
  `external/`, `ring/`, `planetary/`, `bevel/`, `rack/` subfamilies matching
  `mechanisms_core/gears/`'s folder layout.
- [docs/bmech/fasteners/](docs/bmech/fasteners/README.md)
- [docs/bmech/bearings/](docs/bmech/bearings/README.md)
- [docs/bmech/springs/](docs/bmech/springs/README.md)
- [docs/bmech/ratchets/](docs/bmech/ratchets/README.md)
