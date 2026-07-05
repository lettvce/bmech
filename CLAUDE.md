# bmech — Blender Mechanism Library

Before making changes to `mechanisms_core/`, read
[docs/CONVENTIONS.md](docs/CONVENTIONS.md) — it's the single source of
truth for how documentation and conventions are organized across part
families, and lists cross-family rules (units, FDM compensation
direction, validation severity, boolean-cutter cleanup,
compensation-vs-clearance).

**When you add or change a mechanism generator, update its documentation
in the same change.** This is not optional follow-up work — see
[docs/CONVENTIONS.md#documentation-is-not-optional--write-it-alongside-the-code](docs/CONVENTIONS.md#documentation-is-not-optional--write-it-alongside-the-code)
for exactly what that means for a new primitive vs. a new family vs. an
edit to an existing one.

Every family is documented:

- [docs/gears/](docs/gears/README.md) — 15 primitives, split into
  `external/`, `ring/`, `planetary/`, `bevel/`, `rack/` subfamilies matching
  `mechanisms_core/gears/`'s folder layout.
- [docs/fasteners/](docs/fasteners/README.md)
- [docs/bearings/](docs/bearings/README.md)
- [docs/springs/](docs/springs/README.md)
- [docs/ratchets/](docs/ratchets/README.md)
