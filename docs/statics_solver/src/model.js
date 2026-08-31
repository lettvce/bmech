// Plain JSON-serializable data model for a 2D pin-jointed truss.
//
// Structure shape:
// {
//   points:   [{ id, x, y }],
//   members:  [{ id, from, to }],          // from/to are point ids; two-force members only
//   supports: [{ point, type, direction }] // type: 'pin' | 'roller' | 'fixed'; direction only used for 'roller'
//   loads:    [{ point, fx, fy }]          // one entry per loaded point (fx/fy summed if repeated)
// }
//
// A 'fixed' (welded) support resists Fx, Fy, AND rotation — 3 unknowns, like
// a pin plus a pure moment reaction. That moment is real but genuinely
// unrecoverable from a pin-jointed truss's own equilibrium: a truss's joints
// never transmit moment (only axial force flows through two-force members),
// so the moment-reaction unknown contributes to whole-body ΣM but to NO
// per-joint ΣFx/ΣFy equation anywhere. That shows up automatically as a rank
// deficiency of exactly 1 in the classifier — welding a truss to a support
// is correctly flagged as statically indeterminate, which matches the real
// engineering fact that trusses can't make use of a moment restraint.

function pointById(structure, id) {
  const p = structure.points.find((pt) => pt.id === id);
  if (!p) throw new Error(`Unknown point id: ${id}`);
  return p;
}

function memberVector(structure, member) {
  const a = pointById(structure, member.from);
  const b = pointById(structure, member.to);
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy);
  if (length === 0) {
    throw new Error(`Member ${member.id} has zero length (points ${member.from} and ${member.to} coincide)`);
  }
  return { dx, dy, length, ux: dx / length, uy: dy / length };
}

// Unknowns are ordered: one axial force per member (tension positive), then
// one or two reaction components per support (pin -> Rx,Ry; roller -> one
// component along `direction`, default straight up).
function buildUnknownList(structure) {
  const unknowns = [];
  for (const m of structure.members) {
    unknowns.push({ kind: 'member', id: m.id, member: m });
  }
  const idCounts = new Map(); // disambiguate multiple supports sharing a point
  const uniqueId = (base) => {
    const n = (idCounts.get(base) ?? 0) + 1;
    idCounts.set(base, n);
    return n === 1 ? base : `${base}#${n}`;
  };

  for (const s of structure.supports) {
    if (s.type === 'pin') {
      unknowns.push({ kind: 'reaction', id: uniqueId(`${s.point}:Rx`), point: s.point, dirX: 1, dirY: 0 });
      unknowns.push({ kind: 'reaction', id: uniqueId(`${s.point}:Ry`), point: s.point, dirX: 0, dirY: 1 });
    } else if (s.type === 'roller') {
      const dir = s.direction ?? { x: 0, y: 1 };
      const len = Math.hypot(dir.x, dir.y) || 1;
      unknowns.push({
        kind: 'reaction',
        id: uniqueId(`${s.point}:R`),
        point: s.point,
        dirX: dir.x / len,
        dirY: dir.y / len,
      });
    } else if (s.type === 'fixed') {
      unknowns.push({ kind: 'reaction', id: uniqueId(`${s.point}:Rx`), point: s.point, dirX: 1, dirY: 0 });
      unknowns.push({ kind: 'reaction', id: uniqueId(`${s.point}:Ry`), point: s.point, dirX: 0, dirY: 1 });
      // A pure couple: no direction, contributes nothing to any force
      // equation (dirX/dirY both 0) but a fixed +1 to any moment equation
      // regardless of which point it's taken about.
      unknowns.push({ kind: 'reaction', id: uniqueId(`${s.point}:M`), point: s.point, dirX: 0, dirY: 0, isMoment: true });
    } else {
      throw new Error(`Unknown support type: ${s.type}`);
    }
  }
  return unknowns;
}

function netLoadAt(structure, pointId) {
  let fx = 0;
  let fy = 0;
  for (const l of structure.loads) {
    if (l.point === pointId) {
      fx += l.fx;
      fy += l.fy;
    }
  }
  return { fx, fy };
}

// Just the reaction unknowns (no member forces) — what a whole-body
// (external) equilibrium analysis works with, before touching internal
// member forces at all.
function buildReactionUnknowns(structure) {
  return buildUnknownList({ ...structure, members: [] });
}

export { pointById, memberVector, buildUnknownList, buildReactionUnknowns, netLoadAt };
