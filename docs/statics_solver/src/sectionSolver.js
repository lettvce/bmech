import { pointById, netLoadAt } from './model.js';
import { reactionForcesByPoint } from './jointSolver.js';
import { solveBySubstitution } from './substitutionSolver.js';

const ON_LINE_EPS = 1e-6;

// Signed distance-ish value: which side of the infinite line through
// line.p1/line.p2 a point falls on (sign only matters, not magnitude).
function sideOf(point, line) {
  const { p1, p2 } = line;
  return (p2.x - p1.x) * (point.y - p1.y) - (p2.y - p1.y) * (point.x - p1.x);
}

// A cutting line is just two clicked screen points extended to an infinite
// line — same idea as a CAD section/cutting plane. Any member whose two
// endpoints fall on opposite sides is "cut"; the line partitions every joint
// into one side or the other.
function computeCut(structure, line) {
  const sideById = new Map();
  const ambiguous = [];
  for (const p of structure.points) {
    const s = sideOf(p, line);
    sideById.set(p.id, s);
    if (Math.abs(s) < ON_LINE_EPS) ambiguous.push(p.id);
  }

  const cutMembers = structure.members.filter((m) => sideById.get(m.from) * sideById.get(m.to) < 0);
  const sideAPoints = structure.points.filter((p) => sideById.get(p.id) > 0).map((p) => p.id);
  const sideBPoints = structure.points.filter((p) => sideById.get(p.id) < 0).map((p) => p.id);

  return { cutMembers, sideAPoints, sideBPoints, ambiguous };
}

function momentOf(at, about, fx, fy) {
  return (at.x - about.x) * fy - (at.y - about.y) * fx;
}

// Method of sections: isolate one side of a cut as its own free body. Any
// member entirely on one side is internal to that free body and never needs
// representing (its two joint contributions would cancel in the whole-body
// sum anyway) — only members crossing the cut appear, as unknown external
// forces acting along their known original direction. This only works when
// reactions are already known, and only when the cut severs <= 3 members
// (exactly the same "3 whole-body equations" ceiling as external reactions).
function solveSection(structure, solveResult, line) {
  if (!solveResult || solveResult.status !== 'determinate') {
    return { solved: false, reason: 'Solve the truss first — method of sections needs known reactions to work with.' };
  }

  const { cutMembers, sideAPoints, sideBPoints, ambiguous } = computeCut(structure, line);

  if (ambiguous.length > 0) {
    return { solved: false, reason: `Cutting line passes through joint(s) ${ambiguous.join(', ')} — nudge it slightly.` };
  }
  if (sideAPoints.length === 0 || sideBPoints.length === 0) {
    return { solved: false, reason: 'Cutting line does not separate the structure into two sides.' };
  }
  if (cutMembers.length === 0) {
    return { solved: false, reason: 'Cutting line does not cross any members.' };
  }
  if (cutMembers.length > 3) {
    return {
      solved: false,
      reason: `Cuts ${cutMembers.length} members — method of sections only has 3 whole-body equations, so it can resolve at most 3 unknowns per cut. Try a cut through fewer members.`,
    };
  }

  // Either side gives the same answer once reactions are known; pick the
  // smaller free body.
  const side = sideAPoints.length <= sideBPoints.length ? sideAPoints : sideBPoints;
  const sideSet = new Set(side);

  const reactionByPoint = reactionForcesByPoint(structure, solveResult.reactions);
  const netKnownAt = (pointId) => {
    const load = netLoadAt(structure, pointId);
    const reaction = reactionByPoint.get(pointId) ?? { fx: 0, fy: 0 };
    return { fx: load.fx + reaction.fx, fy: load.fy + reaction.fy };
  };

  // Each cut member contributes one unknown, anchored at whichever endpoint
  // sits on this side — same tension-pulls-toward-the-other-end convention
  // used everywhere else in the solver.
  const unknownMeta = cutMembers.map((m) => {
    const here = sideSet.has(m.from) ? m.from : m.to;
    const there = here === m.from ? m.to : m.from;
    const a = pointById(structure, here);
    const b = pointById(structure, there);
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len = Math.hypot(dx, dy);
    return { id: m.id, point: here, ux: dx / len, uy: dy / len };
  });
  const unknownIds = unknownMeta.map((u) => u.id);

  let knownFx = 0;
  let knownFy = 0;
  for (const pid of side) {
    const k = netKnownAt(pid);
    knownFx += k.fx;
    knownFy += k.fy;
  }

  const equations = [
    {
      label: 'ΣFx = 0 (section)',
      coeff: Object.fromEntries(unknownMeta.map((u) => [u.id, u.ux])),
      rhs: -knownFx,
    },
    {
      label: 'ΣFy = 0 (section)',
      coeff: Object.fromEntries(unknownMeta.map((u) => [u.id, u.uy])),
      rhs: -knownFy,
    },
  ];

  // Moment about every joint on this side that a cut member actually
  // touches — the "busy point" trick, generalized from reactions.
  const momentPoints = [...new Set(unknownMeta.map((u) => u.point))];
  for (const pointId of momentPoints) {
    const about = pointById(structure, pointId);
    const coeff = {};
    for (const u of unknownMeta) {
      coeff[u.id] = momentOf(pointById(structure, u.point), about, u.ux, u.uy);
    }
    let knownMoment = 0;
    for (const pid of side) {
      const k = netKnownAt(pid);
      knownMoment += momentOf(pointById(structure, pid), about, k.fx, k.fy);
    }
    equations.push({ label: `ΣM about ${pointId} = 0 (section)`, coeff, rhs: -knownMoment });
  }

  const result = solveBySubstitution(unknownIds, equations);
  if (!result.solved) {
    return { solved: false, reason: result.reason, steps: result.steps };
  }

  const memberForces = unknownMeta.map((u) => {
    const force = result.values[u.id];
    return { id: u.id, force, state: Math.abs(force) < 1e-6 ? 'zero-force' : force > 0 ? 'tension' : 'compression' };
  });

  return {
    solved: true,
    memberForces,
    steps: result.steps,
    cutMemberIds: cutMembers.map((m) => m.id),
    freeBodyPoints: side,
  };
}

export { computeCut, solveSection };
