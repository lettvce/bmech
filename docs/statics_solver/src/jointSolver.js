import { memberVector, netLoadAt, buildReactionUnknowns } from './model.js';
import { solveSquare, EPS } from './linalg.js';

// Turn solved reaction magnitudes (each along a known direction) back into
// per-point (fx, fy) force vectors, so joints can treat reactions exactly
// like any other known external force.
function reactionForcesByPoint(structure, reactions) {
  const unknowns = buildReactionUnknowns(structure);
  const dirById = new Map(unknowns.map((u) => [u.id, u]));
  const byPoint = new Map();
  for (const r of reactions) {
    const u = dirById.get(r.id);
    const cur = byPoint.get(r.point) ?? { fx: 0, fy: 0 };
    cur.fx += u.dirX * r.value;
    cur.fy += u.dirY * r.value;
    byPoint.set(r.point, cur);
  }
  return byPoint;
}

// Method of joints, done the way a person does it: with reactions already
// known, repeatedly find a joint that currently has only 1 or 2 unsolved
// member forces (2 equations per joint can resolve at most 2 unknowns) and
// solve it directly. Falls back to "stuck" if some joint never drops to
// <=2 unknowns without others being solved first (e.g. non-simple trusses
// needing the method of sections / a global solve).
function solveJointsBySubstitution(structure, reactions) {
  const reactionByPoint = reactionForcesByPoint(structure, reactions);
  const members = structure.members;

  const incidence = new Map(structure.points.map((p) => [p.id, []]));
  for (const m of members) {
    const { ux, uy } = memberVector(structure, m);
    incidence.get(m.from).push({ memberId: m.id, coeffX: ux, coeffY: uy });
    incidence.get(m.to).push({ memberId: m.id, coeffX: -ux, coeffY: -uy });
  }

  function netKnownAt(pointId) {
    const load = netLoadAt(structure, pointId);
    const reaction = reactionByPoint.get(pointId) ?? { fx: 0, fy: 0 };
    return { fx: load.fx + reaction.fx, fy: load.fy + reaction.fy };
  }

  const solved = {};
  const steps = [];
  let progress = true;

  while (progress && Object.keys(solved).length < members.length) {
    progress = false;
    for (const p of structure.points) {
      const allTerms = incidence.get(p.id);
      const terms = allTerms.filter((t) => !(t.memberId in solved));
      if (terms.length === 0 || terms.length > 2) continue;

      const known = netKnownAt(p.id);
      let kx = -known.fx;
      let ky = -known.fy;
      for (const t of allTerms) {
        if (t.memberId in solved) {
          kx -= t.coeffX * solved[t.memberId];
          ky -= t.coeffY * solved[t.memberId];
        }
      }

      if (terms.length === 1) {
        const t = terms[0];
        const value = Math.abs(t.coeffX) >= Math.abs(t.coeffY) ? kx / t.coeffX : ky / t.coeffY;
        solved[t.memberId] = value;
        steps.push({ joint: p.id, solvesFor: [t.memberId], value: [value], note: 'only unknown member at this joint' });
        progress = true;
      } else {
        const [a, b] = terms;
        const A = [
          [a.coeffX, b.coeffX],
          [a.coeffY, b.coeffY],
        ];
        const det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (Math.abs(det) < EPS) continue; // both members collinear here — can't separate them yet
        const x = solveSquare(A, [kx, ky]);
        if (!x) continue;
        solved[a.memberId] = x[0];
        solved[b.memberId] = x[1];
        steps.push({
          joint: p.id,
          solvesFor: [a.memberId, b.memberId],
          value: x,
          note: 'exactly two unknown members at this joint, solved together',
        });
        progress = true;
      }
    }
  }

  if (Object.keys(solved).length < members.length) {
    return {
      solved: false,
      reason: 'Some joint never dropped to ≤ 2 unknown members — needs a global (matrix) solve.',
      steps,
    };
  }

  const memberForces = members.map((m) => ({
    id: m.id,
    force: solved[m.id],
    state: Math.abs(solved[m.id]) < EPS * 1e3 ? 'zero-force' : solved[m.id] > 0 ? 'tension' : 'compression',
  }));
  return { solved: true, memberForces, steps };
}

export { solveJointsBySubstitution, reactionForcesByPoint };
