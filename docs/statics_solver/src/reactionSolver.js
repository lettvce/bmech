import { pointById, buildReactionUnknowns } from './model.js';
import { solveBySubstitution } from './substitutionSolver.js';

// Moment about `about` of a force (fx,fy) applied at `at`, CCW-positive.
function momentOf(at, about, fx, fy) {
  const rx = at.x - about.x;
  const ry = at.y - about.y;
  return rx * fy - ry * fx;
}

function totalLoadFxFy(structure) {
  let fx = 0;
  let fy = 0;
  for (const l of structure.loads) {
    fx += l.fx;
    fy += l.fy;
  }
  return { fx, fy };
}

function totalLoadMomentAbout(structure, about) {
  let m = 0;
  for (const l of structure.loads) {
    const p = pointById(structure, l.point);
    m += momentOf(p, about, l.fx, l.fy);
  }
  return m;
}

// Build the candidate equation pool for whole-body (external) equilibrium:
// sum Fx, sum Fy, and one moment equation about each distinct support point.
// Every candidate here only involves reaction unknowns — member forces are
// internal and cancel out of whole-body equilibrium entirely. Moment
// equations about a support point automatically "test well" in the shared
// substitution solver because every reaction acting AT that point drops out
// (zero moment arm) — so busy points (pins, where two unknown components
// meet) tend to leave the fewest unknowns standing.
function buildCandidateEquations(structure, unknowns) {
  const equations = [];

  const { fx: loadFx, fy: loadFy } = totalLoadFxFy(structure);
  equations.push({
    label: 'ΣFx = 0',
    coeff: Object.fromEntries(unknowns.map((u) => [u.id, u.dirX])),
    rhs: -loadFx,
  });
  equations.push({
    label: 'ΣFy = 0',
    coeff: Object.fromEntries(unknowns.map((u) => [u.id, u.dirY])),
    rhs: -loadFy,
  });

  const supportPointIds = [...new Set(unknowns.map((u) => u.point))];
  for (const pointId of supportPointIds) {
    const about = pointById(structure, pointId);
    const coeff = {};
    for (const u of unknowns) {
      // A pure couple (fixed support's moment reaction) has the same value
      // no matter what point you sum moments about — that's what makes it a
      // couple rather than a force with a moment arm.
      coeff[u.id] = u.isMoment ? 1 : momentOf(pointById(structure, u.point), about, u.dirX, u.dirY);
    }
    equations.push({
      label: `ΣM about ${pointId} = 0`,
      coeff,
      rhs: -totalLoadMomentAbout(structure, about),
    });
  }

  return equations;
}

function solveReactionsBySubstitution(structure) {
  const unknowns = buildReactionUnknowns(structure);
  const n = unknowns.length;

  if (n !== 3) {
    return {
      solved: false,
      reason: `${n} external reaction unknowns — whole-body equilibrium (3 equations) alone only pins down exactly 3. ` +
        (n > 3 ? 'Externally indeterminate; needs member/compatibility equations too.' : 'Under-restrained; likely a mechanism.'),
      steps: [],
    };
  }

  const equations = buildCandidateEquations(structure, unknowns);
  const result = solveBySubstitution(unknowns.map((u) => u.id), equations);

  if (!result.solved) {
    return { solved: false, reason: result.reason, steps: result.steps };
  }

  const reactions = unknowns.map((u) => ({ id: u.id, point: u.point, value: result.values[u.id] }));
  return { solved: true, reactions, steps: result.steps };
}

export { solveReactionsBySubstitution, buildCandidateEquations, momentOf };
