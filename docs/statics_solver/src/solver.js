import { memberVector, buildUnknownList, netLoadAt } from './model.js';
import { rank, solveSquare, EPS } from './linalg.js';
import { solveReactionsBySubstitution } from './reactionSolver.js';
import { solveJointsBySubstitution } from './jointSolver.js';

// Build the global equilibrium matrix A (2*numPoints x numUnknowns) and load
// vector b such that A*x = b, x = [memberForces..., reactionComponents...].
function buildSystem(structure) {
  const unknowns = buildUnknownList(structure);
  const points = structure.points;
  const rowOf = new Map(points.map((p, i) => [p.id, i]));

  const numEquations = 2 * points.length;
  const A = Array.from({ length: numEquations }, () => new Array(unknowns.length).fill(0));
  const b = new Array(numEquations).fill(0);

  // External loads go on the RHS: sum(unknowns' contributions) = -load
  for (const p of points) {
    const { fx, fy } = netLoadAt(structure, p.id);
    const r = rowOf.get(p.id);
    b[2 * r] = -fx;
    b[2 * r + 1] = -fy;
  }

  unknowns.forEach((u, col) => {
    if (u.kind === 'member') {
      const { ux, uy } = memberVector(structure, u.member);
      // Tension pulls each joint toward the other end of the member.
      const rFrom = rowOf.get(u.member.from);
      const rTo = rowOf.get(u.member.to);
      A[2 * rFrom][col] += ux;
      A[2 * rFrom + 1][col] += uy;
      A[2 * rTo][col] += -ux;
      A[2 * rTo + 1][col] += -uy;
    } else {
      const r = rowOf.get(u.point);
      A[2 * r][col] += u.dirX;
      A[2 * r + 1][col] += u.dirY;
    }
  });

  return { A, b, unknowns, numEquations };
}

// Classify and, if possible, solve. Returns:
//   { status: 'determinate', memberForces, reactions }
//   { status: 'indeterminate', degree }               // needs member stiffness, not solved here
//   { status: 'unstable', reason }                     // mechanism: equations don't fully constrain it
function solveTruss(structure) {
  const { A, b, unknowns, numEquations } = buildSystem(structure);
  const numUnknowns = unknowns.length;
  const rankA = rank(A, numUnknowns);

  if (numUnknowns > numEquations || rankA < numUnknowns) {
    if (rankA === numEquations) {
      // Consistent but underdetermined: more unknowns than independent
      // equations can pin down. Classic static indeterminacy.
      //
      // This is exactly what happens with a 'fixed' (welded) support: its
      // moment reaction never appears in any per-joint force equation (a
      // truss's pin joints can't transmit moment), so that unknown's column
      // is entirely zero — it can never be pinned down by member analysis.
      // Reactions themselves might still be individually solvable from pure
      // whole-body equilibrium even though member forces aren't, so surface
      // that when it's available rather than going silent.
      const reactionSteps = solveReactionsBySubstitution(structure);
      return {
        status: 'indeterminate',
        degree: numUnknowns - rankA,
        reactionSteps: reactionSteps.solved ? reactionSteps : undefined,
      };
    }
    // Rank-deficient equations too: the structure (or part of it) can move
    // as a mechanism regardless of how many unknowns it has.
    return {
      status: 'unstable',
      reason: 'Equilibrium equations are linearly dependent — the structure (or a sub-part of it) is a mechanism.',
    };
  }

  if (numUnknowns < numEquations) {
    return {
      status: 'unstable',
      reason: `Not enough unknowns (${numUnknowns}) for the number of equilibrium equations (${numEquations}) — under-braced structure.`,
    };
  }

  // numUnknowns === numEquations and full rank: a unique solution exists.
  // Get it two ways:
  //
  //  1. The matrix solve (below) — always correct, always available, this is
  //     the ground truth / fallback.
  //  2. The substitution pathway a person would actually use: solve external
  //     reactions first from whole-body equilibrium (only possible when there
  //     are exactly 3 reaction unknowns — otherwise reactions and member
  //     forces are coupled and can't be separated), then feed those known
  //     reactions into joint-by-joint substitution for member forces.
  //
  // The substitution pathway is preferred when it fully completes, since it
  // comes with a human-readable step log; the matrix result is the fallback
  // whenever either substitution phase gets stuck (externally indeterminate
  // reactions, or a joint that never drops to <=2 unknowns).
  const x = solveSquare(A, b);
  if (!x) {
    return { status: 'unstable', reason: 'Equilibrium matrix is singular despite matching counts (degenerate geometry).' };
  }

  const matrixMemberForces = [];
  const matrixReactions = [];
  unknowns.forEach((u, i) => {
    const value = x[i];
    if (u.kind === 'member') {
      matrixMemberForces.push({
        id: u.id,
        force: value,
        state: Math.abs(value) < EPS * 1e3 ? 'zero-force' : value > 0 ? 'tension' : 'compression',
      });
    } else {
      matrixReactions.push({ id: u.id, point: u.point, value });
    }
  });

  const reactionSteps = solveReactionsBySubstitution(structure);

  if (reactionSteps.solved) {
    const jointResult = solveJointsBySubstitution(structure, reactionSteps.reactions);
    if (jointResult.solved) {
      return {
        status: 'determinate',
        memberForces: jointResult.memberForces,
        reactions: reactionSteps.reactions,
        reactionSteps,
        jointSteps: jointResult.steps,
        method: 'substitution',
      };
    }
  }

  return {
    status: 'determinate',
    memberForces: matrixMemberForces,
    reactions: matrixReactions,
    reactionSteps,
    method: 'matrix',
  };
}

export { buildSystem, solveTruss };
