import { solveTruss } from './solver.js';

const EPS = 1e-6;

function* combinations(arr, k) {
  if (k === 0) {
    yield [];
    return;
  }
  for (let i = 0; i <= arr.length - k; i++) {
    for (const rest of combinations(arr.slice(i + 1), k - 1)) {
      yield [arr[i], ...rest];
    }
  }
}

// A cable member (member.type === 'cable') can only pull, never push — if
// equilibrium would require it to carry compression, it goes slack (force
// 0) instead, exactly like a real cable buckling out of the way. That's a
// genuinely different kind of problem than the rest of this solver: it's not
// "solve the linear system," it's "find which subset of cables can
// physically be taut at once" — a real cable, not a two-force member that
// happens to allow negative force.
//
// Two cables bracing the same panel (a classic X-brace) are often
// statically indeterminate as a pair — solving with both "active" doesn't
// even produce individual member forces, let alone let us check their sign.
// So this searches, smallest-removal-first, over every subset of cables to
// treat as slack, and returns the first subset whose remaining structure (a)
// solves to a unique determinate answer and (b) has every kept cable in
// real tension (force >= 0).
function solveTrussWithCables(structure) {
  const cables = structure.members.filter((m) => m.type === 'cable');
  if (cables.length === 0) {
    return solveTruss(structure);
  }

  const nonCableMembers = structure.members.filter((m) => m.type !== 'cable');

  for (let numRemoved = 0; numRemoved <= cables.length; numRemoved++) {
    for (const removedSet of combinations(cables, numRemoved)) {
      const removedIds = new Set(removedSet.map((c) => c.id));
      const activeMembers = [...nonCableMembers, ...cables.filter((c) => !removedIds.has(c.id))];
      const result = solveTruss({ ...structure, members: activeMembers });

      if (result.status !== 'determinate') continue;

      const keptCablesInTension = result.memberForces
        .filter((mf) => !removedIds.has(mf.id) && cables.some((c) => c.id === mf.id))
        .every((mf) => mf.force >= -EPS);
      if (!keptCablesInTension) continue;

      const memberForces = structure.members.map((m) => {
        if (removedIds.has(m.id)) return { id: m.id, force: 0, state: 'slack' };
        return result.memberForces.find((mf) => mf.id === m.id);
      });

      return { ...result, memberForces, removedCableIds: [...removedIds] };
    }
  }

  return {
    status: 'unstable',
    reason: 'No combination of taut/slack cables satisfies equilibrium — the structure needs a cable to push, which is impossible.',
  };
}

export { solveTrussWithCables };
