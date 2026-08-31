import { solveSquare, EPS } from './linalg.js';

// Generic greedy substitution used everywhere in this app that isolates a
// free body and tests its equilibrium equations: repeatedly find whichever
// equation currently has exactly one unresolved unknown and solve it
// directly; fall back to a small simultaneous solve only when nothing ever
// isolates a single unknown on its own.
//
// `equations` is [{ label, coeff: { unknownId: number }, rhs }].
function solveBySubstitution(unknownIds, equations) {
  const solved = {};
  const steps = [];
  const used = new Set();

  while (Object.keys(solved).length < unknownIds.length) {
    let best = null;
    for (let i = 0; i < equations.length; i++) {
      if (used.has(i)) continue;
      const eq = equations[i];
      const remaining = unknownIds.filter((id) => !(id in solved) && Math.abs(eq.coeff[id] || 0) > EPS);
      if (remaining.length === 0) continue;
      if (!best || remaining.length < best.remaining.length) {
        best = { index: i, eq, remaining };
        if (remaining.length === 1) break;
      }
    }
    if (!best) break;

    if (best.remaining.length === 1) {
      const id = best.remaining[0];
      const known = Object.entries(solved).reduce((s, [k, v]) => s + (best.eq.coeff[k] || 0) * v, 0);
      const value = (best.eq.rhs - known) / best.eq.coeff[id];
      solved[id] = value;
      used.add(best.index);
      steps.push({ equation: best.eq.label, solvesFor: [id], value: [value], note: 'only unknown left in this equation' });
      continue;
    }

    const remainingIds = unknownIds.filter((id) => !(id in solved));
    const rows = [];
    for (let i = 0; i < equations.length; i++) {
      if (used.has(i)) continue;
      if (rows.length === remainingIds.length) break;
      const eq = equations[i];
      const row = remainingIds.map((id) => eq.coeff[id] || 0);
      if (row.every((c) => Math.abs(c) < EPS)) continue;
      const known = Object.entries(solved).reduce((s, [k, v]) => s + (eq.coeff[k] || 0) * v, 0);
      rows.push({ i, row, rhs: eq.rhs - known });
    }
    if (rows.length < remainingIds.length) break;

    const A = rows.map((r) => r.row);
    const b = rows.map((r) => r.rhs);
    const x = solveSquare(A, b);
    if (!x) break;

    remainingIds.forEach((id, idx) => {
      solved[id] = x[idx];
    });
    rows.forEach((r) => used.add(r.i));
    steps.push({
      equation: rows.map((r) => equations[r.i].label).join(' & '),
      solvesFor: remainingIds,
      value: remainingIds.map((id) => solved[id]),
      note: 'no single-unknown equation was available — solved simultaneously',
    });
  }

  if (Object.keys(solved).length < unknownIds.length) {
    return { solved: false, values: solved, steps, reason: 'Could not isolate all unknowns even with simultaneous solving.' };
  }
  return { solved: true, values: solved, steps };
}

export { solveBySubstitution };
