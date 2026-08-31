// Minimal dependency-free linear algebra: Gaussian elimination with partial
// pivoting, used both to solve square systems and to compute matrix rank for
// classifying a truss as determinate / indeterminate / unstable.

const EPS = 1e-9;

// Row-reduce a (possibly augmented) matrix to row-echelon form in place on a
// copy, returning the reduced matrix and the rank of the left `numCols`
// columns (pass the unaugmented column count to get rank(A); pass the full
// width to get rank([A|b])).
function rowEchelon(matrix, numCols) {
  const m = matrix.map((row) => row.slice());
  const numRows = m.length;
  let rank = 0;

  for (let col = 0; col < numCols && rank < numRows; col++) {
    let pivotRow = -1;
    let pivotVal = EPS;
    for (let r = rank; r < numRows; r++) {
      if (Math.abs(m[r][col]) > pivotVal) {
        pivotVal = Math.abs(m[r][col]);
        pivotRow = r;
      }
    }
    if (pivotRow === -1) continue; // no pivot in this column

    [m[rank], m[pivotRow]] = [m[pivotRow], m[rank]];

    for (let r = 0; r < numRows; r++) {
      if (r === rank) continue;
      const factor = m[r][col] / m[rank][col];
      if (factor === 0) continue;
      for (let c = col; c < m[r].length; c++) {
        m[r][c] -= factor * m[rank][c];
      }
    }
    rank++;
  }

  return { reduced: m, rank };
}

function rank(matrix, numCols) {
  if (matrix.length === 0) return 0;
  return rowEchelon(matrix, numCols ?? matrix[0].length).rank;
}

// Solve A x = b for a square, full-rank A. Returns null if A is singular.
function solveSquare(A, b) {
  const n = A.length;
  const augmented = A.map((row, i) => [...row, b[i]]);
  const { reduced, rank: r } = rowEchelon(augmented, n);
  if (r < n) return null;

  // reduced is now in row-echelon (diagonal-ish, not fully normalized) form
  // with each pivot row's leading entry in its own column thanks to full
  // pivoting-to-rank; solve by dividing each row by its pivot.
  const x = new Array(n).fill(0);
  for (let row = 0; row < n; row++) {
    const pivotCol = row; // rowEchelon puts pivot i in column i when rank === n
    x[pivotCol] = reduced[row][n] / reduced[row][pivotCol];
  }
  return x;
}

export { rowEchelon, rank, solveSquare, EPS };
