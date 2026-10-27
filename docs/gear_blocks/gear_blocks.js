// Pure geometry for the Gear Blocks page - no Three.js, no DOM, so the
// same file runs in the browser and under plain `node` for the headless
// checks in dev/. All lengths in mm.
//
// Block frame of reference (also the exported STL's print orientation,
// matching demo_gear_for_kids.blend): axle along Z, gear centered on the
// origin (z in [-face/2, face/2]), back wall below at y = -height. In the
// train view the back wall sits on the table, so Y is simply "up".

// --- Spur profile: ported 1:1 from bmech's spur_gear_core.py -----------
// (build_tooth_profile / build_gear_profile) so a block gear is the same
// outline the Blender spur generator makes. Tooth 0 is centered on +X.

const ADDENDUM_COEFF = 1.0;
const DEDENDUM_COEFF = 1.25;
const INVOLUTE_POINTS = 15;
const SPACE_PTS = 4;

function involutePoint(rb, t) {
  return [rb * (Math.cos(t) + t * Math.sin(t)), rb * (Math.sin(t) - t * Math.cos(t))];
}

function involuteAngleAtRadius(rb, r) {
  const ratio = r / rb;
  return ratio < 1 ? 0 : Math.sqrt(ratio * ratio - 1);
}

function rot([x, y], a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [x * c - y * s, x * s + y * c];
}

function toothProfile(m, z, paDeg) {
  const pa = (paDeg * Math.PI) / 180;
  const rp = (m * z) / 2;
  const rb = rp * Math.cos(pa);
  const ra = rp + ADDENDUM_COEFF * m;
  const rf = rp - DEDENDUM_COEFF * m;
  const halfTooth = Math.PI / (2 * z);

  const tStart = involuteAngleAtRadius(rb, Math.max(rf, rb));
  const tTip = involuteAngleAtRadius(rb, ra);
  const raw = [];
  for (let i = 0; i < INVOLUTE_POINTS; i++) {
    raw.push(involutePoint(rb, tStart + ((tTip - tStart) * i) / (INVOLUTE_POINTS - 1)));
  }
  const [px, py] = involutePoint(rb, involuteAngleAtRadius(rb, rp));
  const rotation = halfTooth + Math.atan2(py, px);

  let right = raw.map(([x, y]) => rot([x, -y], rotation));
  if (rb > rf) {
    const a = Math.atan2(right[0][1], right[0][0]);
    right = [[rf * Math.cos(a), rf * Math.sin(a)], ...right];
  }
  const left = right.map(([x, y]) => [x, -y]);
  return [...left, [ra, 0], ...right.slice().reverse()];
}

export function gearProfile(m, z, paDeg) {
  const pa = (paDeg * Math.PI) / 180;
  const rp = (m * z) / 2;
  const rb = rp * Math.cos(pa);
  const rf = rp - DEDENDUM_COEFF * m;
  const halfTooth = Math.PI / (2 * z);
  const pitchAng = (2 * Math.PI) / z;

  const [px, py] = involutePoint(rb, involuteAngleAtRadius(rb, rp));
  const rotation = halfTooth + Math.atan2(py, px);
  const rootLocal = involutePoint(rb, involuteAngleAtRadius(rb, Math.max(rf, rb)));
  const rootRot = rot([rootLocal[0], -rootLocal[1]], rotation);
  const rootR = Math.atan2(rootRot[1], rootRot[0]);
  const rootL = -rootR;

  const tooth = toothProfile(m, z, paDeg);
  const out = [];
  for (let i = 0; i < z; i++) {
    const ta = i * pitchAng;
    for (const p of tooth) out.push(rot(p, ta));
    const a0 = ta + rootR, a1 = ta + pitchAng + rootL;
    for (let j = 1; j <= SPACE_PTS; j++) {
      const a = a0 + ((a1 - a0) * j) / (SPACE_PTS + 1);
      out.push([rf * Math.cos(a), rf * Math.sin(a)]);
    }
  }
  return out;
}

// --- Parameters -------------------------------------------------------

export const DEFAULTS = {
  module: 3.6,
  pressureAngle: 20,
  face: 20,            // gear thickness
  height: 110,         // axle center -> inner face of the back wall
  depth: 100,          // outer span of the frame along the axle
  plate: 3.2,          // top/bottom plate thickness
  wall: 3.2,           // back wall thickness
  neckD: 30,           // axle diameter through the gear
  boreClear: 0.3,      // radial gear-bore clearance (print-in-place)
  shoulderStep: 1,     // shoulder radius above the neck (the captive lip)
  shoulderGap: 0.3,    // axial gap from gear face to where the 45° shoulder starts
  capR: 20,            // plate boss radius around the axle
  wedgeDeg: 30,        // included angle of the plate taper
  lip: 3,              // plate overhang past the wall face
  meshClear: 1,        // added to the theoretical center distance
  standWidth: 80,      // same for every block; the gap between stands varies
  pinR: 6,             // crank pin radius (drives a Geneva / yoke)
  pinH: 15,            // pin height above the gear face
  pinRound: 2,         // round-over on the pin top
};

const PIN_MARGIN = 1.5;  // pin wall to the edges of its face cell
const PIN_CLEAR = 0.5;   // pin wall to the axle shoulder (print-in-place)
const FACE_EDGE = 1.0;   // target edge length on the pin gear's top face

// Allowed pin offsets (axle to pin center) for a gear: outside the bore
// and the axle shoulder, inside the tooth roots with room for the cell.
export function pinRange(z, P) {
  const d = derive(z, P);
  const lo = Math.max(d.rBore + P.pinR + PIN_MARGIN, d.rShoulder + PIN_CLEAR + P.pinR);
  const hi = d.rRoot - 0.5 - PIN_MARGIN - P.pinR;
  return { lo, hi, ok: hi >= lo };
}

export function clampPinOffset(z, P, off) {
  const { lo, hi } = pinRange(z, P);
  return Math.min(Math.max(off, lo), hi);
}

export function derive(z, P) {
  const rp = (P.module * z) / 2;
  const rTip = rp + ADDENDUM_COEFF * P.module;
  const rRoot = rp - DEDENDUM_COEFF * P.module;
  const rNeck = P.neckD / 2;
  return {
    z, rp, rTip, rRoot, rNeck,
    rBore: rNeck + P.boreClear,
    rShoulder: rNeck + P.shoulderStep,
    baseWidth: P.standWidth,
  };
}

// --- Mesh helpers -----------------------------------------------------

function unwrapAngles(pts) {
  const out = [];
  let prev = null;
  for (const [x, y] of pts) {
    const a = Math.atan2(y, x);
    if (prev === null) { out.push(a); prev = a; continue; }
    let d = a - prev;
    d -= 2 * Math.PI * Math.round(d / (2 * Math.PI));
    prev += d;
    out.push(prev);
  }
  return out;
}

// Triangulates the flat ring between a star-shaped outer loop and a
// circular inner loop by walking both in angle order (both CCW, inner
// loop starting at the outer loop's first bearing). up=true -> +Z normal.
// Angle order alone isn't enough: tooth flanks run almost radially, so
// the angle-preferred step can be a clockwise (folded) sliver when the
// next bore point sits just behind in angle - then the other step is
// taken. This also covers the zero-area case where a bore point shares
// the bearing the profile repeats at each flank/root junction.
function zipAnnulus(tris, verts, outerIdx, outerAng, innerIdx, innerAng, up) {
  const N = outerIdx.length, M = innerIdx.length;
  const A = [...outerAng, outerAng[0] + 2 * Math.PI];
  const B = [...innerAng, innerAng[0] + 2 * Math.PI];
  const ccw = ([a, b, c]) => {
    const [ax, ay] = verts[a], [bx, by] = verts[b], [cx, cy] = verts[c];
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 1e-12;
  };
  let i = 0, j = 0;
  while (i < N || j < M) {
    const tOut = i < N ? [outerIdx[i], outerIdx[(i + 1) % N], innerIdx[j % M]] : null;
    const tIn = j < M ? [outerIdx[i % N], innerIdx[(j + 1) % M], innerIdx[j]] : null;
    let takeOuter = !tIn || (tOut && A[i + 1] <= B[j + 1]);
    if (takeOuter && tIn && !ccw(tOut) && ccw(tIn)) takeOuter = false;
    else if (!takeOuter && tOut && !ccw(tIn) && ccw(tOut)) takeOuter = true;
    const t = takeOuter ? tOut : tIn;
    if (takeOuter) i++; else j++;
    tris.push(up ? t : [t[0], t[2], t[1]]);
  }
}

// zipAnnulus for any two CCW loops that are both star-shaped about
// `center` (not just the axle), with no assumption about where either
// loop starts. Used for the pin's face cell.
function zipStar(tris, verts, outerIdx, innerIdx, [cx, cy], up) {
  const TAU = 2 * Math.PI;
  const angs = (loop, start) => {
    const out = [];
    let prev = null;
    for (const i of loop) {
      let a = Math.atan2(verts[i][1] - cy, verts[i][0] - cx);
      if (prev === null) a = start + ((((a - start) % TAU) + TAU) % TAU);
      else a = prev + ((((a - prev + Math.PI) % TAU) + TAU) % TAU) - Math.PI;
      out.push(a); prev = a;
    }
    return out;
  };
  const A = angs(outerIdx, -Math.PI);
  // start the inner loop at its first point at/after outer[0]'s bearing
  const lag = (i) => {
    const a = Math.atan2(verts[i][1] - cy, verts[i][0] - cx) - A[0];
    return ((a % TAU) + TAU) % TAU;
  };
  let k0 = 0;
  for (let k = 1; k < innerIdx.length; k++) if (lag(innerIdx[k]) < lag(innerIdx[k0])) k0 = k;
  const inner = [...innerIdx.slice(k0), ...innerIdx.slice(0, k0)];
  zipAnnulus(tris, verts, outerIdx, A, inner, angs(inner, A[0]), up);
}

// Pin gear top face, without hole-filling: split into cells that are
// each star-shaped about some center, so every piece is a zip.
//   outer band : tooth outline <-> mid circle            (about the axle)
//   inner strip: bore arc <-> mid arc, outside the cell  (quads)
//   pin cell   : wedge from bore to mid circle <-> pin ring (about the pin)
// The bore and mid circles share one angular grid (starting at the
// outline's first bearing, as zipAnnulus expects); the cell's side edges
// sit on grid lines and are subdivided to the face spacing, and the
// strip faces beside them reuse those points so nothing T-junctions.
function pinTopFace(verts, tris, d, P, off, ot, ang, h) {
  const TAU = 2 * Math.PI;
  const rMid = (off + P.pinR + d.rRoot) / 2;
  const alpha = Math.asin((P.pinR + PIN_MARGIN) / off);
  const K = Math.max(96, Math.ceil((TAU * rMid) / FACE_EDGE));
  const grid = [];
  for (let k = 0; k < K; k++) grid.push(ang[0] + (TAU * k) / K);
  const ring = (r, zz) => grid.map((a) => { verts.push([r * Math.cos(a), r * Math.sin(a), zz]); return verts.length - 1; });
  const bt = ring(d.rBore, h), mt = ring(rMid, h);
  zipAnnulus(tris, verts, ot, ang, mt, grid, true);

  // Cell spans grid steps kA -> kB (CCW) around the pin at bearing 0.
  const rel = (k) => Math.atan2(Math.sin(grid[k]), Math.cos(grid[k]));
  let kA = -1, kB = -1;
  for (let k = 0; k < K; k++) {
    if (rel(k) <= -alpha && (kA < 0 || rel(k) > rel(kA))) kA = k;
    if (rel(k) >= alpha && (kB < 0 || rel(k) < rel(kB))) kB = k;
  }
  const nR = Math.max(2, Math.ceil((rMid - d.rBore) / FACE_EDGE));
  const radial = (k) => {
    const out = [];
    for (let t = 1; t < nR; t++) {
      const r = d.rBore + ((rMid - d.rBore) * t) / nR;
      verts.push([r * Math.cos(grid[k]), r * Math.sin(grid[k]), h]);
      out.push(verts.length - 1);
    }
    return out;   // runs outward
  };
  const radA = radial(kA), radB = radial(kB);

  for (let k = kB; k !== kA; k = (k + 1) % K) {
    const k1 = (k + 1) % K;
    if (k === kB) {
      // [bt k, radB.., mt k, mt k1, bt k1], fanned from bt k1 (off radB's line)
      const poly = [bt[k], ...radB, mt[k], mt[k1]];
      for (let i = 0; i + 1 < poly.length; i++) tris.push([bt[k1], poly[i], poly[i + 1]]);
    } else if (k1 === kA) {
      // [bt k, mt k, mt k1, radA reversed.., bt k1], fanned from bt k
      const poly = [mt[k], mt[k1], ...radA.slice().reverse(), bt[k1]];
      for (let i = 0; i + 1 < poly.length; i++) tris.push([bt[k], poly[i], poly[i + 1]]);
    } else {
      tris.push([bt[k], mt[k], mt[k1]], [bt[k], mt[k1], bt[k1]]);
    }
  }

  const cell = [];
  for (let k = kA; ; k = (k + 1) % K) { cell.push(bt[k]); if (k === kB) break; }
  cell.push(...radB);
  for (let k = kB; ; k = (k - 1 + K) % K) { cell.push(mt[k]); if (k === kA) break; }
  cell.push(...radA.slice().reverse());
  let area2 = 0;
  for (let i = 0; i < cell.length; i++) {
    const [x1, y1] = verts[cell[i]], [x2, y2] = verts[cell[(i + 1) % cell.length]];
    area2 += x1 * y2 - x2 * y1;
  }
  if (area2 < 0) cell.reverse();

  const S = Math.max(24, Math.ceil((TAU * P.pinR) / FACE_EDGE));
  const pinRing = [];
  for (let s = 0; s < S; s++) {
    const a = (TAU * s) / S;
    verts.push([off + P.pinR * Math.cos(a), P.pinR * Math.sin(a), h]);
    pinRing.push(verts.length - 1);
  }
  zipStar(tris, verts, cell, pinRing, [off, 0], true);
  buildPinBody(verts, tris, P, off, pinRing, h);
  return { bt, grid };
}

// Straight wall, quarter-round top edge, flat top - lathed from the
// face's own pin ring so the pin is welded into the gear by construction.
function buildPinBody(verts, tris, P, off, ring0, h) {
  const S = ring0.length;
  const rr = Math.min(P.pinRound, P.pinR * 0.9, P.pinH * 0.9);
  const zr = h + P.pinH - rr;
  const SEGS = 6;
  const prof = [[P.pinR, zr]];
  for (let k = 1; k <= SEGS; k++) {
    const a = (Math.PI / 2) * (k / SEGS);
    prof.push([P.pinR - rr + rr * Math.cos(a), zr + rr * Math.sin(a)]);
  }
  let prev = ring0;
  for (const [r, zz] of prof) {
    const cur = [];
    for (let s = 0; s < S; s++) {
      const a = (2 * Math.PI * s) / S;
      verts.push([off + r * Math.cos(a), r * Math.sin(a), zz]);
      cur.push(verts.length - 1);
    }
    for (let s = 0; s < S; s++) {
      const s1 = (s + 1) % S;
      tris.push([prev[s], prev[s1], cur[s1]], [prev[s], cur[s1], cur[s]]);
    }
    prev = cur;
  }
  verts.push([off, 0, h + P.pinH]);
  const pole = verts.length - 1;
  for (let s = 0; s < S; s++) tris.push([pole, prev[s], prev[(s + 1) % S]]);
}

// --- Gear ---------------------------------------------------------------

// pinOff: null for a plain gear, else the crank pin's offset from the
// axle (clamped into pinRange). The pin stands on the +Z face at +X.
export function buildGear(z, P, pinOff = null, boreSegs = 96) {
  const d = derive(z, P);
  const outline = gearProfile(P.module, z, P.pressureAngle);
  const N = outline.length;
  const ang = unwrapAngles(outline);
  const h = P.face / 2;
  const verts = [];
  const tris = [];
  const withPin = pinOff !== null && pinRange(z, P).ok;

  const ob = [], ot = [];
  for (const [x, y] of outline) { ob.push(verts.length); verts.push([x, y, -h]); }
  for (const [x, y] of outline) { ot.push(verts.length); verts.push([x, y, h]); }
  for (let k = 0; k < N; k++) {
    const k1 = (k + 1) % N;
    tris.push([ob[k], ob[k1], ot[k1]], [ob[k], ot[k1], ot[k]]);
  }

  let it, innerAng;
  if (withPin) {
    const top = pinTopFace(verts, tris, d, P, clampPinOffset(z, P, pinOff), ot, ang, h);
    it = top.bt; innerAng = top.grid;
  } else {
    innerAng = [];
    for (let k = 0; k < boreSegs; k++) innerAng.push(ang[0] + (2 * Math.PI * k) / boreSegs);
    it = innerAng.map((a) => { verts.push([d.rBore * Math.cos(a), d.rBore * Math.sin(a), h]); return verts.length - 1; });
    zipAnnulus(tris, verts, ot, ang, it, innerAng, true);
  }
  const ib = innerAng.map((a) => { verts.push([d.rBore * Math.cos(a), d.rBore * Math.sin(a), -h]); return verts.length - 1; });
  for (let k = 0; k < ib.length; k++) {
    const k1 = (k + 1) % ib.length;
    tris.push([ib[k1], ib[k], it[k]], [ib[k1], it[k], it[k1]]);
  }
  zipAnnulus(tris, verts, ob, ang, ib, innerAng, false);
  return { verts, tris };
}

// --- Frame: C-bracket loft + axle ---------------------------------------

// Upper edge of the plate outline at x: the boss arc, then the two
// tangent lines of the wedge, floored at the lip line past the wall.
function plateEdgeY(x, P) {
  const half = ((P.wedgeDeg / 2) * Math.PI) / 180;
  const rc = P.capR;
  const xt = rc * Math.cos(half);
  const ax = Math.abs(x);
  let y;
  if (ax <= xt) y = Math.sqrt(Math.max(rc * rc - ax * ax, 0));
  else y = rc * Math.sin(half) - (ax - xt) / Math.tan(half);
  return Math.max(y, -P.height + P.lip);
}

function buildBracket(z, P, verts, tris) {
  const d = derive(z, P);
  const W2 = d.baseWidth / 2;
  const half = ((P.wedgeDeg / 2) * Math.PI) / 180;
  const rc = P.capR;
  const xt = rc * Math.cos(half);
  const xLip = xt + (rc * Math.sin(half) + P.height - P.lip) * Math.tan(half);

  // Stations along X: plate ends, wedge kinks, and the boss arc.
  const xs = new Set([-W2, W2]);
  for (const k of [xt, xLip]) { if (k < W2) { xs.add(k); xs.add(-k); } }
  const ARC = 32;
  for (let i = 0; i <= ARC; i++) {
    const x = -xt + (2 * xt * i) / ARC;
    if (Math.abs(x) < W2) xs.add(x);
  }
  const stations = [...xs].sort((a, b) => a - b);

  const Zo = P.depth / 2, Zi = Zo - P.plate;
  const yb = -P.height - P.wall, yw = -P.height;
  const rings = stations.map((x) => {
    const ym = plateEdgeY(x, P);
    const sec = [[yb, -Zo], [ym, -Zo], [ym, -Zi], [yw, -Zi], [yw, Zi], [ym, Zi], [ym, Zo], [yb, Zo]];
    return sec.map(([y, zz]) => { verts.push([x, y, zz]); return verts.length - 1; });
  });
  for (let s = 0; s < rings.length - 1; s++) {
    const r0 = rings[s], r1 = rings[s + 1];
    for (let k = 0; k < 8; k++) {
      const k1 = (k + 1) % 8;
      tris.push([r0[k], r0[k1], r1[k1]], [r0[k], r1[k1], r1[k]]);
    }
  }
  const capTris = [[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 4, 7], [4, 5, 6], [4, 6, 7]];
  const last = rings[rings.length - 1], first = rings[0];
  for (const [a, b, c] of capTris) {
    tris.push([last[a], last[b], last[c]]);
    tris.push([first[a], first[c], first[b]]);
  }
}

// Solid of revolution: shoulder from plate to plate, necked through the
// gear with 45° chamfers. Runs 1 mm into each plate - a separate closed
// shell that overlaps the bracket (slicers union overlapping shells).
function buildAxle(z, P, verts, tris, segs = 96) {
  const d = derive(z, P);
  const Zi = P.depth / 2 - P.plate;
  const zEnd = Zi + 1;
  const zNeck = P.face / 2 + P.shoulderGap;
  const zSh = zNeck + P.shoulderStep;
  const prof = [
    [d.rShoulder, -zEnd], [d.rShoulder, -zSh], [d.rNeck, -zNeck],
    [d.rNeck, zNeck], [d.rShoulder, zSh], [d.rShoulder, zEnd],
  ];
  const rings = prof.map(([r, zz]) => {
    const ring = [];
    for (let k = 0; k < segs; k++) {
      const a = (2 * Math.PI * k) / segs;
      ring.push(verts.length);
      verts.push([r * Math.cos(a), r * Math.sin(a), zz]);
    }
    return ring;
  });
  for (let s = 0; s < rings.length - 1; s++) {
    const r0 = rings[s], r1 = rings[s + 1];
    for (let k = 0; k < segs; k++) {
      const k1 = (k + 1) % segs;
      tris.push([r0[k], r0[k1], r1[k1]], [r0[k], r1[k1], r1[k]]);
    }
  }
  const cb = verts.length; verts.push([0, 0, -zEnd]);
  const ct = verts.length; verts.push([0, 0, zEnd]);
  const bot = rings[0], top = rings[rings.length - 1];
  for (let k = 0; k < segs; k++) {
    const k1 = (k + 1) % segs;
    tris.push([cb, bot[k1], bot[k]]);
    tris.push([ct, top[k], top[k1]]);
  }
}

export function buildFrame(z, P) {
  const verts = [], tris = [];
  buildBracket(z, P, verts, tris);
  buildAxle(z, P, verts, tris);
  return { verts, tris };
}

export function mergeMeshes(...meshes) {
  const verts = [], tris = [];
  for (const m of meshes) {
    const off = verts.length;
    verts.push(...m.verts);
    for (const [a, b, c] of m.tris) tris.push([a + off, b + off, c + off]);
  }
  return { verts, tris };
}

export function buildBlock(z, P, pinOff = null) {
  return mergeMeshes(buildFrame(z, P), buildGear(z, P, pinOff));
}

// --- Train layout -------------------------------------------------------

// Blocks stand along +X at center distance rp_i + rp_j + meshClear; the
// gap between their stands is whatever that leaves after standWidth. Gear phases are chosen so a tooth of each
// gear sits in a gap of the previous one (tooth 0 is on +X at angle 0);
// ratio is turns of this gear per turn of the first, signed.
export function layoutTrain(teeth, P) {
  const out = [];
  let x = 0;
  for (let i = 0; i < teeth.length; i++) {
    const d = derive(teeth[i], P);
    if (i > 0) {
      const prev = out[i - 1];
      x += prev.d.rp + d.rp + P.meshClear;
      // Contact point is at bearing 0 from prev, PI from this gear.
      const uPrev = (prev.d.z * (0 - prev.phase)) / (2 * Math.PI);
      const phase = Math.PI - (2 * Math.PI * (0.5 - uPrev)) / d.z;
      const ratio = (-prev.ratio * prev.d.z) / d.z;
      out.push({ x, d, phase, ratio });
    } else {
      out.push({ x, d, phase: 0, ratio: 1 });
    }
  }
  return out;
}

// --- Checks -------------------------------------------------------------

// Fewest teeth that leave any room for the crank pin.
export function minPinTeeth(P) {
  for (let z = 8; z < 400; z++) if (pinRange(z, P).ok) return z;
  return null;
}

// level: "err" blocks export, "warn" is advisory. pin: offset or null.
export function checkBlock(z, P, pin = null) {
  const d = derive(z, P);
  const msgs = [];
  if (z < 8) msgs.push({ level: "err", text: `${z}T is too few teeth (min 8).` });
  else if (z < 17) msgs.push({ level: "warn", text: `${z}T will undercut at ${P.pressureAngle}° (17+ avoids it).` });
  if (d.rRoot - d.rBore < 2.5) {
    msgs.push({ level: "err", text: `Bore leaves only ${(d.rRoot - d.rBore).toFixed(1)} mm under the teeth (need 2.5).` });
  }
  if (P.height - d.rTip < 2) {
    msgs.push({ level: "err", text: `Gear tip (r ${d.rTip.toFixed(1)}) hits the back wall - use fewer teeth, or raise Stand Height (Advanced) to ${Math.ceil(d.rTip + 2)}+.` });
  }
  if (d.baseWidth / 2 < P.capR + 1) {
    msgs.push({ level: "err", text: `Stand is narrower than the axle boss - widen Stand Width or shrink Boss Radius.` });
  }
  if (P.capR < d.rShoulder + 2) {
    msgs.push({ level: "err", text: `Boss Radius must be at least ${(d.rShoulder + 2).toFixed(1)} to hold the shoulder.` });
  }
  const Zi = P.depth / 2 - P.plate;
  if (Zi < P.face / 2 + P.shoulderGap + P.shoulderStep + 1) {
    msgs.push({ level: "err", text: `Stand Depth is too shallow for the gear and shoulders.` });
  }
  if (pin !== null) {
    if (!pinRange(z, P).ok) {
      const n = minPinTeeth(P);
      msgs.push({ level: "err", text: `${z}T has no room for a crank pin${n ? ` - use ${n}+ teeth` : ""}.` });
    }
    if (P.face / 2 + P.pinH > Zi - 2) {
      msgs.push({ level: "err", text: `Crank pin reaches the stand plate - lower Pin Height to ${Math.floor(Zi - 2 - P.face / 2)} or less.` });
    }
  }
  return msgs;
}

// Per-block messages for a whole train: each block's own checks, plus
// stands that would overlap a neighbor (two small gears are closer
// together than one Stand Width). Pins never need a neighbor check: each
// stays inside its own root circle, and meshing root circles never touch.
export function checkTrain(teeth, P, pins = []) {
  const msgs = teeth.map((z, i) => checkBlock(z, P, pins[i] ?? null));
  const L = layoutTrain(teeth, P);
  for (let i = 0; i + 1 < L.length; i++) {
    const gap = L[i + 1].x - L[i].x - P.standWidth;
    if (gap < 0) {
      const text = `Stands ${i + 1} and ${i + 2} overlap by ${(-gap).toFixed(1)} mm - use more teeth or a narrower Stand Width.`;
      msgs[i].push({ level: "err", text });
      msgs[i + 1].push({ level: "err", text });
    }
  }
  return msgs;
}

// --- STL ----------------------------------------------------------------

function faceNormal(a, b, c) {
  const ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
  const vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const len = Math.hypot(nx, ny, nz) || 1;
  return [nx / len, ny / len, nz / len];
}

// Binary STL: 80-byte header, uint32 count, 50 bytes per triangle.
export function buildBinarySTL(verts, tris) {
  const buffer = new ArrayBuffer(84 + 50 * tris.length);
  const view = new DataView(buffer);
  view.setUint32(80, tris.length, true);
  let o = 84;
  for (const [i0, i1, i2] of tris) {
    const a = verts[i0], b = verts[i1], c = verts[i2];
    for (const n of faceNormal(a, b, c)) { view.setFloat32(o, n, true); o += 4; }
    for (const v of [a, b, c]) for (const q of v) { view.setFloat32(o, q, true); o += 4; }
    o += 2;
  }
  return buffer;
}
