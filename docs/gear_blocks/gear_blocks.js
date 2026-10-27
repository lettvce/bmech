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
};

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

// --- Gear ---------------------------------------------------------------

export function buildGear(z, P, boreSegs = 96) {
  const d = derive(z, P);
  const outline = gearProfile(P.module, z, P.pressureAngle);
  const N = outline.length;
  const ang = unwrapAngles(outline);
  const h = P.face / 2;
  const verts = [];
  const tris = [];

  const ob = [], ot = [], ib = [], it = [];
  for (const [x, y] of outline) { ob.push(verts.length); verts.push([x, y, -h]); }
  for (const [x, y] of outline) { ot.push(verts.length); verts.push([x, y, h]); }
  const innerAng = [];
  for (let k = 0; k < boreSegs; k++) {
    const a = ang[0] + (2 * Math.PI * k) / boreSegs;
    innerAng.push(a);
    ib.push(verts.length); verts.push([d.rBore * Math.cos(a), d.rBore * Math.sin(a), -h]);
  }
  for (let k = 0; k < boreSegs; k++) {
    const a = innerAng[k];
    it.push(verts.length); verts.push([d.rBore * Math.cos(a), d.rBore * Math.sin(a), h]);
  }

  for (let k = 0; k < N; k++) {
    const k1 = (k + 1) % N;
    tris.push([ob[k], ob[k1], ot[k1]], [ob[k], ot[k1], ot[k]]);
  }
  for (let k = 0; k < boreSegs; k++) {
    const k1 = (k + 1) % boreSegs;
    tris.push([ib[k1], ib[k], it[k]], [ib[k1], it[k], it[k1]]);
  }
  zipAnnulus(tris, verts, ot, ang, it, innerAng, true);
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

export function buildBlock(z, P) {
  return mergeMeshes(buildFrame(z, P), buildGear(z, P));
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

// level: "err" blocks export, "warn" is advisory.
export function checkBlock(z, P) {
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
  return msgs;
}

// Per-block messages for a whole train: each block's own checks, plus
// stands that would overlap a neighbor (two small gears are closer
// together than one Stand Width).
export function checkTrain(teeth, P) {
  const msgs = teeth.map((z) => checkBlock(z, P));
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
