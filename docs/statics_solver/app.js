import { solveTrussWithCables } from './src/cableSolver.js';
import { solveSection } from './src/sectionSolver.js';

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const resultEl = document.getElementById('result');
const toolHintEl = document.getElementById('toolHint');

// --- world <-> screen transform (world: meters, y-up; screen: pixels, y-down) ---
const view = { scale: 60, originX: 0, originY: 0 }; // originX/Y set on resize (screen px of world 0,0)

function resize() {
  canvas.width = canvas.clientWidth * devicePixelRatio;
  canvas.height = canvas.clientHeight * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  view.originX = canvas.clientWidth / 2;
  view.originY = canvas.clientHeight * 0.7;
  draw();
}
window.addEventListener('resize', resize);

function worldToScreen(x, y) {
  return { x: view.originX + x * view.scale, y: view.originY - y * view.scale };
}
function screenToWorld(x, y) {
  return { x: (x - view.originX) / view.scale, y: (view.originY - y) / view.scale };
}

// --- pan the canvas with middle-mouse-button drag ---
let panState = null; // { startX, startY, startOriginX, startOriginY }

canvas.addEventListener('mousedown', (e) => {
  if (e.button !== 1) return; // middle button only
  e.preventDefault(); // stop the browser's autoscroll icon from taking over
  panState = { startX: e.clientX, startY: e.clientY, startOriginX: view.originX, startOriginY: view.originY };
  canvas.style.cursor = 'grabbing';
});

window.addEventListener('mousemove', (e) => {
  if (!panState) return;
  view.originX = panState.startOriginX + (e.clientX - panState.startX);
  view.originY = panState.startOriginY + (e.clientY - panState.startY);
  draw();
});

window.addEventListener('mouseup', (e) => {
  if (e.button !== 1 || !panState) return;
  panState = null;
  canvas.style.cursor = '';
});

// Middle-click's default action (autoscroll) can still fire from a plain
// mousedown in some browsers unless the resulting auxclick is suppressed too.
canvas.addEventListener('auxclick', (e) => {
  if (e.button === 1) e.preventDefault();
});

// --- zoom with the scroll wheel, centered on the cursor ---
const MIN_SCALE = 5;
const MAX_SCALE = 800;

canvas.addEventListener(
  'wheel',
  (e) => {
    e.preventDefault(); // stop the page itself from scrolling
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const worldBefore = screenToWorld(mx, my);

    const factor = Math.exp(-e.deltaY * 0.001);
    view.scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, view.scale * factor));

    // Re-anchor the origin so the world point under the cursor doesn't
    // drift — the same point stays under the mouse as it zooms.
    view.originX = mx - worldBefore.x * view.scale;
    view.originY = my + worldBefore.y * view.scale;
    draw();
  },
  { passive: false }
);

// --- click-and-drag to connect two points with a member/cable ---
// Layered on top of the existing click-click flow rather than replacing it:
// a mousedown on a point just remembers it; if the mouse never actually
// moves before mouseup, nothing here fires and the browser's normal
// synthetic 'click' event runs the old two-click logic untouched. Only a
// real drag (past a small threshold) creates the member directly, and it
// suppresses that one following click so the old handler doesn't also fire.
let dragFrom = null;
let dragStartScreen = null;
let dragCurrentScreen = null;
let dragMoved = false;
let suppressNextClick = false;
const DRAG_THRESHOLD = 4; // px

canvas.addEventListener('mousedown', (e) => {
  if (e.button !== 0) return; // left button only — middle is reserved for pan
  if (tool !== 'member' && tool !== 'cable') return;
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;
  const p = findPointNear(sx, sy);
  if (!p) return;
  dragFrom = p;
  dragStartScreen = { x: sx, y: sy };
  dragCurrentScreen = dragStartScreen;
  dragMoved = false;
});

window.addEventListener('mousemove', (e) => {
  if (!dragFrom) return;
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;
  dragCurrentScreen = { x: sx, y: sy };
  if (!dragMoved && Math.hypot(sx - dragStartScreen.x, sy - dragStartScreen.y) > DRAG_THRESHOLD) {
    dragMoved = true;
  }
  if (dragMoved) draw();
});

window.addEventListener('mouseup', (e) => {
  if (e.button !== 0 || !dragFrom) return;
  if (dragMoved) {
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const target = findPointNear(sx, sy);
    if (target && target.id !== dragFrom.id) {
      pushHistory();
      const newMember = { id: `M${nextMemberNum++}`, from: dragFrom.id, to: target.id };
      if (tool === 'cable') newMember.type = 'cable';
      members.push(newMember);
      lastResult = null;
      resultEl.textContent = '';
      renderLists();
    }
    suppressNextClick = true; // this drag's mouseup will still fire a native 'click' next
  }
  dragFrom = null;
  dragStartScreen = null;
  dragCurrentScreen = null;
  dragMoved = false;
  draw();
});

// --- model state ---
let points = []; // {id, x, y}
let members = []; // {id, from, to}
let supports = []; // {point, type, direction}
let loads = []; // {id, point, fx, fy}
let nextPointNum = 1;
let nextMemberNum = 1;
let nextLoadNum = 1;

let lastResult = null; // solveTrussWithCables() output, used for coloring/labels
let pendingSectionPoint = null; // world point of the first click while drawing a cutting line
let sectionLine = null; // { p1, p2 } world points, extended to an infinite line for cutting
let sectionResult = null; // solveSection() output

// --- persistence: survive a page refresh via localStorage ---
// Versioned key so a future schema change can't load garbage into a stale
// structure instead of just starting fresh.
const STORAGE_KEY = 'statics_solver:v1';
let saveTimer = null;

function scheduleSave() {
  clearTimeout(saveTimer);
  saveTimer = setTimeout(() => {
    try {
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({ points, members, supports, loads, nextPointNum, nextMemberNum, nextLoadNum })
      );
    } catch {
      // Storage unavailable (private browsing, quota, disabled) — the
      // canvas still works fine, it just won't survive a refresh.
    }
  }, 250);
}

function loadFromStorage() {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return false;
    const s = JSON.parse(raw);
    if (!s || !Array.isArray(s.points)) return false;
    points = s.points ?? [];
    members = s.members ?? [];
    supports = s.supports ?? [];
    loads = s.loads ?? [];
    nextPointNum = s.nextPointNum ?? points.length + 1;
    nextMemberNum = s.nextMemberNum ?? members.length + 1;
    nextLoadNum = s.nextLoadNum ?? loads.length + 1;
    return true;
  } catch {
    return false; // corrupted/stale save — just start fresh rather than crash
  }
}

// --- undo / redo ---
let undoStack = [];
let redoStack = [];

function snapshotState() {
  return JSON.parse(JSON.stringify({ points, members, supports, loads, nextPointNum, nextMemberNum, nextLoadNum }));
}

// Call BEFORE mutating points/members/supports/loads, so the stack holds the
// state to go back to, not the state that just happened.
function pushHistory() {
  undoStack.push(snapshotState());
  if (undoStack.length > 100) undoStack.shift();
  redoStack = [];
}

function applyState(s) {
  ({ points, members, supports, loads, nextPointNum, nextMemberNum, nextLoadNum } = JSON.parse(JSON.stringify(s)));
  pendingMemberFrom = null;
  pendingSectionPoint = null;
  sectionLine = null;
  sectionResult = null;
  document.getElementById('sectionResult').innerHTML = '';
  lastResult = null;
  resultEl.textContent = '';
  updatePreciseMemberPanel();
  renderLists();
  draw();
}

function undo() {
  if (undoStack.length === 0) return;
  redoStack.push(snapshotState());
  applyState(undoStack.pop());
}

function redo() {
  if (redoStack.length === 0) return;
  undoStack.push(snapshotState());
  applyState(redoStack.pop());
}

window.addEventListener('keydown', (e) => {
  const key = e.key.toLowerCase();
  const mod = e.ctrlKey || e.metaKey;
  if (!mod) return;
  if (key === 'z' && !e.shiftKey) {
    e.preventDefault();
    undo();
  } else if (key === 'y' || (key === 'z' && e.shiftKey)) {
    e.preventDefault();
    redo();
  }
});

const GRID = 0.5; // snap grid, meters
const PICK_RADIUS = 14; // px

function snap(v) {
  return Math.round(v / GRID) * GRID;
}

function findPointNear(screenX, screenY) {
  let best = null;
  let bestDist = PICK_RADIUS;
  for (const p of points) {
    const s = worldToScreen(p.x, p.y);
    const d = Math.hypot(s.x - screenX, s.y - screenY);
    if (d < bestDist) {
      bestDist = d;
      best = p;
    }
  }
  return best;
}

function findMemberNear(screenX, screenY) {
  let best = null;
  let bestDist = 8;
  for (const m of members) {
    const a = points.find((p) => p.id === m.from);
    const b = points.find((p) => p.id === m.to);
    const sa = worldToScreen(a.x, a.y);
    const sb = worldToScreen(b.x, b.y);
    const d = distToSegment(screenX, screenY, sa, sb);
    if (d < bestDist) {
      bestDist = d;
      best = m;
    }
  }
  return best;
}

function distToSegment(px, py, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy || 1;
  let t = ((px - a.x) * dx + (py - a.y) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  const cx = a.x + t * dx;
  const cy = a.y + t * dy;
  return Math.hypot(px - cx, py - cy);
}

// --- tools ---
let tool = 'point';
let pendingMemberFrom = null;

const toolHints = {
  point: 'Click empty space to place a point (snaps to grid).',
  member: 'Click and drag from one point to another to connect them — or click, then click again.',
  cable: 'Click and drag from one point to another to connect them with a cable (tension-only — goes slack instead of carrying compression) — or click, then click again.',
  pin: 'Click a point to add a pin support (fixes both X and Y).',
  roller: 'Click a point to add a roller support (fixes one direction, set below).',
  fixed: 'Click a point to add a fixed/welded support (fixes X, Y, AND rotation — shown as an X). Note: a pin-jointed truss can\'t transmit moment through its joints, so this will make the truss indeterminate unless it\'s the only thing holding that side up.',
  load: 'Click a point to apply the Fx/Fy load set below.',
  section: 'Click two points anywhere to draw a cutting line through the members it crosses. Solve the truss first.',
  delete: 'Click a point, member, or force arrow to remove it (or use the × buttons in the sidebar).',
};

document.querySelectorAll('button[data-tool]').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('button[data-tool]').forEach((b) => b.classList.remove('active'));
    btn.classList.add('active');
    tool = btn.dataset.tool;
    pendingMemberFrom = null;
    pendingSectionPoint = null;
    toolHintEl.textContent = toolHints[tool];
    updatePreciseMemberPanel();
    draw();
  });
});
toolHintEl.textContent = toolHints[tool];

// --- precise member placement (three ways to define it from an anchor
// point: aligned length + angle, or one axis's distance + angle with the
// other axis derived from the angle's slope) ---
const preciseMemberEl = document.getElementById('preciseMember');
const preciseDistEl = document.getElementById('preciseDist');
const preciseDistLabelEl = document.getElementById('preciseDistLabel');
const preciseAngleEl = document.getElementById('preciseAngle');
let preciseMode = 'aligned'; // 'aligned' | 'x' | 'y'

const preciseModeLabels = {
  aligned: 'Length (m)',
  x: 'X distance (m)',
  y: 'Y distance (m)',
};

document.querySelectorAll('button[data-precise-mode]').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('button[data-precise-mode]').forEach((b) => b.classList.remove('active'));
    btn.classList.add('active');
    preciseMode = btn.dataset.preciseMode;
    preciseDistLabelEl.textContent = preciseModeLabels[preciseMode];
  });
});

function updatePreciseMemberPanel() {
  const active = (tool === 'member' || tool === 'cable') && !!pendingMemberFrom;
  preciseMemberEl.classList.toggle('disabled', !active);
  preciseDistEl.disabled = !active;
  preciseAngleEl.disabled = !active;
  document.getElementById('preciseCreateBtn').disabled = !active;
}
updatePreciseMemberPanel();

// Given the anchor, the chosen distance value, and the angle, compute the
// new point's offset. Aligned = classic length-along-the-line. X/Y-distance
// hold that one axis's displacement fixed and derive the other from the
// line's slope (tan/cot of the angle) — the same angle, just a different
// way of pinning down how far along it to go.
function computePreciseOffset(mode, dist, angleRad) {
  if (mode === 'aligned') {
    return { dx: dist * Math.cos(angleRad), dy: dist * Math.sin(angleRad) };
  }
  if (mode === 'x') {
    const cos = Math.cos(angleRad);
    if (Math.abs(cos) < 1e-9) return null; // vertical line — no finite x-distance works
    return { dx: dist, dy: dist * Math.tan(angleRad) };
  }
  // mode === 'y'
  const sin = Math.sin(angleRad);
  if (Math.abs(sin) < 1e-9) return null; // horizontal line — no finite y-distance works
  return { dx: dist / Math.tan(angleRad), dy: dist };
}

document.getElementById('preciseCreateBtn').addEventListener('click', () => {
  if ((tool !== 'member' && tool !== 'cable') || !pendingMemberFrom) return;
  const dist = Number(preciseDistEl.value);
  const angleDeg = Number(preciseAngleEl.value);
  if (!Number.isFinite(dist) || dist === 0 || !Number.isFinite(angleDeg)) return;
  const angleRad = (angleDeg * Math.PI) / 180;
  const offset = computePreciseOffset(preciseMode, dist, angleRad);
  if (!offset) {
    alert(
      preciseMode === 'x'
        ? 'A vertical line (90°/270°) has no finite X-distance — use Aligned or Y-dist instead.'
        : 'A horizontal line (0°/180°) has no finite Y-distance — use Aligned or X-dist instead.'
    );
    return;
  }
  pushHistory();
  const from = pendingMemberFrom;
  const to = { id: `P${nextPointNum++}`, x: from.x + offset.dx, y: from.y + offset.dy };
  points.push(to);
  const newMember = { id: `M${nextMemberNum++}`, from: from.id, to: to.id };
  if (tool === 'cable') newMember.type = 'cable';
  members.push(newMember);
  pendingMemberFrom = null;
  lastResult = null;
  resultEl.textContent = '';
  updatePreciseMemberPanel();
  renderLists();
  draw();
});

canvas.addEventListener('click', (e) => {
  if (suppressNextClick) {
    suppressNextClick = false;
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const sx = e.clientX - rect.left;
  const sy = e.clientY - rect.top;

  if (tool === 'point') {
    const w = screenToWorld(sx, sy);
    pushHistory();
    points.push({ id: `P${nextPointNum++}`, x: snap(w.x), y: snap(w.y) });
  } else if (tool === 'member' || tool === 'cable') {
    const p = findPointNear(sx, sy);
    if (!p) return;
    if (!pendingMemberFrom) {
      pendingMemberFrom = p;
    } else if (pendingMemberFrom.id !== p.id) {
      pushHistory();
      const newMember = { id: `M${nextMemberNum++}`, from: pendingMemberFrom.id, to: p.id };
      if (tool === 'cable') newMember.type = 'cable';
      members.push(newMember);
      pendingMemberFrom = null;
    }
    updatePreciseMemberPanel();
  } else if (tool === 'pin') {
    const p = findPointNear(sx, sy);
    if (!p) return;
    pushHistory();
    supports = supports.filter((s) => s.point !== p.id); // one support per point, keep it simple
    supports.push({ point: p.id, type: 'pin' });
  } else if (tool === 'roller') {
    const p = findPointNear(sx, sy);
    if (!p) return;
    pushHistory();
    const [dx, dy] = document.getElementById('rollerDir').value.split(',').map(Number);
    supports = supports.filter((s) => s.point !== p.id);
    supports.push({ point: p.id, type: 'roller', direction: { x: dx, y: dy } });
  } else if (tool === 'fixed') {
    const p = findPointNear(sx, sy);
    if (!p) return;
    pushHistory();
    supports = supports.filter((s) => s.point !== p.id);
    supports.push({ point: p.id, type: 'fixed' });
  } else if (tool === 'load') {
    const p = findPointNear(sx, sy);
    if (!p) return;
    pushHistory();
    const fx = Number(document.getElementById('loadFx').value) || 0;
    const fy = Number(document.getElementById('loadFy').value) || 0;
    loads.push({ id: `F${nextLoadNum++}`, point: p.id, fx, fy });
  } else if (tool === 'section') {
    const w = screenToWorld(sx, sy);
    if (!pendingSectionPoint) {
      sectionLine = null;
      sectionResult = null;
      document.getElementById('sectionResult').innerHTML = '';
      pendingSectionPoint = w;
    } else {
      sectionLine = { p1: pendingSectionPoint, p2: w };
      pendingSectionPoint = null;
      sectionResult = solveSection({ points, members, supports, loads }, lastResult, sectionLine);
      renderSectionResult(sectionResult);
    }
  } else if (tool === 'delete') {
    const p = findPointNear(sx, sy);
    const l = !p ? findLoadNear(sx, sy) : null;
    const m = !p && !l ? findMemberNear(sx, sy) : null;
    if (p) {
      pushHistory();
      points = points.filter((pt) => pt.id !== p.id);
      members = members.filter((mm) => mm.from !== p.id && mm.to !== p.id);
      supports = supports.filter((s) => s.point !== p.id);
      loads = loads.filter((ld) => ld.point !== p.id);
    } else if (l) {
      pushHistory();
      loads = loads.filter((ld) => ld.id !== l.id);
    } else if (m) {
      pushHistory();
      members = members.filter((mm) => mm.id !== m.id);
    }
  }

  if (tool !== 'section') {
    lastResult = null;
    resultEl.textContent = '';
    sectionLine = null;
    sectionResult = null;
    document.getElementById('sectionResult').innerHTML = '';
  }
  renderLists();
  draw();
});

// Pick a load by clicking near its arrow (tail-to-tip), not just the point,
// so it can be deleted without deleting the point it's attached to.
function findLoadNear(screenX, screenY) {
  let best = null;
  let bestDist = 10;
  for (const l of loads) {
    const p = points.find((pt) => pt.id === l.point);
    if (!p) continue;
    const mag = Math.hypot(l.fx, l.fy) || 1;
    const sp = worldToScreen(p.x, p.y);
    const tip = { x: sp.x + (l.fx / mag) * 40, y: sp.y - (l.fy / mag) * 40 };
    const d = distToSegment(screenX, screenY, sp, tip);
    if (d < bestDist) {
      bestDist = d;
      best = l;
    }
  }
  return best;
}

document.getElementById('clearBtn').addEventListener('click', () => {
  if (points.length || members.length || supports.length || loads.length) pushHistory();
  points = [];
  members = [];
  supports = [];
  loads = [];
  nextPointNum = 1;
  nextMemberNum = 1;
  nextLoadNum = 1;
  lastResult = null;
  pendingMemberFrom = null;
  pendingSectionPoint = null;
  sectionLine = null;
  sectionResult = null;
  document.getElementById('sectionResult').innerHTML = '';
  resultEl.textContent = '';
  renderLists();
  draw();
});

document.getElementById('solveBtn').addEventListener('click', () => {
  const structure = { points, members, supports, loads };
  sectionLine = null;
  sectionResult = null;
  document.getElementById('sectionResult').innerHTML = '';
  try {
    lastResult = solveTrussWithCables(structure);
  } catch (err) {
    lastResult = null;
    resultEl.textContent = `Error: ${err.message}`;
    draw();
    return;
  }
  renderResult(lastResult);
  draw();
});

function renderResult(result) {
  if (result.status === 'indeterminate') {
    let html = `Statically indeterminate (degree ${result.degree}).<br>Member forces can't be solved this way — would need member stiffness (EA) and the force/stiffness method.`;
    if (result.reactionSteps?.solved) {
      html += '<br><br><b>Reactions are still fully known</b>, even though member forces aren\'t:<ul>';
      for (const r of result.reactionSteps.reactions) {
        const unit = r.id.split(':')[1]?.startsWith('M') ? 'N·m' : 'N';
        html += `<li>${r.id}: ${r.value.toFixed(2)} ${unit}</li>`;
      }
      html += '</ul>';
    }
    resultEl.innerHTML = html;
    return;
  }
  if (result.status === 'unstable') {
    resultEl.textContent = `Unstable: ${result.reason}`;
    return;
  }

  let html = `<div style="color:#666;font-size:11px;">solved via: ${result.method === 'substitution' ? 'reactions-first substitution' : 'matrix fallback'}</div>`;
  if (result.removedCableIds?.length) {
    html += `<div style="color:#666;font-size:11px;">slack cable(s): ${result.removedCableIds.join(', ')} (went slack instead of carrying compression)</div>`;
  }
  html += '<b>Member forces</b><ul>';
  for (const m of result.memberForces) {
    html += `<li class="memberRow ${m.state}">${m.id}: ${m.force.toFixed(2)} N (${m.state})</li>`;
  }
  html += '</ul><b>Reactions</b><ul>';
  for (const r of result.reactions) {
    const unit = r.id.split(':')[1]?.startsWith('M') ? 'N·m' : 'N';
    html += `<li>${r.id}: ${r.value.toFixed(2)} ${unit}</li>`;
  }
  html += '</ul>';

  if (result.reactionSteps?.solved) {
    html += '<b>How the reactions were found</b><ol id="steps">';
    for (const s of result.reactionSteps.steps) {
      const val = s.value.map((v) => v.toFixed(2)).join(', ');
      html += `<li>${s.equation} → solved <code>${s.solvesFor.join(', ')}</code> = ${val} <i>(${s.note})</i></li>`;
    }
    html += '</ol>';
  } else if (result.reactionSteps?.reason) {
    html += `<i>${result.reactionSteps.reason}</i>`;
  }

  if (result.jointSteps) {
    html += '<b>How the member forces were found</b><ol id="steps">';
    for (const s of result.jointSteps) {
      const val = s.value.map((v) => v.toFixed(2)).join(', ');
      html += `<li>Joint ${s.joint} → solved <code>${s.solvesFor.join(', ')}</code> = ${val} <i>(${s.note})</i></li>`;
    }
    html += '</ol>';
  }

  resultEl.innerHTML = html;
}

function renderSectionResult(result) {
  const el = document.getElementById('sectionResult');
  if (!result.solved) {
    el.innerHTML = `<div style="border-top:1px solid #ccc;padding-top:8px;margin-top:8px;"><b>Section</b><br><i>${result.reason}</i></div>`;
    return;
  }
  let html = '<div style="border-top:1px solid #ccc;padding-top:8px;margin-top:8px;">';
  html += `<b>Section</b> — cuts ${result.cutMemberIds.join(', ')}<ul>`;
  for (const m of result.memberForces) {
    html += `<li class="memberRow ${m.state}">${m.id}: ${m.force.toFixed(2)} N (${m.state})</li>`;
  }
  html += '</ul><b>How it was solved</b><ol id="steps">';
  for (const s of result.steps) {
    const val = s.value.map((v) => v.toFixed(2)).join(', ');
    html += `<li>${s.equation} → solved <code>${s.solvesFor.join(', ')}</code> = ${val} <i>(${s.note})</i></li>`;
  }
  html += '</ol></div>';
  el.innerHTML = html;
}

// --- drawing ---
function draw() {
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);

  // grid
  ctx.strokeStyle = '#e5e5e5';
  ctx.lineWidth = 1;
  const gridPx = GRID * view.scale;
  for (let x = view.originX % gridPx; x < w; x += gridPx) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, h);
    ctx.stroke();
  }
  for (let y = view.originY % gridPx; y < h; y += gridPx) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }
  // X and Y axes through the world origin — thicker/darker than the grid so
  // orientation is always obvious at a glance, labeled at their positive end.
  ctx.strokeStyle = '#555';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, view.originY);
  ctx.lineTo(w, view.originY);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(view.originX, 0);
  ctx.lineTo(view.originX, h);
  ctx.stroke();

  ctx.fillStyle = '#555';
  ctx.font = 'bold 13px system-ui';
  ctx.fillText('X', w - 18, view.originY - 6);
  ctx.fillText('Y', view.originX + 6, 16);

  const forceById = new Map((lastResult?.memberForces ?? []).map((m) => [m.id, m]));
  const cutMemberIds = new Set(sectionResult?.solved ? sectionResult.cutMemberIds : []);

  // members — lines first, in their own pass, so a later member's line can
  // never paint over an earlier member's force label (canvas draws are
  // strictly paint-order-dependent, and members can cross each other).
  for (const m of members) {
    const a = points.find((p) => p.id === m.from);
    const b = points.find((p) => p.id === m.to);
    if (!a || !b) continue;
    const sa = worldToScreen(a.x, a.y);
    const sb = worldToScreen(b.x, b.y);
    const f = forceById.get(m.id);
    const stateColors = { tension: '#1a7f37', compression: '#b3261e', 'zero-force': '#888', slack: '#999' };
    ctx.strokeStyle = f ? stateColors[f.state] ?? '#333' : m.type === 'cable' ? '#555' : '#333';
    ctx.lineWidth = m.type === 'cable' ? 2 : 3;
    ctx.setLineDash(m.type === 'cable' ? [7, 4] : []);
    ctx.beginPath();
    ctx.moveTo(sa.x, sa.y);
    ctx.lineTo(sb.x, sb.y);
    ctx.stroke();
    ctx.setLineDash([]);

    if (cutMemberIds.has(m.id)) {
      ctx.save();
      ctx.strokeStyle = '#e07b00';
      ctx.lineWidth = 2;
      ctx.setLineDash([2, 4]);
      ctx.beginPath();
      ctx.moveTo(sa.x, sa.y);
      ctx.lineTo(sb.x, sb.y);
      ctx.stroke();
      ctx.restore();
    }
  }

  // member force labels — a separate pass, painted after EVERY member line
  // so labels always sit above the members, never underneath a crossing one.
  for (const m of members) {
    const a = points.find((p) => p.id === m.from);
    const b = points.find((p) => p.id === m.to);
    if (!a || !b) continue;
    const sa = worldToScreen(a.x, a.y);
    const sb = worldToScreen(b.x, b.y);
    const f = forceById.get(m.id);
    if (f) {
      const stateColors = { tension: '#1a7f37', compression: '#b3261e', 'zero-force': '#888', slack: '#999' };
      const mx = (sa.x + sb.x) / 2;
      const my = (sa.y + sb.y) / 2;
      ctx.fillStyle = stateColors[f.state] ?? '#333';
      ctx.font = '11px system-ui';
      ctx.fillText(`${f.force.toFixed(1)}N`, mx + 4, my - 4);
    }
  }

  // cutting line (section tool), extended across the visible canvas
  if (sectionLine) {
    const { p1, p2 } = sectionLine;
    const s1 = worldToScreen(p1.x, p1.y);
    const s2 = worldToScreen(p2.x, p2.y);
    const dx = s2.x - s1.x;
    const dy = s2.y - s1.y;
    const len = Math.hypot(dx, dy) || 1;
    const ext = 2000;
    const ex = (dx / len) * ext;
    const ey = (dy / len) * ext;
    ctx.save();
    ctx.strokeStyle = '#e07b00';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    ctx.moveTo(s1.x - ex, s1.y - ey);
    ctx.lineTo(s2.x + ex, s2.y + ey);
    ctx.stroke();
    ctx.restore();
  }
  if (tool === 'section' && pendingSectionPoint) {
    const s = worldToScreen(pendingSectionPoint.x, pendingSectionPoint.y);
    ctx.fillStyle = '#e07b00';
    ctx.beginPath();
    ctx.arc(s.x, s.y, 5, 0, Math.PI * 2);
    ctx.fill();
  }

  // pending member preview (click-click flow)
  if ((tool === 'member' || tool === 'cable') && pendingMemberFrom) {
    const s = worldToScreen(pendingMemberFrom.x, pendingMemberFrom.y);
    ctx.fillStyle = '#0969da';
    ctx.beginPath();
    ctx.arc(s.x, s.y, 8, 0, Math.PI * 2);
    ctx.stroke();
  }

  // live rubber-band line while dragging from a point (click-and-drag flow)
  if (dragFrom && dragMoved && dragCurrentScreen) {
    const s = worldToScreen(dragFrom.x, dragFrom.y);
    ctx.save();
    ctx.strokeStyle = '#0969da';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(s.x, s.y);
    ctx.lineTo(dragCurrentScreen.x, dragCurrentScreen.y);
    ctx.stroke();
    ctx.restore();
  }

  // supports
  for (const s of supports) {
    const p = points.find((pt) => pt.id === s.point);
    if (!p) continue;
    const sp = worldToScreen(p.x, p.y);
    ctx.fillStyle = '#444';
    ctx.strokeStyle = '#444';
    if (s.type === 'pin') {
      ctx.beginPath();
      ctx.moveTo(sp.x, sp.y);
      ctx.lineTo(sp.x - 10, sp.y + 16);
      ctx.lineTo(sp.x + 10, sp.y + 16);
      ctx.closePath();
      ctx.fill();
    } else if (s.type === 'fixed') {
      // Welded: an X below the joint.
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(sp.x - 8, sp.y + 6);
      ctx.lineTo(sp.x + 8, sp.y + 22);
      ctx.moveTo(sp.x + 8, sp.y + 6);
      ctx.lineTo(sp.x - 8, sp.y + 22);
      ctx.stroke();
    } else {
      // Roller: a plain circle sitting under the joint (direction shown only
      // via the reaction arrow after solving, to keep the glyph simple).
      ctx.beginPath();
      ctx.arc(sp.x, sp.y + 12, 8, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // reactions (after solve)
  if (lastResult?.status === 'determinate') {
    for (const r of lastResult.reactions) {
      const p = points.find((pt) => pt.id === r.point);
      if (!p) continue;
      const kind = r.id.split(':')[1]; // "Rx", "Ry", "R", or "M" (optionally "#2" suffixed)
      if (kind.startsWith('M')) {
        drawMomentGlyph(p, r.value, '#0969da');
        continue;
      }
      let dirX, dirY;
      if (kind.startsWith('Rx')) [dirX, dirY] = [1, 0];
      else if (kind.startsWith('Ry')) [dirX, dirY] = [0, 1];
      else {
        const support = supports.find((s) => s.point === r.point && s.type === 'roller');
        dirX = support?.direction?.x ?? 0;
        dirY = support?.direction?.y ?? 1;
      }
      const sign = Math.sign(r.value) || 1;
      drawArrow(p, dirX * sign, dirY * sign, '#0969da', `${Math.abs(r.value).toFixed(1)}N`);
    }
  }

  // loads
  for (const l of loads) {
    const p = points.find((pt) => pt.id === l.point);
    if (!p) continue;
    const mag = Math.hypot(l.fx, l.fy) || 1;
    drawArrow(p, l.fx / mag, l.fy / mag, '#9a3fd4', `${mag.toFixed(1)}N`);
  }

  // points (drawn last, on top)
  for (const p of points) {
    const sp = worldToScreen(p.x, p.y);
    ctx.fillStyle = '#000';
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.font = '11px system-ui';
    ctx.fillText(p.id, sp.x + 6, sp.y - 6);
  }

  statusEl.textContent = `Points: ${points.length}  Members: ${members.length}  Supports: ${supports.length}  Loads: ${loads.length}`;
  scheduleSave(); // debounced — draw() runs on every mutation (and on pan/zoom, harmlessly)
}

// A moment reaction has no direction to draw as an arrow — it's a couple,
// same value regardless of viewpoint — so render it as a curved arrow
// looping the joint instead, with rotation sense matching the sign
// convention used everywhere else (positive = CCW).
function drawMomentGlyph(worldPoint, value, color) {
  const sp = worldToScreen(worldPoint.x, worldPoint.y);
  const radius = 20;
  const ccw = value < 0; // canvas angles increase clockwise on screen, so
  // a positive (CCW, math-convention) moment sweeps with ccw=false here.
  const startAngle = -0.3;
  const endAngle = Math.PI * 1.5;
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(sp.x, sp.y, radius, startAngle, endAngle, ccw);
  ctx.stroke();

  const tipAngle = ccw ? startAngle : endAngle;
  const tip = { x: sp.x + radius * Math.cos(tipAngle), y: sp.y + radius * Math.sin(tipAngle) };
  const tangent = tipAngle + (ccw ? -Math.PI / 2 : Math.PI / 2);
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - 7 * Math.cos(tangent - 0.5), tip.y - 7 * Math.sin(tangent - 0.5));
  ctx.lineTo(tip.x - 7 * Math.cos(tangent + 0.5), tip.y - 7 * Math.sin(tangent + 0.5));
  ctx.closePath();
  ctx.fill();

  ctx.font = '11px system-ui';
  ctx.fillText(`${Math.abs(value).toFixed(1)}N·m`, sp.x + radius + 4, sp.y - radius);
}

function drawArrow(worldPoint, ux, uy, color, label) {
  const len = 40;
  const sp = worldToScreen(worldPoint.x, worldPoint.y);
  const tip = { x: sp.x + ux * len, y: sp.y - uy * len };
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(sp.x, sp.y);
  ctx.lineTo(tip.x, tip.y);
  ctx.stroke();
  const angle = Math.atan2(tip.y - sp.y, tip.x - sp.x);
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - 8 * Math.cos(angle - 0.4), tip.y - 8 * Math.sin(angle - 0.4));
  ctx.lineTo(tip.x - 8 * Math.cos(angle + 0.4), tip.y - 8 * Math.sin(angle + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.font = '11px system-ui';
  ctx.fillText(label, tip.x + 4, tip.y);
}

// --- editable Points / Forces lists in the sidebar ---
function renamePoint(oldId, newId) {
  const p = points.find((pt) => pt.id === oldId);
  p.id = newId;
  for (const m of members) {
    if (m.from === oldId) m.from = newId;
    if (m.to === oldId) m.to = newId;
  }
  for (const s of supports) if (s.point === oldId) s.point = newId;
  for (const l of loads) if (l.point === oldId) l.point = newId;
  if (pendingMemberFrom?.id === oldId) pendingMemberFrom = p;
}

// Numeric sidebar fields fire 'input' on every keystroke (needed for live
// redraw) but should only cost ONE undo step per edit session, not one per
// keystroke — capture the pre-edit state on focus, commit it to the undo
// stack on 'change' (blur/Enter).
let preEditSnapshot = null;
function armUndoOnCommit(input) {
  input.addEventListener('focus', () => {
    preEditSnapshot = snapshotState();
  });
  input.addEventListener('change', () => {
    if (preEditSnapshot) {
      undoStack.push(preEditSnapshot);
      if (undoStack.length > 100) undoStack.shift();
      redoStack = [];
      preEditSnapshot = null;
    }
  });
}

function renderLists() {
  const pointsListEl = document.getElementById('pointsList');
  pointsListEl.innerHTML = '';
  for (const p of points) {
    const row = document.createElement('div');
    row.className = 'objRow';

    const idInput = document.createElement('input');
    idInput.type = 'text';
    idInput.value = p.id;
    idInput.title = 'Rename point';
    idInput.addEventListener('change', () => {
      const newId = idInput.value.trim();
      if (!newId || newId === p.id) {
        idInput.value = p.id;
        return;
      }
      if (points.some((pt) => pt.id === newId)) {
        alert(`Point "${newId}" already exists`);
        idInput.value = p.id;
        return;
      }
      pushHistory();
      renamePoint(p.id, newId);
      renderLists();
      draw();
    });

    const xInput = document.createElement('input');
    xInput.type = 'number';
    xInput.step = '0.1';
    xInput.value = p.x;
    xInput.title = 'X';
    armUndoOnCommit(xInput);
    xInput.addEventListener('input', () => {
      p.x = Number(xInput.value) || 0;
      lastResult = null;
      resultEl.textContent = '';
      draw();
    });

    const yInput = document.createElement('input');
    yInput.type = 'number';
    yInput.step = '0.1';
    yInput.value = p.y;
    yInput.title = 'Y';
    armUndoOnCommit(yInput);
    yInput.addEventListener('input', () => {
      p.y = Number(yInput.value) || 0;
      lastResult = null;
      resultEl.textContent = '';
      draw();
    });

    const delBtn = document.createElement('button');
    delBtn.textContent = '×';
    delBtn.title = 'Delete point';
    delBtn.addEventListener('click', () => {
      pushHistory();
      points = points.filter((pt) => pt.id !== p.id);
      members = members.filter((m) => m.from !== p.id && m.to !== p.id);
      supports = supports.filter((s) => s.point !== p.id);
      loads = loads.filter((l) => l.point !== p.id);
      lastResult = null;
      resultEl.textContent = '';
      renderLists();
      draw();
    });

    row.append(idInput, xInput, yInput, delBtn);
    pointsListEl.appendChild(row);
  }

  const membersListEl = document.getElementById('membersList');
  membersListEl.innerHTML = '';
  for (const m of members) {
    const from = points.find((p) => p.id === m.from);
    const to = points.find((p) => p.id === m.to);
    if (!from || !to) continue;

    const row = document.createElement('div');
    row.className = 'objRow';

    const label = document.createElement('span');
    label.textContent = `${m.id}${m.type === 'cable' ? ' (cable)' : ''} (${m.from}→${m.to})`;
    label.style.fontSize = '11px';
    label.style.minWidth = '70px';

    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const lengthInput = document.createElement('input');
    lengthInput.type = 'number';
    lengthInput.step = '0.1';
    lengthInput.min = '0.01';
    lengthInput.value = Math.hypot(dx, dy).toFixed(3);
    lengthInput.title = 'Length (m) — moves the "to" point';
    armUndoOnCommit(lengthInput);
    lengthInput.addEventListener('input', () => {
      const newLength = Number(lengthInput.value);
      if (!(newLength > 0)) return;
      const curDx = to.x - from.x;
      const curDy = to.y - from.y;
      const angle = Math.atan2(curDy, curDx);
      to.x = from.x + newLength * Math.cos(angle);
      to.y = from.y + newLength * Math.sin(angle);
      lastResult = null;
      resultEl.textContent = '';
      draw(); // not renderLists() — would rebuild the input mid-keystroke and steal focus
    });

    const angleInput = document.createElement('input');
    angleInput.type = 'number';
    angleInput.step = '1';
    angleInput.value = ((Math.atan2(dy, dx) * 180) / Math.PI).toFixed(1);
    angleInput.title = 'Angle (° from +X, CCW) — moves the "to" point';
    armUndoOnCommit(angleInput);
    angleInput.addEventListener('input', () => {
      const newAngleDeg = Number(angleInput.value);
      if (!Number.isFinite(newAngleDeg)) return;
      const curLength = Math.hypot(to.x - from.x, to.y - from.y);
      const angle = (newAngleDeg * Math.PI) / 180;
      to.x = from.x + curLength * Math.cos(angle);
      to.y = from.y + curLength * Math.sin(angle);
      lastResult = null;
      resultEl.textContent = '';
      draw(); // not renderLists() — would rebuild the input mid-keystroke and steal focus
    });

    const delBtn = document.createElement('button');
    delBtn.textContent = '×';
    delBtn.title = 'Delete member';
    delBtn.addEventListener('click', () => {
      pushHistory();
      members = members.filter((mm) => mm.id !== m.id);
      lastResult = null;
      resultEl.textContent = '';
      renderLists();
      draw();
    });

    row.append(label, lengthInput, angleInput, delBtn);
    membersListEl.appendChild(row);
  }

  const loadsListEl = document.getElementById('loadsList');
  loadsListEl.innerHTML = '';
  for (const l of loads) {
    const row = document.createElement('div');
    row.className = 'objRow';

    const idInput = document.createElement('input');
    idInput.type = 'text';
    idInput.value = l.id;
    idInput.title = 'Rename force';
    idInput.addEventListener('change', () => {
      pushHistory();
      l.id = idInput.value.trim() || l.id;
      idInput.value = l.id;
      draw();
    });

    const pointSelect = document.createElement('select');
    for (const p of points) {
      const opt = document.createElement('option');
      opt.value = p.id;
      opt.textContent = p.id;
      if (p.id === l.point) opt.selected = true;
      pointSelect.appendChild(opt);
    }
    pointSelect.title = 'Applied at point';
    pointSelect.addEventListener('change', () => {
      pushHistory();
      l.point = pointSelect.value;
      lastResult = null;
      resultEl.textContent = '';
      draw();
    });

    const fxInput = document.createElement('input');
    fxInput.type = 'number';
    fxInput.step = '0.5';
    fxInput.value = l.fx;
    fxInput.title = 'Fx';
    armUndoOnCommit(fxInput);
    fxInput.addEventListener('input', () => {
      l.fx = Number(fxInput.value) || 0;
      lastResult = null;
      resultEl.textContent = '';
      draw();
    });

    const fyInput = document.createElement('input');
    fyInput.type = 'number';
    fyInput.step = '0.5';
    fyInput.value = l.fy;
    fyInput.title = 'Fy';
    armUndoOnCommit(fyInput);
    fyInput.addEventListener('input', () => {
      l.fy = Number(fyInput.value) || 0;
      lastResult = null;
      resultEl.textContent = '';
      draw();
    });

    const delBtn = document.createElement('button');
    delBtn.textContent = '×';
    delBtn.title = 'Delete force';
    delBtn.addEventListener('click', () => {
      pushHistory();
      loads = loads.filter((ld) => ld.id !== l.id);
      lastResult = null;
      resultEl.textContent = '';
      renderLists();
      draw();
    });

    row.append(idInput, pointSelect, fxInput, fyInput, delBtn);
    loadsListEl.appendChild(row);
  }
}

// --- nonprofessional-use disclaimer gate ---
// Shown once per browser (remembered via localStorage), not once per
// session, so returning visitors aren't re-blocked every visit — but it
// defaults to VISIBLE in the HTML itself and only gets hidden here, so if
// this script fails to run for any reason the disclaimer fails closed
// rather than silently not appearing.
const DISCLAIMER_KEY = 'statics_solver:disclaimer_accepted:v1';
const disclaimerOverlay = document.getElementById('disclaimerOverlay');

try {
  if (localStorage.getItem(DISCLAIMER_KEY) === '1') {
    disclaimerOverlay.style.display = 'none';
  }
} catch {
  // Storage unavailable — just leave the disclaimer up; better to ask again
  // than to risk silently skipping it.
}

document.getElementById('disclaimerAccept').addEventListener('click', () => {
  try {
    localStorage.setItem(DISCLAIMER_KEY, '1');
  } catch {
    // Can't remember it — the user will just see it again next visit.
  }
  disclaimerOverlay.style.display = 'none';
});

document.getElementById('disclaimerDeny').addEventListener('click', () => {
  window.location.href = '/';
});

loadFromStorage();
resize();
renderLists();
