// worker.js - runs the shaft rail's Python (shaft_rail/*.py, copied into py/ by publish_web.py) in
// Pyodide, off the main thread. Messages:  {id, cmd: "build" | "info", spec}  ->  {id, info, meshes}
// where meshes[name] = {pos: Float32Array, idx: Uint32Array, tris}. Bump PY_VERSION whenever the
// py/ files change: GitHub Pages lets browsers cache them for 10 minutes.
const PY_VERSION = 6;
const PY_FILES = ["shaft_rail.py", "mesh2d.py", "geneva.py", "rail_mechs.py", "web_api.py"];

importScripts("https://cdn.jsdelivr.net/pyodide/v0.26.4/full/pyodide.js");

let pageCall = null;

const ready = (async () => {
  const py = await loadPyodide();
  py.FS.mkdirTree("/app");
  for (const f of PY_FILES) {
    const r = await fetch(`py/${f}?v=${PY_VERSION}`);
    if (!r.ok) throw new Error(`failed to fetch ${f}: ${r.status}`);
    py.FS.writeFile(`/app/${f}`, await r.text());
  }
  py.runPython(`
import sys
sys.path.insert(0, "/app")
import web_api
`);
  pageCall = py.globals.get("web_api").page_call;
})();

self.onmessage = async (e) => {
  const { id, cmd, spec } = e.data;
  try {
    await ready;
    const t0 = performance.now();
    const res = pageCall(JSON.stringify(spec), cmd === "build");
    const [infoJson, meshProxy] = res.toJs({ dict_converter: Object.fromEntries });
    res.destroy();
    const meshes = {};
    const transfer = [];
    for (const [name, [pos, idx, tris]] of Object.entries(meshProxy)) {
      const p = new Float32Array(pos.buffer, pos.byteOffset, pos.byteLength / 4).slice();
      const i = new Uint32Array(idx.buffer, idx.byteOffset, idx.byteLength / 4).slice();
      meshes[name] = { pos: p, idx: i, tris };
      transfer.push(p.buffer, i.buffer);
    }
    self.postMessage({ id, info: JSON.parse(infoJson), meshes, ms: performance.now() - t0 }, transfer);
  } catch (err) {
    self.postMessage({ id, error: String(err && err.message ? err.message : err) });
  }
};
