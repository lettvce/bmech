"""
web_api.py - the page's entry point (runs in Pyodide; plain CPython for tests).

build(spec_json) -> dict with the layout summary and every part as triangles:
    meshes[name] = (positions: bytes of float32 xyz, indices: bytes of uint32)
Rail parts (body, screw, handle, endstop, nut) are shared by every carrier; mechanisms are one per rail
(per nut: a two-sided rail has two per row, see rail_mechs.row_side).
Parts are cached by their inputs, so changing one rail's mechanism rebuilds only that mesh.
"""

import array
import json
import math

import rail_mechs as rm
import shaft_rail as sr
from mesh2d import triangulate

_CACHE = {}
_CACHE_MAX = 64


def _cached(key, fn):
    if key in _CACHE:
        return _CACHE[key]
    if len(_CACHE) >= _CACHE_MAX:
        _CACHE.pop(next(iter(_CACHE)))
    _CACHE[key] = v = fn()
    return v


def _newell(verts, f):
    nx = ny = nz = 0.0
    for i in range(len(f)):
        a, b = verts[f[i]], verts[f[(i + 1) % len(f)]]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / L, ny / L, nz / L


def triangles(verts, faces):
    """Faces (tris, quads, planar n-gons, possibly concave or with collinear corners) -> triangles with
    the same winding. A collinear corner the ear clipper skips is put back as a zero-area sliver, so
    the triangle mesh stays watertight (no T-junctions)."""
    out = []
    for f in faces:
        n = len(f)
        if n == 3:
            out.append(tuple(f))
            continue
        if n == 4:
            out += [(f[0], f[1], f[2]), (f[0], f[2], f[3])]
            continue
        nrm = _newell(verts, f)
        a = (1.0, 0.0, 0.0) if abs(nrm[0]) < 0.9 else (0.0, 1.0, 0.0)
        u = (a[1] * nrm[2] - a[2] * nrm[1], a[2] * nrm[0] - a[0] * nrm[2], a[0] * nrm[1] - a[1] * nrm[0])
        w = (nrm[1] * u[2] - nrm[2] * u[1], nrm[2] * u[0] - nrm[0] * u[2], nrm[0] * u[1] - nrm[1] * u[0])
        pts = [(sum(verts[i][k] * u[k] for k in range(3)), sum(verts[i][k] * w[k] for k in range(3))) for i in f]
        tris = triangulate(pts, list(range(n)), [])
        used = {i for t in tris for i in t}
        # collinear corners the clipper dropped: a sliver (prev, i, next) restores the edge split
        edges = {(t[k], t[(k + 1) % 3]) for t in tris for k in range(3)}
        for i in range(n):
            if i in used:
                continue
            # find the triangle edge that spans this corner along the boundary
            p, q = (i - 1) % n, (i + 1) % n
            while p not in used:
                p = (p - 1) % n
            while q not in used:
                q = (q + 1) % n
            if (p, q) in edges:
                run = [p]
                k = (p + 1) % n
                while k != q:
                    run.append(k)
                    k = (k + 1) % n
                run.append(q)
                # replace triangle (p, q, r) by a fan from r over the run
                for t in list(tris):
                    for k3 in range(3):
                        if (t[k3], t[(k3 + 1) % 3]) == (p, q):
                            r = t[(k3 + 2) % 3]
                            tris.remove(t)
                            tris += [(run[s], run[s + 1], r) for s in range(len(run) - 1)]
                            break
                used.update(run)
                edges = {(t[k], t[(k + 1) % 3]) for t in tris for k in range(3)}
        out += [(f[a_], f[b_], f[c_]) for a_, b_, c_ in tris]
    return out


def _pack(verts, faces):
    tris = triangles(verts, faces)
    pos = array.array("f", [c for v in verts for c in v])
    idx = array.array("I", [i for t in tris for i in t])
    return pos.tobytes(), idx.tobytes(), len(tris)


def _rail_key(q):
    return json.dumps({k: q[k] for k in list(sr.DEFAULTS) + list(sr.EXTRA_DEFAULTS)}, sort_keys=True)


def build(spec_json, with_meshes=True):
    """with_meshes=False skips every mesh (the page uses it to re-check a layout after a nut moves)."""
    spec = json.loads(spec_json) if isinstance(spec_json, str) else spec_json
    lay = rm.layout(spec)
    q = lay["q"]
    rk = _rail_key(q)
    meshes = {}
    for name, fn in () if not with_meshes else (("body", sr.build_body), ("screw", sr.build_screw), ("handle", sr.build_handle),
                     ("endstop", sr.build_endstop), ("nut", sr.build_nut)):
        meshes[name] = _cached((name, rk), lambda fn=fn: _pack(*fn(q)))
    gk = json.dumps(lay["gear"], sort_keys=True)
    rails = []
    for it in lay["rails"]:
        i = it["index"]
        name = "mech_%d" % i
        if it["kind"] != "none" and with_meshes:
            mk = json.dumps({k: v for k, v in it["mq"].items() if isinstance(v, (int, float, bool, str))},
                            sort_keys=True)
            meshes[name] = _cached((it["kind"], rk, gk, mk), lambda i=i: _pack(*rm.build_mech(lay, i)))
        mq = it.get("mq") or {}
        info = dict(index=i, kind=it["kind"], x=it["x"], y=it["y"], row=it["row"], side=it["side"],
                    label=it["label"], parent=it.get("parent"),
                    offset=it.get("offset", 0.0), distance=it.get("distance"), prog=it["prog"],
                    prims=lay["prims"][i], mesh=name if it["kind"] != "none" else None)
        if it["kind"] == "spur":
            info.update(teeth=mq["teeth"], thickness=mq["thickness"], tip=mq["ra"])
        elif it["kind"] == "cluster":
            info.update(teeth=mq["teeth"], thickness=mq["thickness"], tip=mq["ra"], teeth_top=mq["teeth_top"],
                        thickness_top=mq["thickness_top"], tip_top=mq["ra_top"])
        elif it["kind"] in ("geneva_driver", "geneva_wheel"):
            info.update(slots=mq["slots"], center_distance=mq["center_distance"], dwell=mq["dwell"],
                        toothed=bool(mq["driver_gear"]), gear_teeth=mq.get("gear_teeth"),
                        a=mq["a"], alpha=mq["alpha"], beta=mq["beta"], plate_r=mq["plate_r"],
                        tip=mq["tip_r"] if it["kind"] == "geneva_wheel" else mq["plate_r"],
                        z_wheel=mq["z_wheel"], z_top=mq["z_top"], plate_thickness=mq["plate_thickness"],
                        wheel_thickness=mq["wheel_thickness"])
        info.update(lift=it.get("lift", 0.0), mesh_layers=it.get("mesh_layers"))
        rails.append(info)
    summary = {k: q[k] for k in ("count", "spacing", "x_min", "x_max", "x_ref", "lead", "lead_angle", "travel",
                                 "turns", "body_length", "body_width", "body_height", "thread_pitch",
                                 "thread_starts", "knob_diameter", "z_top", "z_tip", "shaft_length",
                                 "r_shaft", "segments", "x_out0", "x_out1", "two_sided", "center_gap", "x_mid")}
    summary["rail_y"] = [sr.carrier_y(q, i) for i in range(q["count"])]
    return dict(rail=summary, gear=lay["gear"], hub=lay["hub"], rails=rails, notes=lay["notes"],
                issues=lay["issues"], input=lay["input"], knobs=lay["knobs"], meshes=meshes,
                defaults=dict(rail=dict(sr.DEFAULTS, **sr.EXTRA_DEFAULTS), gear=rm.GEAR_DEFAULTS, mech=rm.MECH_DEFAULTS))


def page_call(spec_json, with_meshes):
    """(info JSON, {mesh name: (positions bytes, indices bytes, triangle count)}) for the worker."""
    r = build(spec_json, with_meshes)
    meshes = r.pop("meshes")
    return json.dumps(r), meshes
