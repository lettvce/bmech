"""
rail_mechs.py - mechanisms that drop over the shaft rail's nut shafts (pure Python, no Blender).

Every mechanism is one closed solid fused to a capped hub: the hub's bore runs over the shaft with
`shaft_clearance`, its roof rests on the shaft tip, and its bottom floats HUB_LIFT above the collar.
Gravity holds it on (the shafts stand vertical); there is no separate stop cap to lose.

    spur           involute spur gear (bmech profile, geneva.gear_outline)
    geneva_driver  Geneva crank: plate (plain disc or spur teeth), locking disc and pin
    geneva_wheel   the slotted wheel, raised on its hub to sit over its driver's plate
    mount          a FIXED base for parts that must not spin (a Scotch yoke's guide, a bracket): it rests
                   on the shaft tip like a hub, and two grip walls hug the nut's front and back faces
                   (the nut stands shaft_rail.NUT_RISE above the body), so it can't turn. Flat deck on
                   top at hub-top height to build on; open underneath, so it prints deck-down unsupported.

Frames: a mechanism is built in its nut's local frame (shaft axis on x = y = 0, z as in
shaft_rail.build_nut), so it is placed at the same (x, y) as its nut.

Layout: rails can link to another rail so their mechanisms mesh. A linked rail's nut sits at
    x = x_parent + side * sqrt(d^2 - dy^2),    dy = rail gap x carrier spacing
where d is the centre distance (spur pair: m (z1 + z2) / 2 + backlash; Geneva: C). layout() returns the
rest positions, an angle program per rail (for animation), collision primitives and notes.

Public API: DEFAULTS, MECH_DEFAULTS, hub(q), build_mech(q, gq, spec, ...), layout(spec), collisions(...)
"""

import math

import geneva
import shaft_rail as sr
from mesh2d import _Mesh, triangulate

HUB_WALL = 1.6          # hub tube wall around the bore
HUB_ROOF = 1.6          # cap over the shaft tip
HUB_LIFT = 0.4          # hub bottom above the collar top; the roof rests on the shaft tip
STACK_ROOM = 0.5        # top of a Geneva stack below the shaft tip (hub roof sits above)
COLLIDE_GAP = 0.3       # parts that are not meant to touch keep at least this apart
MOUNT_WALL = 2.0        # grip wall thickness, along the rail
MOUNT_FLOAT = 0.4       # grip walls above the body top; pocket ceiling above the collar top

GEAR_DEFAULTS = dict(module=2.4, pressure_angle=20.0, backlash=0.25)    # chunky teeth, still many ratios
MECH_DEFAULTS = {
    "none": {},
    "spur": dict(teeth=16, thickness=5.0),
    "geneva_driver": dict(slots=6, center_distance=40.0, pin_diameter=5.0, clearance=0.2,
                          plate_thickness=3.0, layer_gap=0.4, wheel_thickness=5.0,
                          toothed=False, gear_teeth=24, tip_round=1.0),
    "geneva_wheel": {},     # takes every dimension from the driver it links to
    "mount": {},            # sized by the nut and shaft
}


# ── Hub ──────────────────────────────────────────────────────────────────────

def hub(q):
    """Hub dimensions in the nut frame."""
    rb = q["r_hub_bore"]
    zb = q["z_shaft0"] + HUB_LIFT
    return dict(rb=rb, rh=rb + HUB_WALL, zb=zb, z_tip=q["z_tip"], z_top=q["z_tip"] + HUB_ROOF,
                room=q["z_tip"] - zb)


def _circle(r, n, a0=0.0):
    return [(r * math.cos(a0 + 2 * math.pi * j / n), r * math.sin(a0 + 2 * math.pi * j / n)) for j in range(n)]


class _Build:
    """Thin layer over mesh2d._Mesh (welded vertices) for z-stacked loops."""

    def __init__(self):
        self.m = _Mesh()

    def loop(self, pts, z):
        return [self.m.v(x, y, z) for x, y in pts]

    def wall(self, lo, hi):
        """Vertical quads between matching loops, outward for a CCW loop (lo below hi)."""
        n = len(lo)
        for j in range(n):
            self.m.faces.append((lo[j], lo[(j + 1) % n], hi[(j + 1) % n], hi[j]))

    def face(self, outer, holes, up):
        self.m.face(outer, holes, (0.0, 0.0, 1.0 if up else -1.0))

    def ngon(self, loop, up):
        self.m.faces.append(tuple(loop) if up else tuple(loop[::-1]))

    def ring(self, outline, period, n_sym, ri, kb, z, up):
        """Flat face between an n_sym-fold symmetric CCW outline (period points per repeat, repeat i
        starting at angle -pi/n_sym + 2 pi i / n_sym) and an inner circle of radius ri with kb points
        per repeat. One wedge is triangulated and rotated. Returns (outline loop, inner loop)."""
        NU, NK = len(outline), n_sym * kb
        inner = _circle(ri, NK, -math.pi / n_sym)
        ol, il = self.loop(outline, z), self.loop(inner, z)
        wedge = [("o", k) for k in range(period + 1)] + [("i", k) for k in range(kb, -1, -1)]

        def pt(t, k):
            return outline[k % NU] if t == "o" else inner[k % NK]
        tris = triangulate([pt(*w) for w in wedge], list(range(len(wedge))), [])
        for r in range(n_sym):
            for tri in tris:
                f = tuple((ol[(wedge[i][1] + r * period) % NU] if wedge[i][0] == "o"
                           else il[(wedge[i][1] + r * kb) % NK]) for i in tri)
                self.m.faces.append(f if up else f[::-1])
        return ol, il

    def bore(self, h, n, z0):
        """Bore wall (facing the axis) from z0 to the shaft tip plus its roof. Returns the bottom loop."""
        c = _circle(h["rb"], n)[::-1]                      # CW: wall faces inward
        lo, hi = self.loop(c, z0), self.loop(c, h["z_tip"])
        self.wall(lo, hi)
        self.m.faces.append(tuple(hi))                     # roof: a CW loop seen from +Z faces down
        return lo

    def hub_top(self, h, n, z0, start=None):
        """Outer hub wall from z0 up to the top, plus the top cap. Returns the bottom loop."""
        c = _circle(h["rh"], n, start or 0.0)
        lo, hi = self.loop(c, z0), self.loop(c, h["z_top"])
        self.wall(lo, hi)
        self.ngon(hi, True)
        return lo

    def done(self):
        return self.m.verts, self.m.faces


def _kb(n_sym, segs):
    return max(1, int(math.ceil(segs / n_sym)))


# ── Mechanisms ───────────────────────────────────────────────────────────────

def spur_q(q, g, spec):
    """Clamped spur gear: teeth so the root leaves a 1 mm wall around the hub; thickness fits the hub."""
    h = hub(q)
    m = g["module"]
    notes = []
    z = int(spec.get("teeth", 16))
    zmin = max(geneva.MIN_TEETH, int(math.ceil(2 * (h["rh"] + 1.0) / m + 2 * geneva.DEDENDUM - 1e-9)))
    if z < zmin or z > 400:
        notes.append(("teeth", z, min(max(z, zmin), 400)))
        z = min(max(z, zmin), 400)
    t = float(spec.get("thickness", 5.0))
    tmax = h["room"] - 0.5
    if t < 0.6 or t > tmax:
        notes.append(("thickness", t, min(max(t, 0.6), tmax)))
        t = min(max(t, 0.6), tmax)
    rp = m * z / 2
    return dict(kind="spur", teeth=z, thickness=t, rp=rp, ra=rp + geneva.ADDENDUM * m,
                rd=rp - geneva.DEDENDUM * m), notes


def build_spur(q, g, sq):
    h, segs = hub(q), q["segments"]
    m, N = g["module"], sq["teeth"]
    kb = _kb(N, segs)
    out = geneva.gear_outline(m, N, g["pressure_angle"], segs)
    U = len(out) // N
    B = _Build()
    zb, z1 = h["zb"], h["zb"] + sq["thickness"]
    ob, hb_ = B.ring(out, U, N, h["rh"], kb, zb, False)
    ot, ht = B.ring(out, U, N, h["rh"], kb, z1, True)
    B.wall(ob, ot)
    B.face(hb_, [B.bore(h, segs, zb)], False)          # annulus under the hub, between rh and rb
    # the hub above the gear: its bottom loop is the ring's inner loop at z1
    c = _circle(h["rh"], N * kb, -math.pi / N)
    top = B.loop(c, h["z_top"])
    B.wall(ht, top)
    B.ngon(top, True)
    return B.done()


def geneva_q(q, g, spec):
    """Clamped Geneva parameters around this rail's hub: both bores are the hub's outer diameter (the
    wheel's hub runs down past the driver's plate, the driver's hub up through its locking disc), and
    the stack has to fit under the shaft tip. If the hub doesn't fit, the centre distance grows."""
    h = hub(q)
    notes = []
    p = dict(slots=spec.get("slots", 6), center_distance=spec.get("center_distance", 40.0),
             pin_diameter=spec.get("pin_diameter", 5.0), clearance=spec.get("clearance", 0.2),
             plate_thickness=spec.get("plate_thickness", 3.0), layer_gap=spec.get("layer_gap", 0.4),
             wheel_thickness=spec.get("wheel_thickness", 5.0), wheel_bore=2 * h["rh"], driver_bore=2 * h["rh"],
             segments=q["segments"], driver_gear=bool(spec.get("toothed", False)),
             gear_module=g["module"], gear_teeth=spec.get("gear_teeth", 28),
             pressure_angle=g["pressure_angle"], mate_teeth=0, tip_round=spec.get("tip_round", 1.0))
    # stack: plate + gap + wheel under the shaft tip
    room = h["room"] - STACK_ROOM
    for name, lo in (("wheel_thickness", 0.6), ("plate_thickness", 0.6)):
        over = p["plate_thickness"] + p["layer_gap"] + p["wheel_thickness"] - room
        if over > 0:
            v = max(lo, p[name] - over)
            notes.append((name, p[name], v))
            p[name] = v

    def hub_fits(c):
        r, _n = geneva.clamp(dict(p, center_distance=c))
        return abs(r["wheel_bore"] - 2 * h["rh"]) < 1e-9 and abs(r["driver_bore"] - 2 * h["rh"]) < 1e-9
    c0 = geneva.clamp(p)[0]["center_distance"]
    if not hub_fits(c0):
        c = geneva._smallest(c0, 2000.0, hub_fits)
        c = math.ceil(c * 100 - 1e-6) / 100
        if not hub_fits(c):
            c = geneva._smallest(c0, 2000.0, hub_fits)
        p["center_distance"] = c
    gp, gnotes = geneva.clamp(p)
    for n_, asked, used in gnotes:
        if n_ not in ("wheel_bore", "driver_bore"):
            notes.append((n_, asked, used))
    if abs(spec.get("center_distance", 40.0) - gp["center_distance"]) > 1e-9 and \
            not any(n_ == "center_distance" for n_, *_r in notes):
        notes.append(("center_distance", spec.get("center_distance", 40.0), gp["center_distance"]))
    gq = geneva.derive(gp)
    gq["toothed"] = gq["driver_gear"]
    return gq, notes


def build_geneva_driver(q, g, gq):
    """Plate (z = hub bottom .. + plate), locking disc + pin up to the stack top, hub to the roof."""
    h, segs = hub(q), q["segments"]
    B = _Build()
    zb = h["zb"]
    zp, zt = zb + gq["plate_thickness"], zb + gq["z_top"]
    lock = geneva._ccw(geneva.lock_outline(gq))
    pin = _circle(gq["pr"], max(12, segs // 2))
    pin = [(gq["a"] + x, y) for x, y in pin]
    lb, lt = B.loop(lock, zp), B.loop(lock, zt)
    nb, nt = B.loop(pin, zp), B.loop(pin, zt)
    B.wall(lb, lt)
    B.wall(nb, nt)
    B.ngon(nt, True)
    bore0 = B.bore(h, segs, zb)
    if gq["driver_gear"]:
        N = gq["gear_teeth"]
        kb = _kb(N, segs)
        out = geneva.gear_outline(gq["gear_module"], N, gq["pressure_angle"], segs)
        U = len(out) // N
        ri = geneva._inner_radius(gq)
        ob, ib = B.ring(out, U, N, ri, kb, zb, False)
        ot, it = B.ring(out, U, N, ri, kb, zp, True)
        B.wall(ob, ot)
        B.face(ib, [bore0], False)
        B.face(it, [lb, nb], True)
    else:
        plate = _circle(gq["plate_r"], segs)
        pb, pt = B.loop(plate, zb), B.loop(plate, zp)
        B.wall(pb, pt)
        B.face(pb, [bore0], False)
        B.face(pt, [lb, nb], True)
    hub_lo = B.hub_top(h, segs, zt)
    B.face(lt, [hub_lo], True)
    return B.done()


def build_geneva_wheel(q, g, gq):
    """Wheel raised to plate + gap on its hub; slot 0 on +X. The wheel runs up to the hub's top, so the
    top is one flat face (print it upside down, no support). Taller than the driver's pin is fine: the
    slots are through-cuts, and geneva's bore clamp keeps the wheel >= 1 mm off the driver's hub."""
    h, segs = hub(q), q["segments"]
    n = gq["slots"]
    out = geneva.wheel_outline(gq)
    out = out[-1:] + out[:-1]                 # start each repeat on the -pi/n bisector
    U = len(out) // n
    kb = _kb(n, segs)
    B = _Build()
    zb = h["zb"]
    zw = zb + gq["z_wheel"]
    c = _circle(h["rh"], n * kb, -math.pi / n)
    lo = B.loop(c, zb)
    B.face(lo, [B.bore(h, segs, zb)], False)
    ob, ib = B.ring(out, U, n, h["rh"], kb, zw, False)
    B.wall(lo, ib)
    ot, it = B.ring(out, U, n, h["rh"], kb, h["z_top"], True)
    B.wall(ob, ot)
    B.ngon(it, True)                          # flat top: the wheel's top runs over the hub roof
    return B.done()


def mount_q(q):
    """Mount dimensions in the nut frame. Rests on the shaft tip (bore ceiling = z_tip), grips the nut's
    +-X faces with fit_clearance; as wide as the nut across the rail, so neighbours never meet it."""
    h = hub(q)
    a = q["nut_length"] / 2 + q["fit_clearance"]        # pocket half-length (nut face + clearance)
    return dict(a=a, hx=a + MOUNT_WALL, hy=q["nut_width"] / 2, rb=h["rb"],
                z_grip=q["z_top"] + MOUNT_FLOAT,          # grip walls' bottom, above the body
                z_pocket=q["z_shaft0"] + MOUNT_FLOAT,     # pocket ceiling, above the collar
                z_ceiling=q["z_tip"], z_deck=h["z_top"],
                grip=q["z_nut"] - (q["z_top"] + MOUNT_FLOAT))


def build_mount(q, mq):
    """Pi-shaped block (grip walls on +-X, a bridge over the nut) extruded across the rail, with a blind
    bore from the pocket ceiling up to the shaft tip. Every face is planar; no booleans."""
    m = _Mesh()
    a, A, B, rb = mq["a"], mq["hx"], mq["hy"], mq["rb"]
    zs, zp, zc, zd = mq["z_grip"], mq["z_pocket"], mq["z_ceiling"], mq["z_deck"]
    V = m.v
    for sy in (1.0, -1.0):                      # the two Pi-shaped sides
        y = sy * B
        pi_ = [V(-A, y, zs), V(-a, y, zs), V(-a, y, zp), V(a, y, zp), V(a, y, zs), V(A, y, zs),
               V(A, y, zd), V(-A, y, zd)]
        m.face(pi_, [], (0.0, sy, 0.0))
    for sx in (1.0, -1.0):
        x = sx * A                              # outer end faces
        m.face([V(x, -B, zs), V(x, B, zs), V(x, B, zd), V(x, -B, zd)], [], (sx, 0.0, 0.0))
        xi = sx * a                             # grip faces (toward the nut)
        m.face([V(xi, -B, zs), V(xi, B, zs), V(xi, B, zp), V(xi, -B, zp)], [], (-sx, 0.0, 0.0))
        m.face([V(xi, -B, zs), V(x, -B, zs), V(x, B, zs), V(xi, B, zs)], [], (0.0, 0.0, -1.0))   # wall bottom
    n = q["segments"]
    lo = [V(rb * math.cos(2 * math.pi * k / n), rb * math.sin(2 * math.pi * k / n), zp) for k in range(n)]
    hi = [V(rb * math.cos(2 * math.pi * k / n), rb * math.sin(2 * math.pi * k / n), zc) for k in range(n)]
    for k in range(n):                          # bore wall, facing the axis
        j = (k + 1) % n
        m.faces.append((lo[j], lo[k], hi[k], hi[j]))
    m.faces.append(tuple(hi[::-1]))             # bore ceiling (rests on the shaft tip), facing down
    m.face([V(-a, -B, zp), V(a, -B, zp), V(a, B, zp), V(-a, B, zp)], [lo], (0.0, 0.0, -1.0))   # pocket ceiling
    m.face([V(-A, -B, zd), V(A, -B, zd), V(A, B, zd), V(-A, B, zd)], [], (0.0, 0.0, 1.0))      # deck
    return m.verts, m.faces


# ── Layout ───────────────────────────────────────────────────────────────────

def _mech_spec(r):
    kind = r.get("mech", "none")
    spec = dict(MECH_DEFAULTS.get(kind, {}))
    spec.update({k: v for k, v in r.items() if k not in ("mech", "link", "pos")})
    return kind, spec


def layout(spec):
    """spec = dict(rail={shaft_rail params}, gear={module, pressure_angle, backlash},
                   rails=[dict(mech=..., link=dict(rail=j, side=+1|-1) or None, pos=0..1, ...)],
                   input=k)
    Returns dict(q, gear, rails=[...], notes=[(where, name, asked, used)], issues=[text], ...)."""
    q = sr.derive(sr.clamp(spec.get("rail", {}))[0])
    rail_notes = sr.clamp(spec.get("rail", {}))[1]
    g = dict(GEAR_DEFAULTS)
    g.update(spec.get("gear", {}))
    rs = list(spec.get("rails", []))[:q["count"]]
    while len(rs) < q["count"]:
        rs.append(dict(mech="none"))
    notes = [("rail", n, a, u) for n, a, u in rail_notes]
    issues = []
    h = hub(q)
    # shared pressure angle: low enough for the fewest teeth in use
    teeth = [int(r.get("teeth", MECH_DEFAULTS["spur"]["teeth"])) for r in rs if r.get("mech") == "spur"]
    pa = float(g["pressure_angle"])
    pa_c = min(max(pa, 10.0), 30.0)
    if teeth:
        pa_c = min(pa_c, math.floor(geneva.max_pressure_angle(max(geneva.MIN_TEETH, min(teeth))) * 10) / 10)
    if abs(pa_c - pa) > 1e-9:
        notes.append(("gear", "pressure_angle", pa, pa_c))
    g["pressure_angle"] = pa_c
    m_c = min(max(float(g["module"]), 0.5), 6.0)
    if abs(m_c - g["module"]) > 1e-9:
        notes.append(("gear", "module", g["module"], m_c))
    g["module"] = m_c
    g["backlash"] = min(max(float(g["backlash"]), 0.0), 1.0)

    out = []
    for i, r in enumerate(rs):
        kind, ms = _mech_spec(r)
        item = dict(index=i, kind=kind, y=sr.carrier_y(q, i), link=r.get("link"), pos=float(r.get("pos", 0.5)))
        if kind == "spur":
            item["mq"], nn = spur_q(q, g, ms)
            notes += [("rail %d" % (i + 1), a, b, c) for a, b, c in nn]
        elif kind == "geneva_driver":
            item["mq"], nn = geneva_q(q, g, ms)
            notes += [("rail %d" % (i + 1), a, b, c) for a, b, c in nn]
        elif kind == "mount":
            item["mq"] = mount_q(q)
        out.append(item)
    # wheels take their driver's dimensions
    for it in out:
        if it["kind"] == "geneva_wheel":
            L = it["link"]
            drv = out[L["rail"]] if L and 0 <= L["rail"] < len(out) else None
            if drv is None or drv["kind"] != "geneva_driver":
                issues.append("Rail %d: a Geneva wheel has to link to a rail with a Geneva driver." % (it["index"] + 1))
                it["kind"] = "none"
            else:
                it["mq"] = drv["mq"]

    # placement: parents first; a link cycle or a bad link leaves the rail on its own slider
    def centre_distance(a, b):
        ka, kb = a["kind"], b["kind"]
        pair = {ka, kb}
        if pair == {"geneva_driver", "geneva_wheel"}:
            return a["mq"]["center_distance"] if ka == "geneva_driver" else b["mq"]["center_distance"]
        za = a["mq"]["teeth"] if ka == "spur" else (a["mq"]["gear_teeth"] if ka == "geneva_driver"
                                                    and a["mq"]["driver_gear"] else None)
        zb_ = b["mq"]["teeth"] if kb == "spur" else (b["mq"]["gear_teeth"] if kb == "geneva_driver"
                                                     and b["mq"]["driver_gear"] else None)
        if za is None or zb_ is None or "geneva_wheel" in pair or pair == {"geneva_driver"}:
            return None
        return g["module"] * (za + zb_) / 2 + g["backlash"]

    placed = {}

    def place(i, seen=()):
        if i in placed:
            return placed[i]
        it = out[i]
        x = sr.nut_x(q, min(max(it["pos"], 0.0), 1.0))
        L = it["link"]
        it["parent"] = None
        if L and L.get("rail") is not None and 0 <= L["rail"] < len(out) and L["rail"] != i:
            j = L["rail"]
            if j in seen:
                issues.append("Rails %d and %d link to each other in a loop." % (i + 1, j + 1))
            elif it["kind"] == "none" or out[j]["kind"] == "none":
                issues.append("Rail %d links to rail %d, but one of them has no mechanism." % (i + 1, j + 1))
            elif "mount" in (it["kind"], out[j]["kind"]):
                issues.append("Rail %d links to rail %d, but a fixed mount doesn't turn or mesh." % (i + 1, j + 1))
            else:
                d = centre_distance(it, out[j])
                if d is None:
                    issues.append("Rail %d (%s) can't mesh with rail %d (%s)."
                                  % (i + 1, LABELS[it["kind"]], j + 1, LABELS[out[j]["kind"]]))
                else:
                    dy = abs(it["y"] - out[j]["y"])
                    xp = place(j, seen + (i,))
                    if d < dy - 1e-9:
                        issues.append("Rail %d can't reach rail %d: the centre distance is %.1f mm but the rails "
                                      "are %.1f mm apart." % (i + 1, j + 1, d, dy))
                    else:
                        side = 1.0 if L.get("side", 1) >= 0 else -1.0
                        it["offset"] = side * math.sqrt(max(0.0, d * d - dy * dy))
                        it["parent"] = j
                        it["distance"] = d
                        x = xp + it["offset"]
        placed[i] = x
        it["x"] = x
        if x < q["x_min"] - 1e-6 or x > q["x_max"] + 1e-6:
            issues.append("Rail %d's nut would be %.1f mm past the end of its travel."
                          % (i + 1, max(q["x_min"] - x, x - q["x_max"])))
        return x

    for i in range(len(out)):
        place(i)

    # angle programs, breadth first from the input rail over the links (both ways)
    k_in = int(spec.get("input", 0))
    k_in = k_in if 0 <= k_in < len(out) else 0
    adj = {i: [] for i in range(len(out))}
    for it in out:
        if it.get("parent") is not None:
            adj[it["index"]].append(it["parent"])
            adj[it["parent"]].append(it["index"])
    progs = {k_in: dict(kind="input")}
    if out[k_in]["kind"] == "geneva_wheel":
        issues.append("Rail %d: a Geneva wheel can't be the input; turn its driver instead." % (k_in + 1))
        progs[k_in] = dict(kind="static")
    elif out[k_in]["kind"] == "mount":
        progs[k_in] = dict(kind="static")         # a fixed mount never turns
    queue = [k_in] if progs[k_in]["kind"] == "input" else []
    while queue:
        p = queue.pop(0)
        for c in adj[p]:
            if c in progs:
                continue
            P, C = out[p], out[c]
            phi = math.atan2(C["y"] - P["y"], C["x"] - P["x"])
            if C["kind"] == "geneva_wheel" and P["kind"] == "geneva_driver":
                gq = P["mq"]
                progs[c] = dict(kind="geneva", parent=p, phi=phi, a=gq["a"], C=gq["center_distance"],
                                alpha=gq["alpha"], beta=gq["beta"])
            elif P["kind"] == "geneva_wheel":
                issues.append("Rail %d: the Geneva wheel on rail %d can't drive it." % (c + 1, p + 1))
                continue
            else:
                n1 = P["mq"]["teeth"] if P["kind"] == "spur" else P["mq"]["gear_teeth"]
                n2 = C["mq"]["teeth"] if C["kind"] == "spur" else C["mq"]["gear_teeth"]
                # bmech gear_train phase rule, evaluated at parent angle 0
                ph = phi + math.pi - math.pi / n2 + (phi % (2 * math.pi / n1)) * n1 / n2
                progs[c] = dict(kind="gear", parent=p, ratio=n1 / n2, phase=ph)
            queue.append(c)
    for it in out:
        it["prog"] = progs.get(it["index"], dict(kind="static"))

    prims = [primitives(q, it) for it in out]
    knobs = knob_boxes(q)
    issues += collisions(q, out, prims, knobs)
    return dict(q=q, gear=g, hub=h, rails=out, notes=notes, issues=issues, input=k_in,
                prims=prims, knobs=knobs)


LABELS = dict(none="nothing", spur="spur gear", geneva_driver="Geneva driver", geneva_wheel="Geneva wheel",
              mount="fixed mount")


# ── Geneva / gear angle (mirrors the JS animation) ───────────────────────────

def angles(lay, theta_in):
    """Rotation of every rail's mechanism for input angle theta_in (same rules the page animates)."""
    out = {}
    order = sorted(range(len(lay["rails"])), key=lambda i: _depth(lay, i))
    for i in order:
        pr = lay["rails"][i]["prog"]
        if pr["kind"] == "input":
            out[i] = theta_in
        elif pr["kind"] == "gear":
            out[i] = pr["phase"] - pr["ratio"] * out[pr["parent"]]
        elif pr["kind"] == "geneva":
            gq = lay["rails"][pr["parent"]]["mq"]
            out[i] = pr["phi"] + geneva.wheel_angle(gq, out[pr["parent"]] - pr["phi"])
        else:
            out[i] = 0.0
    return out


def _depth(lay, i):
    d, k = 0, i
    while lay["rails"][k]["prog"].get("parent") is not None and d < 64:
        k = lay["rails"][k]["prog"]["parent"]
        d += 1
    return d


# ── Collision primitives (plan circles in z bands) ───────────────────────────

def primitives(q, it):
    """[(tag, r, z0, z1)] about the rail's shaft axis (nut frame z): plan circles. A box footprint adds
    its half-sizes: (tag, r, z0, z1, hx, hy), with r the circle around the box."""
    h = hub(q)
    k = it["kind"]
    if k == "none":
        return [("shaft", q["r_collar"], q["z_nut"], q["z_tip"])]
    if k == "mount":
        mq = it["mq"]
        return [("mount", math.hypot(mq["hx"], mq["hy"]), mq["z_grip"], mq["z_deck"], mq["hx"], mq["hy"])]
    out = [("hub", h["rh"], q["z_nut"], h["z_top"])]
    mq = it["mq"]
    zb = h["zb"]
    if k == "spur":
        out.append(("gear", mq["ra"], zb, zb + mq["thickness"]))
    elif k == "geneva_driver":
        out.append(("gear" if mq["driver_gear"] else "plate", mq["plate_r"], zb, zb + mq["plate_thickness"]))
        out.append(("crank", mq["a"] + mq["pr"], zb + mq["plate_thickness"], zb + mq["z_top"]))
    elif k == "geneva_wheel":
        out.append(("wheel", mq["tip_r"], zb + mq["z_wheel"], h["z_top"]))     # flat top at the hub roof
    return out


def knob_boxes(q):
    """Handles and endstops stick up past the body top: [(x0, x1, y0, y1, z1)] per rail (nut frame z)."""
    R = q["knob_diameter"] / 2
    x_neck = q["x_out0"] - sr.NECK_LENGTH
    x_far = x_neck - sr.KNOB_LENGTH
    x_e0 = q["x_out1"]
    x_e1 = q["screw_length"] + q["journal"] + sr.ENDSTOP_FLOAT + sr.ENDSTOP_CAP
    out = []
    for i in range(q["count"]):
        y = sr.carrier_y(q, i)
        out.append((x_far, x_neck, y - R, y + R, R))
        out.append((x_e0, x_e1, y - R, y + R, R))
    return out


def _gap(pa, xa, ya, pb, xb, yb):
    """Plan clearance between two primitives (circle or box), centred at their rails' shafts."""
    ba, bb = len(pa) > 4, len(pb) > 4
    dx, dy = abs(xa - xb), abs(ya - yb)
    if not ba and not bb:
        return math.hypot(dx, dy) - pa[1] - pb[1]
    if ba and bb:
        gx, gy = dx - pa[4] - pb[4], dy - pa[5] - pb[5]
        return math.hypot(max(gx, 0.0), max(gy, 0.0)) if gx > 0 or gy > 0 else max(gx, gy)
    box, circ = (pa, pb) if ba else (pb, pa)
    gx, gy = dx - box[4], dy - box[5]
    out = math.hypot(max(gx, 0.0), max(gy, 0.0)) if gx > 0 or gy > 0 else max(gx, gy)
    return out - circ[1]


def collisions(q, rails, prims, knobs, xs=None):
    """Plain-language list of parts that would touch. Meshing pairs are skipped where they are meant
    to overlap in plan (gear tips with gear tips, a Geneva driver with its wheel)."""
    xs = xs or [it["x"] for it in rails]
    msgs = []
    linked = set()
    for it in rails:
        if it.get("parent") is not None:
            linked.add(frozenset((it["index"], it["parent"])))
    for i in range(len(rails)):
        for j in range(i + 1, len(rails)):
            pair = frozenset((i, j))
            geneva_pair = pair in linked and {rails[i]["kind"], rails[j]["kind"]} == {"geneva_driver", "geneva_wheel"}
            for pa in prims[i]:
                ta, ra, a0, a1 = pa[:4]
                for pb in prims[j]:
                    tb, rb, b0, b1 = pb[:4]
                    if min(a1, b1) - max(a0, b0) <= 1e-6:
                        continue
                    if pair in linked and (geneva_pair or (ta == "gear" and tb == "gear")):
                        continue
                    gap = _gap(pa, xs[i], rails[i]["y"], pb, xs[j], rails[j]["y"])
                    if gap < COLLIDE_GAP - 1e-9:
                        msgs.append("Rail %d's %s hits rail %d's %s (%.1f mm short)."
                                    % (i + 1, _part(ta), j + 1, _part(tb), COLLIDE_GAP - gap))
                        break
                else:
                    continue
                break
    for i, it in enumerate(rails):
        for pr in prims[i]:
            tag, r, z0, z1 = pr[:4]
            if tag in ("shaft",):
                continue
            hx, hy = (pr[4], pr[5]) if len(pr) > 4 else (0.0, 0.0)
            for x0, x1, y0, y1, ztop in knobs:
                if z0 >= ztop:
                    continue
                dx = max(x0 - xs[i] - hx, 0.0, xs[i] - hx - x1)
                dy = max(y0 - it["y"] - hy, 0.0, it["y"] - hy - y1)
                if math.hypot(dx, dy) < (r if len(pr) <= 4 else 0.0) + COLLIDE_GAP:
                    msgs.append("Rail %d's %s hits a knob or endstop; move the nut in." % (i + 1, _part(tag)))
                    break
            else:
                continue
            break
    return msgs


def _part(tag):
    return dict(shaft="shaft", hub="hub", gear="gear", plate="crank plate", crank="pin and locking disc",
                wheel="wheel", mount="fixed mount").get(tag, tag)


# ── Meshes ───────────────────────────────────────────────────────────────────

def build_mech(lay, i):
    it = lay["rails"][i]
    q, g = lay["q"], lay["gear"]
    if it["kind"] == "spur":
        return build_spur(q, g, it["mq"])
    if it["kind"] == "geneva_driver":
        return build_geneva_driver(q, g, it["mq"])
    if it["kind"] == "geneva_wheel":
        return build_geneva_wheel(q, g, it["mq"])
    if it["kind"] == "mount":
        return build_mount(q, it["mq"])
    return None
