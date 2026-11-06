"""
geneva.py - parametric external Geneva drive (pure Python, no Blender), boolean-free.

Everything follows from the slot count n and the centre distance C (radial pin entry):

    beta  = pi / n                 half the wheel's index angle
    alpha = pi/2 - pi/n            half the crank's engagement angle
    a     = C sin(beta)            crank radius (pin circle)
    R     = C cos(beta)            radius at which the pin enters a slot (slot mouth)
    slot end centre at C - a       the pin's closest approach to the wheel centre

Conventions for the details (the parts that are design choices, not math):

    slot half-width   w2 = pin radius + clearance, semicircular end at C - a
    locking disc      r_L = a - pin radius - clearance            (on the driver)
    locking arcs      radius r_L + clearance, centred at distance C on each bisector (on the wheel)
    wheel tips        where a locking arc meets a slot edge (sharp)
    relief cut        circle of radius (wheel tip radius + clearance), centred at distance C
                      along the pin's direction, cut out of the locking disc

Layout (driver at the origin, wheel at (C, 0)); 1 BU = 1 mm:

    z = plate + gap + wheel   +---------+ pin + locking disc tops       wheel top
                              |         |                     +-----------------+
    z = plate + gap           |         |                     +-----------------+ wheel bottom
    z = plate     +-----------+---------+-------+  <- crank plate top
    z = 0         +-----------------------------+

Driver = crank plate + locking disc + pin as one watertight solid: the plate's top face is the
plate disc with the locking disc and pin outlines as holes, and those hole loops are the bottom
rings of the locking-disc and pin walls (shared vertices). Planar faces with holes are triangulated
by ear clipping (holes bridged in) followed by Delaunay edge flips.

Motion: the driver turns at constant speed (crank angle t, pin pointing at the wheel at t = 0); the
wheel follows wheel_angle(t), which is also exported as a Blender simple-expression driver.

Public API: DEFAULTS, clamp(params), derive(params), build(params) -> {"driver": (v, f), "wheel": (v, f)},
wheel_outline(q), lock_outline(q), wheel_angle(q, t), wheel_angle_expr(q), sweep(q, steps),
signed_volume(), check_closed().
"""

import math

DEFAULTS = dict(
    slots=6,
    center_distance=40.0,
    pin_diameter=5.0,
    clearance=0.2,          # slot width, locking arc gap, relief cut
    wheel_thickness=5.0,
    plate_thickness=3.0,
    layer_gap=0.4,          # vertical gap between the crank plate and the wheel
    wheel_bore=5.0,         # 0 = no hole
    driver_bore=5.0,        # 0 = no hole
    segments=64,            # per full circle
    tip_round=1.0,          # radius rounding the wheel's slot-entry tips (0 = sharp)
    # toothed crank plate: the driver's plate becomes an involute spur gear (bmech conventions,
    # tooth 0 on +X, addendum 1 m, dedendum 1.25 m), so another gear of the same module can drive it
    driver_gear=False,
    gear_module=1.0,
    gear_teeth=60,
    pressure_angle=20.0,    # degrees
    mate_teeth=0,           # > 0: also build a meshing spur gear (preview / test partner)
    mate_angle=180.0,       # degrees, direction of the mate from the driver axis (wheel is at 0)
)

MIN_WALL = 1.0
_EPS = 1e-9


# ── Derived dimensions + validation ──────────────────────────────────────────

def _core(q):
    """Closed-form dimensions shared by clamp(), derive() and the outlines."""
    n, C = q["slots"], q["center_distance"]
    pr, cl = q["pin_diameter"] / 2, q["clearance"]
    beta = math.pi / n
    a = C * math.sin(beta)
    R = C * math.cos(beta)
    # concave arcs (slot ends, locking arcs, relief) are sampled on a radius grown by the chord
    # sag, so their facets sit on the true circle instead of eating the clearance; convex parts
    # (pin, locking disc) are inscribed, which only adds clearance
    f = 1 / math.cos(math.pi / q.get("segments", DEFAULTS["segments"]))
    w2 = (pr + cl) * f
    s_end = C - a
    r_lock = a - pr - cl
    r_arc = (r_lock + cl) * f
    # locking arc (centre Cb on the bisector) meets the slot's upper edge y = w2 at x = s_tip
    cbx, cby = C * math.cos(beta), C * math.sin(beta)
    h = r_arc * r_arc - (cby - w2) ** 2
    s_tip = cbx - math.sqrt(h) if h >= 0 else R
    tip_r = math.hypot(s_tip, w2)
    k = dict(pr=pr, cl=cl, beta=beta, alpha=math.pi / 2 - beta, a=a, R=R, w2=w2, s_end=s_end,
             r_lock=r_lock, r_arc=r_arc, s_tip=s_tip, tip_r=tip_r, center_distance=C)
    k["relief_r"] = (_relief_radius(k, n) + cl) * f
    return k


def _relief_radius(k, n, steps=120):
    """Farthest any wheel tip gets from the relief centre (C, 0 in the driver frame) while it is
    over the locking disc during engagement. Textbook value is the tip radius; the tips don't
    trace that circle in the driver's turning frame, so it comes out a little larger."""
    C, alpha = k["center_distance"], k["alpha"]
    reach = (k["r_lock"] - 1e-6) ** 2       # tips resting on the locking arcs sit at r_lock + cl
    worst = k["tip_r"]
    for i in range(steps + 1):
        t = -alpha + 2 * alpha * i / steps
        psi = wheel_angle(k, t)
        ct, st = math.cos(-t), math.sin(-t)
        for j in range(n):
            for sy in (1, -1):
                ang = psi + 2 * k["beta"] * j
                x0, y0 = k["s_tip"], sy * k["w2"]
                X = C + math.cos(ang) * x0 - math.sin(ang) * y0
                Y = math.sin(ang) * x0 + math.cos(ang) * y0
                x, y = ct * X - st * Y, st * X + ct * Y
                if x * x + y * y < reach:
                    worst = max(worst, math.hypot(x - C, y))
    return worst


def _geometry_ok(q):
    k = _core(q)
    return (k["s_end"] - k["w2"] >= MIN_WALL                                      # wall at the hub
            and k["s_end"] * math.sin(k["beta"]) - k["w2"] >= MIN_WALL / 2      # between slot ends
            and k["r_lock"] >= MIN_WALL                                          # locking disc
            and k["s_tip"] - k["s_end"] >= k["w2"]                               # slot edge exists
            and q["center_distance"] - k["relief_r"] < k["r_lock"] - 0.5)        # relief leaves a disc


def clamp(p=None):
    """Pull every parameter into the range derive() accepts, in dependency order.
    Returns (params, notes) with notes = [(name, asked, used), ...] for each value that moved."""
    q = dict(DEFAULTS)
    q.update(p or {})
    asked = dict(q)

    def lim(name, lo=None, hi=None):
        v = q[name]
        if hi is not None and v > hi:
            v = hi
        if lo is not None and v < lo:
            v = lo
        q[name] = v

    for name in ("slots", "segments"):
        q[name] = int(q[name])
    lim("slots", 3, 24)
    lim("segments", 16, 256)
    lim("center_distance", 5.0)
    lim("clearance", 0.0, 1.0)
    lim("wheel_thickness", 0.6)
    lim("plate_thickness", 0.6)
    lim("layer_gap", 0.0, 5.0)
    lim("pin_diameter", 0.5)
    # pin shrinks first; if even the smallest pin can't fit, give up clearance
    if not _geometry_ok(q):
        q["pin_diameter"] = _largest(0.5, q["pin_diameter"], lambda d: _geometry_ok(dict(q, pin_diameter=d)))
    if not _geometry_ok(q):
        q["clearance"] = _largest(0.0, q["clearance"], lambda c: _geometry_ok(dict(q, clearance=c)))
    if not _geometry_ok(q):
        q["center_distance"] = _smallest(q["center_distance"], 1000.0,
                                         lambda c: _geometry_ok(dict(q, center_distance=c)))
    k = _core(q)
    lim("wheel_bore", 0.0, max(0.0, 2 * (k["s_end"] - k["w2"] - MIN_WALL)))
    room = min(k["r_lock"], q["center_distance"] - k["relief_r"]) - MIN_WALL
    lim("driver_bore", 0.0, max(0.0, 2 * room))
    if q["driver_bore"] < 0.5:
        q["driver_bore"] = 0.0
    if q["wheel_bore"] < 0.5:
        q["wheel_bore"] = 0.0
    q["driver_gear"] = bool(q["driver_gear"])
    for name in ("gear_teeth", "mate_teeth"):
        q[name] = int(q[name])
    if q["driver_gear"]:
        _clamp_gear(q, lim)
    lim("tip_round", 0.0)
    if q["tip_round"] > 0 and _tip_fillet(q, q["tip_round"]) is None:
        q["tip_round"] = math.floor(_largest(0.0, q["tip_round"], lambda r: _tip_fillet(q, r) is not None) * 100) / 100
    notes = [(n, asked[n], q[n]) for n in DEFAULTS if abs(q[n] - asked[n]) > 1e-9]
    return q, notes


def _teeth_range(q):
    """(fewest, most) driver teeth: the root circle must carry the pin (root >= a + pin r + 0.5) and
    the tips must pass the wheel's shaft (tip <= C - wheel bore/2 - clearance)."""
    k, m = _core(q), q["gear_module"]
    lo = int(math.ceil(2 * (k["a"] + k["pr"] + 0.5) / m + 2 * DEDENDUM - 1e-9))
    hi = int(math.floor(2 * (q["center_distance"] - q["wheel_bore"] / 2 - k["cl"]) / m - 2 * ADDENDUM + 1e-9))
    return max(lo, MIN_TEETH), hi


def _clamp_gear(q, lim):
    lim("gear_module", 0.2, 10.0)
    lim("pressure_angle", 10.0, 30.0)

    def fits(c):
        r = dict(q, center_distance=c)
        return _geometry_ok(r) and _teeth_range(r)[0] <= _teeth_range(r)[1]
    if not fits(q["center_distance"]):
        c = _smallest(q["center_distance"], 5000.0, fits)
        q["center_distance"] = math.ceil(c * 100 - 1e-6) / 100          # tidy, still fits
        if not fits(q["center_distance"]):
            q["center_distance"] = c
    lo, hi = _teeth_range(q)
    lim("gear_teeth", lo, hi)
    if q["mate_teeth"] > 0:
        # the mate carries the driver's bore size; its root must leave a 1 mm wall around it
        need = int(math.ceil((q["driver_bore"] / 2 + MIN_WALL) * 2 / q["gear_module"] + 2 * DEDENDUM - 1e-9))
        lim("mate_teeth", max(MIN_TEETH, need), 1000)
        ang = (q["mate_angle"] + 180.0) % 360.0 - 180.0             # into (-180, 180]
        q["mate_angle"] = 180.0 if ang == -180.0 else ang
        if not _mate_clear(q):         # keep the mate's tips off the wheel's shaft
            q["mate_angle"] = 180.0
    fewest = min(q["gear_teeth"], q["mate_teeth"] or q["gear_teeth"])
    lim("pressure_angle", None, math.floor(max_pressure_angle(fewest) * 10) / 10)


def _mate_clear(q):
    m = q["gear_module"]
    d = m * (q["gear_teeth"] + q["mate_teeth"]) / 2
    ph = math.radians(q["mate_angle"])
    gap = math.hypot(d * math.cos(ph) - q["center_distance"], d * math.sin(ph))
    return gap >= m * (q["mate_teeth"] / 2 + ADDENDUM) + q["wheel_bore"] / 2 + q["clearance"]


def _largest(lo, hi, ok, iters=40):
    """Largest value in [lo, hi] with ok(value) (assumes ok is monotone); lo if none."""
    if ok(hi):
        return hi
    if not ok(lo):
        return lo
    for _ in range(iters):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def _smallest(lo, hi, ok, iters=40):
    if ok(lo):
        return lo
    for _ in range(iters):
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


def derive(p=None):
    """Validated parameter set plus derived dimensions (raises ValueError on anything clamp() moves)."""
    q, notes = clamp(p)
    if notes:
        raise ValueError("out of range: " + ", ".join("%s %r -> %r" % n for n in notes))
    k = _core(q)
    q.update(k)
    q["plate_r"] = max(k["a"] + k["pr"] + 0.5,
                       min(k["a"] + 2 * k["pr"], q["center_distance"] - q["wheel_bore"] / 2 - k["cl"]))
    q["z_wheel"] = q["plate_thickness"] + q["layer_gap"]
    q["z_top"] = q["z_wheel"] + q["wheel_thickness"]
    q["dwell"] = 1 - k["alpha"] / math.pi            # fraction of a crank turn the wheel is locked
    if q["driver_gear"]:
        m, N = q["gear_module"], q["gear_teeth"]
        q["gear_rp"] = m * N / 2
        q["gear_ra"] = q["gear_rp"] + ADDENDUM * m
        q["gear_rd"] = q["gear_rp"] - DEDENDUM * m
        q["plate_r"] = q["gear_ra"]
        if q["mate_teeth"] > 0:
            N2 = q["mate_teeth"]
            ph = math.radians(q["mate_angle"])
            q["mate_distance"] = m * (N + N2) / 2
            q["mate_ratio"] = N / N2              # mate turns this much (opposite way) per crank radian
            # mate angle at crank angle 0 (bmech gear_train phase rule: a mate tooth space faces the
            # driver, shifted by how far phi sits from a driver tooth centre)
            q["mate_phase"] = ph + math.pi - math.pi / N2 + (ph % (2 * math.pi / N)) * N / N2
    return q


def mate_angle_at(q, t):
    """Mate gear rotation (about its own centre) for crank angle t."""
    return q["mate_phase"] - q["mate_ratio"] * t


def mate_angle_expr(q, var="t"):
    return "%.9g-%.9g*%s" % (q["mate_phase"], q["mate_ratio"], var)


# ── 2D outlines (all CCW) ────────────────────────────────────────────────────

def _arc(cx, cy, r, t0, t1, segs, include_start=True):
    """Points on a circle from angle t0 to t1 (signed sweep), at most 2pi/segs apart."""
    k = max(1, int(math.ceil(abs(t1 - t0) / (2 * math.pi) * segs - 1e-9)))
    first = 0 if include_start else 1
    return [(cx + r * math.cos(t0 + (t1 - t0) * i / k), cy + r * math.sin(t0 + (t1 - t0) * i / k))
            for i in range(first, k + 1)]


def _wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def circle(r, segs, cx=0.0, cy=0.0):
    return [(cx + r * math.cos(2 * math.pi * i / segs), cy + r * math.sin(2 * math.pi * i / segs))
            for i in range(segs)]


def wheel_outline(q):
    """Geneva wheel outline in its own frame, slot i along angle 2*pi*i/n, CCW."""
    unit = _wheel_unit(q)
    out = []
    for i in range(q["slots"]):
        c, s = math.cos(2 * q["beta"] * i), math.sin(2 * q["beta"] * i)
        out += [(c * x - s * y, s * x + c * y) for x, y in unit]
    return out


def _tip_fillet(q, rf):
    """Fillet of radius rf at slot 0's upper tip, tangent to the slot edge (y = w2) and to the locking
    arc (the wheel is outside that circle). Returns (F, T, t_arc) - fillet centre, tangent point on the
    arc, its angle about the arc centre - with the slot edge's tangent point at (F.x, w2); None if it
    doesn't fit (the straight slot edge must keep a slot width of length, and T must stay on the arc)."""
    k = q if "s_tip" in q else _core(q)
    beta, w2, C = k["beta"], k["w2"], k["center_distance"]
    cbx, cby = C * math.cos(beta), C * math.sin(beta)
    R = k["r_arc"] + rf
    h = R * R - (w2 + rf - cby) ** 2
    if h < 0:
        return None
    fx, fy = cbx - math.sqrt(h), w2 + rf
    tx, ty = cbx + (fx - cbx) * k["r_arc"] / R, cby + (fy - cby) * k["r_arc"] / R
    t_arc = math.atan2(ty - cby, tx - cbx)
    t1 = math.pi + beta
    t_tip = math.atan2(w2 - cby, k["s_tip"] - cbx)
    span = _wrap(t1 - t_tip)                        # the arc runs from the tip to the bisector
    used = _wrap(t_arc - t_tip)
    if fx < k["s_end"] + k["w2"] or not (0 <= used * (1 if span > 0 else -1) < abs(span) - 1e-6):
        return None
    return (fx, fy), (tx, ty), t_arc


def _wheel_unit(q):
    """Slot 0's stretch of the outline: from just past the bisector at -beta, around slot 0, to the
    bisector point at +beta (inclusive). n rotated copies make the whole outline."""
    segs = q["segments"]
    beta, w2, s_end, C = q["beta"], q["w2"], q["s_end"], q["center_distance"]
    cbx, cby = C * math.cos(beta), C * math.sin(beta)
    # half sector: slot 0's end apex -> upper slot edge -> tip -> locking arc -> bisector
    half = _arc(s_end, 0.0, w2, math.pi, math.pi / 2, segs)          # quarter of the slot end
    rf = q.get("tip_round", 0.0)
    fil = _tip_fillet(q, rf) if rf > 0 else None
    if fil:
        (fx, fy), _t, t0 = fil
        # slot edge to the fillet, then round the tip (convex: inscribed chords only add clearance)
        a_end = math.atan2(_t[1] - fy, _t[0] - fx)
        half += _arc(fx, fy, rf, -math.pi / 2, -math.pi / 2 + (a_end + math.pi / 2) % (2 * math.pi), segs)
    else:
        half.append((q["s_tip"], w2))                                 # slot edge up to the tip
        t0 = math.atan2(w2 - cby, q["s_tip"] - cbx)
    t1 = math.pi + beta                                               # deepest point, on the bisector
    half += _arc(cbx, cby, q["r_arc"], t0, t0 + _wrap(t1 - t0), segs, include_start=False)
    mirror = [(x, -y) for x, y in reversed(half)]                     # bisector(-) -> apex
    return mirror[1:] + half[1:]                                       # drop shared bisector / apex


def lock_outline(q):
    """Locking disc minus the relief cut, in the driver frame (pin on +X), CCW."""
    segs, C = q["segments"], q["center_distance"]
    rl, rr = q["r_lock"], q["relief_r"]
    xi = (rl * rl + C * C - rr * rr) / (2 * C)
    yi = math.sqrt(max(0.0, rl * rl - xi * xi))
    phi = math.atan2(yi, xi)
    pts = _arc(0.0, 0.0, rl, phi, 2 * math.pi - phi, segs)
    s0 = math.atan2(-yi, xi - C) % (2 * math.pi)                      # in (pi, 2pi)
    s1 = math.atan2(yi, xi - C)                                       # in (0, pi)
    pts += _arc(C, 0.0, rr, s0, s1, segs, include_start=False)[:-1]   # relief, back to the start
    return pts


# ── Kinematics ───────────────────────────────────────────────────────────────

def wheel_angle(q, t):
    """Wheel rotation (rad, about its own centre) for crank angle t; wheel at (C, 0), driver at the
    origin, pin at angle t. At t = 0 the pin points at the wheel and slot 0 points back at the driver."""
    k = math.floor((t + math.pi) / (2 * math.pi))
    w = t - 2 * math.pi * k
    if abs(w) < q["alpha"]:
        phi = math.atan2(q["a"] * math.sin(w), q["center_distance"] - q["a"] * math.cos(w))
    else:
        phi = q["beta"] if w > 0 else -q["beta"]
    return math.pi - 2 * q["beta"] * k - phi


def wheel_angle_expr(q, var="t"):
    """wheel_angle() as a Blender simple expression (no Python driver namespace needed). Clamping
    the crank angle to +-alpha makes atan2 land exactly on +-beta, so the dwell needs no branch.
    Blender stores driver expressions in 256 bytes, hence the short form."""
    k = "floor(%s/%.9g+.5)" % (var, 2 * math.pi)
    c = "min(max(%s-%.9g*%s,%.9g),%.9g)" % (var, 2 * math.pi, k, -q["alpha"], q["alpha"])
    e = "pi-%.9g*%s-atan2(%.9g*sin(%s),%.9g-%.9g*cos(%s))" % (
        2 * q["beta"], k, q["a"], c, q["center_distance"], q["a"], c)
    if len(e) > 255:
        raise ValueError("driver expression too long (%d)" % len(e))
    return e


def sweep(q, steps=720):
    """Turn the crank once (the motion repeats every turn) and measure, in the plane of the wheel:
    pin -> wheel edge and locking disc -> wheel edge. Returns the minimum clearance of each
    (negative = collision) and the crank angles where they occur."""
    C, a, pr = q["center_distance"], q["a"], q["pr"]
    wheel = wheel_outline(q)
    lock = lock_outline(q)
    rl, rr = q["r_lock"], q["relief_r"]
    worst_pin, worst_lock = (1e9, 0.0), (1e9, 0.0)
    for i in range(steps):
        t = -math.pi + 2 * math.pi * i / steps
        psi = wheel_angle(q, t)
        # wheel outline -> driver frame
        cw, sw = math.cos(psi), math.sin(psi)
        ct, st = math.cos(-t), math.sin(-t)
        wd = []
        for x, y in wheel:
            X, Y = C + cw * x - sw * y, sw * x + cw * y
            wd.append((ct * X - st * Y, st * X + ct * Y))
        # pin: centre (a, 0) in the driver frame
        d = _poly_dist((a, 0.0), wd, near=pr + 2.0)
        if d is None:
            d = 1e9
        elif _inside((a, 0.0), wd):
            d = -d
        if d - pr < worst_pin[0]:
            worst_pin = (d - pr, t)
        # locking disc: wheel vertices against the lock region (disc r_L minus relief circle)
        for x, y in wd:
            if x * x + y * y > (rl + 2) ** 2:
                continue
            s = max(math.hypot(x, y) - rl, rr - math.hypot(x - C, y))
            if s < worst_lock[0]:
                worst_lock = (s, t)
        # and lock vertices against the wheel polygon
        for p in lock:
            dd = _poly_dist(p, wd, near=2.0)
            if dd is None:
                continue
            if _inside(p, wd):
                dd = -dd
            if dd < worst_lock[0]:
                worst_lock = (dd, t)
    return dict(pin=worst_pin[0], pin_at=worst_pin[1], lock=worst_lock[0], lock_at=worst_lock[1])


def _poly_dist(p, poly, near=None):
    """Distance from p to the closed polyline poly (None if nothing is within `near`)."""
    best = 1e18
    px, py = p
    m = len(poly)
    for i in range(m):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        if near is not None and (min(ax, bx) - near > px or max(ax, bx) + near < px
                                 or min(ay, by) - near > py or max(ay, by) + near < py):
            continue
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        u = 0.0 if L < 1e-24 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
        d = (px - ax - u * dx) ** 2 + (py - ay - u * dy) ** 2
        if d < best:
            best = d
    if best == 1e18:
        return None
    return math.sqrt(best)


def _inside(p, poly):
    px, py = p
    c = False
    m = len(poly)
    for i in range(m):
        ax, ay = poly[i - 1]
        bx, by = poly[i]
        if (ay > py) != (by > py) and px < ax + (py - ay) * (bx - ax) / (by - ay):
            c = not c
    return c


# ── Triangulation of planar faces with holes ─────────────────────────────────

def _area(poly):
    return 0.5 * sum(poly[i - 1][0] * poly[i][1] - poly[i][0] * poly[i - 1][1] for i in range(len(poly)))


def _orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def triangulate(pts, outer, holes):
    """Triangles (index triples into pts, CCW) covering the polygon `outer` (CCW index loop) minus
    `holes` (CW index loops). Holes are bridged into the outer loop, the result ear-clipped, then
    edges are flipped to Delaunay (loop edges stay fixed)."""
    ring = list(outer)
    for hole in sorted(holes, key=lambda h: -max(pts[i][0] for i in h)):
        ring = _bridge(pts, ring, hole)
    tris = _earclip(pts, ring)
    fixed = set()
    for loop in [outer] + list(holes):
        for i in range(len(loop)):
            fixed.add(frozenset((loop[i - 1], loop[i])))
    return _delaunay(pts, tris, fixed)


def _bridge(pts, ring, hole):
    """Splice `hole` into `ring` through a mutually visible pair (Eberly's rightmost-vertex method)."""
    hi = max(range(len(hole)), key=lambda i: (pts[hole[i]][0], pts[hole[i]][1]))
    M = pts[hole[hi]]
    best, best_x, edge = None, 1e18, None
    for i in range(len(ring)):
        A, B = pts[ring[i - 1]], pts[ring[i]]
        if (A[1] > M[1]) == (B[1] > M[1]):
            if A[1] == M[1] and A[0] >= M[0] and A[0] < best_x:
                best_x, best, edge = A[0], i - 1, None
            continue
        x = A[0] + (M[1] - A[1]) * (B[0] - A[0]) / (B[1] - A[1])
        if M[0] <= x < best_x:
            best_x, edge = x, (i - 1, i)
    if edge is not None:
        i0, i1 = edge
        A, B = pts[ring[i0]], pts[ring[i1]]
        best = i0 if A[0] > B[0] else i1
        P = pts[ring[best]]
        I = (best_x, M[1])
        # a reflex vertex inside triangle M-I-P blocks P: take the one closest in angle to the ray
        tri = (M, I, P) if _orient(M, I, P) > 0 else (M, P, I)
        best_ang = None
        for j in range(len(ring)):
            Q = pts[ring[j]]
            if Q == P or Q == M:
                continue
            if _in_tri(Q, *tri) and _orient(pts[ring[j - 1]], Q, pts[ring[(j + 1) % len(ring)]]) <= 0:
                ang = abs(math.atan2(Q[1] - M[1], Q[0] - M[0]))
                dist = (Q[0] - M[0]) ** 2 + (Q[1] - M[1]) ** 2
                if best_ang is None or (ang, dist) < best_ang:
                    best_ang, best = (ang, dist), j
    h = hole[hi:] + hole[:hi]
    return ring[:best + 1] + h + [h[0], ring[best]] + ring[best + 1:]


def _in_tri(p, a, b, c, eps=1e-12):
    return _orient(a, b, p) >= -eps and _orient(b, c, p) >= -eps and _orient(c, a, p) >= -eps


def _earclip(pts, ring):
    """Ear clipping over a doubly linked ring; only reflex vertices can block an ear, and the scan
    resumes where the last ear was cut (O(n * reflex) instead of restarting every time)."""
    m = len(ring)
    nxt = [(i + 1) % m for i in range(m)]
    prv = [(i - 1) % m for i in range(m)]
    P = [pts[i] for i in ring]

    def reflex(i):
        return _orient(P[prv[i]], P[i], P[nxt[i]]) <= 1e-14

    refl = {i for i in range(m) if reflex(i)}
    tris = []
    i, left, stall = 0, m, 0
    while left > 3:
        a, b, c = prv[i], i, nxt[i]
        ear = b not in refl
        if ear:
            A, B, Cc = P[a], P[b], P[c]
            x0, x1 = min(A[0], B[0], Cc[0]), max(A[0], B[0], Cc[0])
            y0, y1 = min(A[1], B[1], Cc[1]), max(A[1], B[1], Cc[1])
            for j in refl:
                if j in (a, c):
                    continue
                Q = P[j]
                if Q[0] < x0 or Q[0] > x1 or Q[1] < y0 or Q[1] > y1 or Q in (A, B, Cc):
                    continue
                if _in_tri(Q, A, B, Cc):
                    ear = False
                    break
        if ear:
            tris.append((ring[a], ring[b], ring[c]))
            nxt[a], prv[c] = c, a
            refl.discard(b)
            left -= 1
            for k in (a, c):
                if reflex(k):
                    refl.add(k)
                else:
                    refl.discard(k)
            i, stall = c, 0
            continue
        i = nxt[i]
        stall += 1
        if stall > left:
            # only degenerate (collinear) vertices left: drop the flattest one
            j, best = i, None
            for _ in range(left):
                o = abs(_orient(P[prv[j]], P[j], P[nxt[j]]))
                if best is None or o < best[0]:
                    best = (o, j)
                j = nxt[j]
            b = best[1]
            a, c = prv[b], nxt[b]
            nxt[a], prv[c] = c, a
            refl.discard(b)
            left -= 1
            for k in (a, c):
                if reflex(k):
                    refl.add(k)
                else:
                    refl.discard(k)
            i, stall = c, 0
    a = i
    b, c = nxt[a], nxt[nxt[a]]
    if _orient(P[a], P[b], P[c]) > 0:
        tris.append((ring[a], ring[b], ring[c]))
    return tris


def _delaunay(pts, tris, fixed):
    tris = [list(t) for t in tris]
    for _pass in range(200):
        edges = {}
        for ti, t in enumerate(tris):
            for k in range(3):
                edges.setdefault(frozenset((t[k], t[(k + 1) % 3])), []).append(ti)
        flipped = 0
        done = set()
        for e, ts in edges.items():
            if len(ts) != 2 or e in fixed or ts[0] in done or ts[1] in done:
                continue
            t0, t1 = tris[ts[0]], tris[ts[1]]
            u, v = tuple(e)
            w = next(x for x in t0 if x not in e)
            x = next(y for y in t1 if y not in e)
            # orient t0 as (u, v, w) CCW
            if _orient(pts[u], pts[v], pts[w]) < 0:
                u, v = v, u
            if not _incircle(pts[u], pts[v], pts[w], pts[x]):
                continue
            # quad u-x-v-w must be convex for the flip
            if _orient(pts[w], pts[x], pts[v]) <= 1e-12 or _orient(pts[x], pts[w], pts[u]) <= 1e-12:
                continue
            tris[ts[0]] = [w, u, x]
            tris[ts[1]] = [x, v, w]
            done.update(ts)
            flipped += 1
        if not flipped:
            break
    return [tuple(t) for t in tris]


def _incircle(a, b, c, d):
    """True if d is strictly inside the circumcircle of CCW triangle a, b, c."""
    ax, ay = a[0] - d[0], a[1] - d[1]
    bx, by = b[0] - d[0], b[1] - d[1]
    cx, cy = c[0] - d[0], c[1] - d[1]
    det = ((ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
           + (cx * cx + cy * cy) * (ax * by - bx * ay))
    scale = max(abs(ax), abs(ay), abs(bx), abs(by), abs(cx), abs(cy), 1.0) ** 4
    return det > 1e-10 * scale


# ── Involute spur gear outline (same profile as bmech spur_gear_core) ────────

ADDENDUM, DEDENDUM, MIN_TEETH = 1.0, 1.25, 8


def _tooth_ok(N, pa_deg):
    pa = math.radians(pa_deg)
    ratio = (1 + 2 * ADDENDUM / N) / math.cos(pa)
    if ratio < 1:
        return True
    tp, tt = math.tan(pa), math.sqrt(ratio * ratio - 1)
    return math.pi / (2 * N) > (tt - math.atan(tt)) - (tp - math.atan(tp))


def max_pressure_angle(N):
    """Largest pressure angle (deg) before the N-tooth gear's flanks cross below the tip."""
    return math.degrees(_largest(0.0, math.radians(89), lambda a: _tooth_ok(N, math.degrees(a))))


def gear_unit(m, N, pa_deg, segs):
    """One tooth of the outline, CCW, from the space centre at -pi/N up to (not including) the space
    centre at +pi/N. Tooth centred on +X. N rotated copies make the whole gear."""
    pa = math.radians(pa_deg)
    rp = m * N / 2
    rb, ra, rd = rp * math.cos(pa), rp + ADDENDUM * m, rp - DEDENDUM * m

    def inv(t):
        return rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t))

    def t_of(r):
        return math.sqrt(max(0.0, (r / rb) ** 2 - 1))
    t0, t1 = t_of(max(rd, rb)), t_of(ra)
    px, py = inv(t_of(rp))
    rot = math.pi / (2 * N) + math.atan2(py, px)
    c, s_ = math.cos(rot), math.sin(rot)
    npts = max(6, segs // 5)
    flank = []                                            # upper flank (+y), root -> tip
    for i in range(npts):
        x, y = inv(t0 + (t1 - t0) * i / (npts - 1))
        flank.append((c * x + s_ * y, s_ * x - c * y))    # mirror, then rotate by rot
    root_ang = math.atan2(flank[0][1], flank[0][0])
    if rb > rd:                                           # undercut: radial line down to the root
        flank.insert(0, (rd * math.cos(root_ang), rd * math.sin(root_ang)))
    half = math.pi / N
    k = max(2, int(math.ceil((half - root_ang) / (2 * math.pi) * segs)))
    # space arc from the -half space centre up to (not including) the lower flank's root
    space = [(rd * math.cos(-half + (half - root_ang) * j / k), rd * math.sin(-half + (half - root_ang) * j / k))
             for j in range(k)]
    lower = [(x, -y) for x, y in flank]
    upper_space = [(x, -y) for x, y in reversed(space[1:])]
    return space + lower + [(ra, 0.0)] + flank[::-1] + upper_space


def gear_outline(m, N, pa_deg, segs):
    unit = gear_unit(m, N, pa_deg, segs)
    out = []
    for i in range(N):
        c, s = math.cos(2 * math.pi * i / N), math.sin(2 * math.pi * i / N)
        out += [(c * x - s * y, s * x + c * y) for x, y in unit]
    return out


def _gear_body(mesh, m, N, pa_deg, segs, ri, z0, z1, bottom_holes, top_holes):
    """Gear plate z0..z1 into `mesh`: the tooth ring (outline to an inner circle of radius ri) is
    triangulated once per tooth and rotated; the inner disc is capped around the given hole loops
    (CW vertex loops already in the mesh at z0 / z1)."""
    unit = gear_unit(m, N, pa_deg, segs)
    U, NU = len(unit), N * len(unit)
    kb = max(1, int(math.ceil(segs / N)))
    out = gear_outline(m, N, pa_deg, segs)
    inner = [(ri * math.cos(-math.pi / N + 2 * math.pi * j / (N * kb)),
              ri * math.sin(-math.pi / N + 2 * math.pi * j / (N * kb))) for j in range(N * kb)]
    flat = out + inner
    # tooth 0's wedge: outline from the -pi/N space centre to the +pi/N one, back along the inner arc
    wedge = list(range(U)) + [U % NU] + [NU + (kb % (N * kb))] + list(range(NU + kb - 1, NU - 1, -1))
    local = [flat[i] for i in wedge]
    tris = triangulate(local, list(range(len(wedge))), [])
    tris = [tuple(wedge[i] for i in t) for t in tris]

    def shift(g, i):
        return (g + i * U) % NU if g < NU else NU + (g - NU + i * kb) % (N * kb)
    ring = [tuple(shift(g, i) for g in t) for i in range(N) for t in tris]
    ob, ot = mesh.loop(out, z0), mesh.loop(out, z1)
    ib, it = mesh.loop(inner, z0), mesh.loop(inner, z1)
    for t in ring:
        mesh.faces.append(tuple(ot[g] if g < NU else it[g - NU] for g in t))
        mesh.faces.append(tuple(ob[g] if g < NU else ib[g - NU] for g in t[::-1]))
    mesh.wall(ob, ot)
    mesh.cap(ib, bottom_holes, False)
    mesh.cap(it, top_holes, True)


def _inner_radius(q):
    """Inner circle of the driver's tooth ring: between the pin's outer edge and the root circle."""
    return q["gear_rd"] - 0.4 * (q["gear_rd"] - q["a"] - q["pr"])


def build_mate(q):
    m = _Mesh()
    segs, N2 = q["segments"], q["mate_teeth"]
    rd2 = q["gear_module"] * (N2 / 2 - DEDENDUM)
    zp = q["plate_thickness"]
    holes_b, holes_t, ri = [], [], rd2 * 0.7
    if q["driver_bore"] > 0:
        b = circle(q["driver_bore"] / 2, segs)[::-1]
        hb, ht = m.loop(b, 0.0), m.loop(b, zp)
        m.wall(hb, ht)
        holes_b, holes_t = [hb], [ht]
        ri = (rd2 + q["driver_bore"] / 2) / 2
    _gear_body(m, q["gear_module"], N2, q["pressure_angle"], segs, ri, 0.0, zp, holes_b, holes_t)
    return m.verts, m.faces


# ── Solids ───────────────────────────────────────────────────────────────────

class _Mesh:
    def __init__(self):
        self.verts, self.faces = [], []

    def loop(self, pts2d, z):
        o = len(self.verts)
        self.verts += [(x, y, z) for x, y in pts2d]
        return list(range(o, o + len(pts2d)))

    def cap(self, outer, holes, up):
        """Planar face over vertex loops (outer CCW, holes CW seen from +Z)."""
        pts = [(v[0], v[1]) for v in self.verts]
        for t in triangulate(pts, outer, holes):
            self.faces.append(t if up else t[::-1])

    def wall(self, bottom, top):
        """Quads between two matching loops; outward for CCW outer / CW hole loops."""
        m = len(bottom)
        for i in range(m):
            j = (i + 1) % m
            self.faces.append((bottom[i], bottom[j], top[j], top[i]))


def _ccw(pts):
    return pts if _area(pts) > 0 else pts[::-1]


def build_wheel(q):
    """Caps are triangulated once for slot 0's wedge (bisector to bisector, outline to bore) and
    rotated, so every slot has identical topology."""
    n, beta = q["slots"], q["beta"]
    unit = _wheel_unit(q)
    U, N = len(unit), n * len(unit)
    out = wheel_outline(q)
    rb = q["wheel_bore"] / 2
    kb = max(1, int(math.ceil(q["segments"] / n))) if rb > 0 else 0
    # bore vertex i*kb + m sits at angle -beta + 2*beta*(i + m/kb): kb per wedge, CCW
    bore = [(rb * math.cos(-beta + 2 * beta * j / kb), rb * math.sin(-beta + 2 * beta * j / kb))
            for j in range(n * kb)]
    flat = out + (bore if rb > 0 else [(0.0, 0.0)])
    # wedge 0 as indices into flat: B- (end of the previous unit), slot 0's unit (ends at B+),
    # down the +beta bisector to the bore, back along the bore, up the -beta bisector
    wedge = [N - 1] + list(range(U))
    wedge += [N + kb] + list(range(N + kb - 1, N - 1, -1)) if rb > 0 else [N]
    if rb > 0 and n == 1:
        raise ValueError("needs at least 2 slots")
    local = [flat[i] for i in wedge]
    tris = triangulate(local, list(range(len(wedge))), [])
    tris = [tuple(wedge[i] for i in t) for t in tris]

    def shift(g, i):
        if g < N:
            return (g + i * U) % N
        if rb <= 0:
            return g
        return N + (g - N + i * kb) % (n * kb)

    cap = [tuple(shift(g, i) for g in t) for i in range(n) for t in tris]
    F = len(flat)
    verts = [(x, y, q["z_wheel"]) for x, y in flat] + [(x, y, q["z_top"]) for x, y in flat]
    faces = [tuple(g + F for g in t) for t in cap] + [t[::-1] for t in cap]
    m = _Mesh()
    m.verts = verts
    m.faces = faces
    m.wall(list(range(N)), [g + F for g in range(N)])
    if rb > 0:
        hole = list(range(N + n * kb - 1, N - 1, -1))                 # CW
        m.wall(hole, [g + F for g in hole])
    else:                                                             # centre vertex: no wall
        pass
    return m.verts, m.faces


def pin_outline(q):
    return circle(q["pr"], max(12, q["segments"] // 2), q["a"], 0.0)


def build_pin(q):
    """The pin alone (plate top to z_top), for build_driver(q, pin=False)."""
    m = _Mesh()
    pin = pin_outline(q)
    nb, nt = m.loop(pin, q["plate_thickness"]), m.loop(pin, q["z_top"])
    m.wall(nb, nt)
    m.faces.append(tuple(nt))
    m.faces.append(tuple(nb[::-1]))
    return m.verts, m.faces


def build_driver(q, pin=True):
    """pin=False leaves the pin off (the plate top runs under it), e.g. to colour it as its own object
    with build_pin()."""
    m = _Mesh()
    segs = q["segments"]
    zp, zt = q["plate_thickness"], q["z_top"]
    lock = _ccw(lock_outline(q))
    lb, lt = m.loop(lock, zp), m.loop(lock, zt)
    m.wall(lb, lt)
    pin_holes = []
    if pin:
        nb, nt = m.loop(pin_outline(q), zp), m.loop(pin_outline(q), zt)
        m.wall(nb, nt)
        m.faces.append(tuple(nt))                                    # pin top: convex n-gon
        pin_holes = [nb[::-1]]
    bottom_holes = []
    if q["driver_bore"] > 0:
        b = circle(q["driver_bore"] / 2, segs)[::-1]
        hb, ht = m.loop(b, 0.0), m.loop(b, zt)
        m.wall(hb, ht)
        m.cap(lt, [ht], True)
        bottom_holes = [hb]
    else:
        m.cap(lt, [], True)
    if q["driver_gear"]:
        ri = _inner_radius(q)
        kin = q["gear_teeth"] * max(1, int(math.ceil(segs / q["gear_teeth"])))
        assert q["a"] + q["pr"] < ri * math.cos(math.pi / kin), "pin outside the gear's inner disc"
        _gear_body(m, q["gear_module"], q["gear_teeth"], q["pressure_angle"], segs, ri, 0.0, zp,
                   bottom_holes, [lb[::-1]] + pin_holes)
    else:
        plate = circle(q["plate_r"], segs)
        pb, pt = m.loop(plate, 0.0), m.loop(plate, zp)
        m.wall(pb, pt)
        m.cap(pt, [lb[::-1]] + pin_holes, True)                      # plate top around both
        m.cap(pb, bottom_holes, False)
    return m.verts, m.faces


def build(p=None):
    """{"driver": (verts, faces), "wheel": (verts, faces)}; wheel centred on (C, 0) in its own frame
    at the origin (place it at x = C), slot 0 on +X."""
    q = derive(p)
    parts = {"driver": build_driver(q), "wheel": build_wheel(q)}
    if q["driver_gear"] and q["mate_teeth"] > 0:
        parts["mate"] = build_mate(q)
    return parts


# ── Checks ───────────────────────────────────────────────────────────────────

def signed_volume(verts, faces):
    vol = 0.0
    for f in faces:
        a = verts[f[0]]
        for i in range(1, len(f) - 1):
            b, c = verts[f[i]], verts[f[i + 1]]
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0]))
    return vol / 6.0


def check_closed(faces):
    """Directed edges not matched by exactly one opposite edge (empty = closed + oriented)."""
    count = {}
    for f in faces:
        for i in range(len(f)):
            e = (f[i], f[(i + 1) % len(f)])
            count[e] = count.get(e, 0) + 1
    return [e for e, c in count.items() if c != 1 or count.get((e[1], e[0]), 0) != 1]


if __name__ == "__main__":
    for label, p in (("defaults", {}), ("n=4", dict(slots=4)), ("n=8 C=60", dict(slots=8, center_distance=60)),
                     ("gear", dict(driver_gear=True, mate_teeth=20))):
        q = derive(clamp(p)[0])
        parts = build(q)
        s = sweep(q, 360)
        print("%-9s a %.2f  tip r %.2f  lock r %.2f  relief r %.2f  dwell %.0f%%"
              % (label, q["a"], q["tip_r"], q["r_lock"], q["relief_r"], 100 * q["dwell"]))
        for name, (v, f) in parts.items():
            print("   %-6s verts %5d faces %5d  volume %9.2f  open edges %d"
                  % (name, len(v), len(f), signed_volume(v, f), len(check_closed(f))))
        print("   sweep: pin clearance %.3f (t=%.1f deg)  lock clearance %.3f (t=%.1f deg)"
              % (s["pin"], math.degrees(s["pin_at"]), s["lock"], math.degrees(s["lock_at"])))
