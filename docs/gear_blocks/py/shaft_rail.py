"""
shaft_rail.py - parametric row of lead-screw carriers (pure Python, no Blender), boolean-free.
Defaults are measured from lead_screw_bones.blend. Parts, all in the screw's frame (X along the
screw, x = 0 at the start of the thread, screw axis on y = z = 0):

    body     one block with a channel per carrier; neighbours share a wall
    screw    right-hand trapezoid thread (one or more starts), plain journal and square socket at each end
    handle   knurled knob; its sleeve runs in the end wall, its square peg plugs into the screw
    endstop  same sleeve and peg on the far end, holds the screw axially against the handle; its cap is
             a disc as wide as the knob, so it is not a tiny loose part
    nut      block guided by the channel walls, threaded through, carries a collar and shaft

Mechanisms drop over a shaft with a capped hub (rail_mechs.py), so there is no separate stop cap.

The wall bores pass the thread, so a screw slides in through one end wall and drops into its channel.

Public API: DEFAULTS, clamp(params), derive(params), carrier_y(q, i), nut_x(q, t), build_body(q),
build_screw(q), build_handle(q), build_endstop(q), build_nut(q).
"""

import math
import os
import sys

# Blender's Text Editor runs a text block with no real __file__, so find the sibling modules here
_HERE = r"C:\Users\Safu\OneDrive\Desktop\Blender\shaft_rail"
if not os.path.isfile(os.path.join(_HERE, "mesh2d.py")):
    _HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from mesh2d import _Mesh, _area  # noqa: E402

DEFAULTS = dict(
    count=3,                # carriers side by side
    thread_diameter=10.0,
    thread_pitch=1.6,       # axial distance between neighbouring crests
    thread_starts=4,        # lead = starts x pitch per turn
    screw_length=100.0,     # threaded length; sets the body length
    nut_length=10.0,        # along the screw
    nut_width=20.0,         # across the channel
    nut_height=15.0,
    shaft_diameter=8.0,
    shaft_length=14.0,      # collar top to tip; a mechanism's hub covers all of it
    wall_thickness=3.2,     # floor, side and end walls
    floor_slot=5.6,         # width of the slot through each channel's floor, 0 for none
    knob_diameter=20.67,
    thread_clearance=0.3,   # nut to screw, radial (0.15 printed too tight, 2026-11-05)
    fit_clearance=0.15,     # nut to channel, sleeve to wall bore
    peg_clearance=0.1,      # square peg to the screw's socket, per side
    shaft_clearance=0.2,    # mechanism hub bore to the shaft, radial (running fit)
    segments=64,            # per full circle, rounded to a multiple of 4
)
ROOT_W, FLANK_W, CREST_W = 0.25, 0.3125, 0.125   # thread profile, x pitch
THREAD_DEPTH = 0.3125       # x pitch, 45 degree flanks
MIN_THREAD = 6.0            # below this the square peg gets too small to print
MIN_SLEEVE = 0.4            # sleeve wall around the journal
JOURNAL_CLEARANCE = 0.1     # journal to sleeve, radial
GAP_HANDLE, GAP_ENDSTOP = 0.2, 0.8    # thread end to the inside of the end wall
JOURNAL_EXTRA = 1.8         # journal length beyond the wall thickness
ENDSTOP_FLOAT = 0.6         # journal end to the endstop's cup floor
ENDSTOP_CAP = 3.2
FLOOR_GAP = 0.45            # nut to channel floor
NUT_MIN_WALL = 1.2          # around the bore, in the nut and above it in the end wall
NECK_LIP = 1.2              # handle neck and endstop cap, beyond the sleeve
NECK_LENGTH, KNOB_LENGTH = 4.8, 15.0
KNOB_CHAMFER = 0.6
KNURL_DEPTH = 0.258
KNURL_PITCH = 2.03          # along the circumference
KNOB_GAP = 1.0              # between neighbouring knobs
COLLAR_LIP, MIN_COLLAR_LIP = 1.0, 0.6
COLLAR_HEIGHT, COLLAR_FILLET = 2.0, 0.4
COLLAR_INSET = 0.05         # keeps the collar off the block's edge, which would pinch the top face
MAX_LEAD_ANGLE = 20.0       # degrees at the pitch diameter; caps the thread starts


def clamp(p=None):
    """(params, notes): out-of-range values replaced, notes = [(name, asked, used)]."""
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

    q["count"] = int(q["count"])
    lim("count", 1, 16)
    q["segments"] = int(q["segments"])
    lim("segments", 24, 240)
    q["segments"] += -q["segments"] % 4
    lim("thread_clearance", 0.05, 0.6)
    lim("fit_clearance", 0.05, 0.6)
    lim("peg_clearance", 0.0, 0.4)
    lim("shaft_clearance", 0.05, 0.6)
    lim("wall_thickness", 2.4, 20.0)
    lim("thread_diameter", MIN_THREAD, 60.0)
    # the root keeps at least 60% of the major radius
    lim("thread_pitch", 0.8, 0.4 * q["thread_diameter"] / 2 / THREAD_DEPTH)
    q["thread_starts"] = int(q["thread_starts"])
    d_pitch = q["thread_diameter"] - THREAD_DEPTH * q["thread_pitch"]
    most = math.tan(math.radians(MAX_LEAD_ANGLE)) * math.pi * d_pitch / q["thread_pitch"]
    lim("thread_starts", 1, max(1, min(8, int(most + 1e-9))))
    bore = q["thread_diameter"] / 2 + q["fit_clearance"]
    nut_min = 2 * (q["thread_diameter"] / 2 + q["thread_clearance"] + NUT_MIN_WALL)
    lim("nut_width", max(nut_min, 2 * (bore + NUT_MIN_WALL)), 200.0)
    lim("nut_height", max(nut_min, 2 * (bore + NUT_MIN_WALL)), 200.0)
    lim("nut_length", max(3 * q["thread_pitch"], 4.0), 200.0)
    lim("floor_slot", 0.0, q["nut_width"] - 4.0)
    lim("screw_length", q["nut_length"] + q["thread_pitch"], 2000.0)
    # the collar has to fit the nut's top, and the shaft has to leave the collar a lip
    collar_max = min(q["nut_length"], q["nut_width"]) - 2 * COLLAR_INSET
    lim("shaft_diameter", 3.0, max(3.0, collar_max - 2 * MIN_COLLAR_LIP))
    lim("shaft_length", min(q["shaft_diameter"] / 2, 2.0) + 1.0, 400.0)
    # neighbouring knobs must clear each other; the neck sets the floor
    spacing = q["nut_width"] + 2 * q["fit_clearance"] + q["wall_thickness"]
    neck = 2 * (q["thread_diameter"] / 2 + NECK_LIP)
    lim("knob_diameter", neck + 2.0, spacing - KNOB_GAP if q["count"] > 1 else 400.0)
    notes = [(n, asked[n], q[n]) for n in DEFAULTS if abs(q[n] - asked[n]) > 1e-9]
    return q, notes


def derive(p=None):
    """Clamped params plus every dimension the builders share."""
    q = clamp(p)[0]
    P = q["thread_pitch"]
    q["lead"] = P * q["thread_starts"]
    q["lead_angle"] = math.degrees(math.atan(q["lead"] / (math.pi * (q["thread_diameter"] - THREAD_DEPTH * P))))
    q["r_maj"] = q["thread_diameter"] / 2
    q["r_root"] = q["r_maj"] - THREAD_DEPTH * P
    q["r_journal"] = min(q["r_root"], q["r_maj"] - JOURNAL_CLEARANCE - MIN_SLEEVE)
    q["r_cup"] = q["r_journal"] + JOURNAL_CLEARANCE
    q["r_bore"] = q["r_maj"] + q["fit_clearance"]
    q["r_neck"] = q["r_maj"] + NECK_LIP
    t = q["wall_thickness"]
    q["journal"] = t + JOURNAL_EXTRA
    q["peg_half"] = q["r_journal"] * 4.0 / 9.0
    q["peg_length"] = q["journal"] - 1.0
    q["chan_half"] = q["nut_width"] / 2 + q["fit_clearance"]
    q["spacing"] = 2 * q["chan_half"] + t
    q["z_top"] = q["nut_height"] / 2
    q["z_floor"] = -q["nut_height"] / 2 - FLOOR_GAP
    q["z_bottom"] = q["z_floor"] - t
    q["x_in0"] = -GAP_HANDLE                      # inside faces of the end walls
    q["x_in1"] = q["screw_length"] + GAP_ENDSTOP
    q["x_out0"] = q["x_in0"] - t
    q["x_out1"] = q["x_in1"] + t
    q["body_length"] = q["x_out1"] - q["x_out0"]
    q["body_width"] = q["count"] * q["spacing"] + t
    q["body_height"] = q["z_top"] - q["z_bottom"]
    # nut centre range; the nut is modelled for a centre on a whole number of pitches
    q["x_min"] = q["x_in0"] + q["nut_length"] / 2
    q["x_max"] = q["x_in1"] - q["nut_length"] / 2
    q["travel"] = q["x_max"] - q["x_min"]
    q["turns"] = q["travel"] / q["lead"]
    q["x_ref"] = math.ceil(q["x_min"] / P) * P
    # collar and shaft
    q["r_shaft"] = q["shaft_diameter"] / 2
    collar_max = min(q["nut_length"], q["nut_width"]) / 2 - COLLAR_INSET
    q["r_collar"] = min(q["r_shaft"] + COLLAR_LIP, collar_max)
    q["z_shaft0"] = q["z_top"] + COLLAR_HEIGHT
    q["z_tip"] = q["z_shaft0"] + q["shaft_length"]
    q["r_hub_bore"] = q["r_shaft"] + q["shaft_clearance"]
    q["shaft_usable"] = q["shaft_length"]
    return q


def carrier_y(q, i):
    return (i - (q["count"] - 1) / 2) * q["spacing"]


def nut_x(q, t):
    """Nut centre for t in 0..1 along its travel."""
    return q["x_min"] + t * q["travel"]


# ── Mesh helpers ─────────────────────────────────────────────────────────────

def _ring(m, x, r, n, rot=0.0):
    """Circle about X, angle running from +Y to +Z."""
    out = []
    for j in range(n):
        a = rot + 2 * math.pi * j / n
        out.append(m.v(x, r * math.cos(a), r * math.sin(a)))
    return out


def _ring_z(m, z, r, n, x=0.0, y=0.0):
    """Circle about Z, angle running from +X to +Y."""
    out = []
    for j in range(n):
        a = 2 * math.pi * j / n
        out.append(m.v(x + r * math.cos(a), y + r * math.sin(a), z))
    return out


def _quad(m, f):
    """Add a quad, dropping corners that welded together."""
    out = []
    for i in f:
        if i not in out:
            out.append(i)
    if len(out) >= 3:
        m.faces.append(tuple(out))


def _band(m, a, b):
    """Ring a to ring b. Outward from the axis when a is below b on the axis."""
    n = len(a)
    for j in range(n):
        _quad(m, (a[j], a[(j + 1) % n], b[(j + 1) % n], b[j]))


def _loft(m, rings):
    for a, b in zip(rings, rings[1:]):
        _band(m, a, b)


def _ngon(m, loop, normal):
    """One flat n-gon facing `normal`, collinear corners allowed."""
    n = normal
    a = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = (a[1] * n[2] - a[2] * n[1], a[2] * n[0] - a[0] * n[2], a[0] * n[1] - a[1] * n[0])
    w = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0])
    pts = [(sum(m.verts[i][k] * u[k] for k in range(3)), sum(m.verts[i][k] * w[k] for k in range(3))) for i in loop]
    m.faces.append(tuple(loop) if _area(pts) > 0 else tuple(loop)[::-1])


def _square(m, x, h):
    """Square about X, same winding as _ring."""
    return [m.v(x, h, -h), m.v(x, h, h), m.v(x, -h, h), m.v(x, -h, -h)]


def _thread_r(q, x, frac, extra):
    """Thread radius at axial position x, where frac = angle / full turn."""
    P = q["thread_pitch"]
    u = ((x - q["lead"] * frac) / P) % 1.0
    lo, hi = q["r_root"] + extra, q["r_maj"] + extra
    a, b, c = ROOT_W, ROOT_W + FLANK_W, ROOT_W + FLANK_W + CREST_W
    if u < a:
        return lo
    if u < b:
        return lo + (hi - lo) * (u - a) / FLANK_W
    if u < c:
        return hi
    return hi - (hi - lo) * (u - c) / FLANK_W


def _helix_wall(m, q, x0, x1, extra, inward):
    """Thread surface with its rows along the helix, so every flank edge is exact. Rows past x0 or
    x1 collapse onto the end plane. Returns the end loops (at x0, at x1), one vertex per segment."""
    n, P, S = q["segments"], q["thread_pitch"], q["thread_starts"]
    L = q["lead"]
    knots = (0.0, ROOT_W, ROOT_W + FLANK_W, ROOT_W + FLANK_W + CREST_W)
    k0 = (math.floor(x0 / P) - S - 1) * 4
    k1 = (math.ceil(x1 / P) + 1) * 4
    cs = [(math.cos(2 * math.pi * j / n), math.sin(2 * math.pi * j / n)) for j in range(n)]

    def vert(x, j):
        r = _thread_r(q, x, j / n, extra)
        return m.v(x, r * cs[j][0], r * cs[j][1])

    def node(k, j):
        k += 4 * S * (j // n)   # one turn on = one lead = 4 knots per start, keeps the wrap welded
        j %= n
        x = P * (k // 4 + knots[k % 4]) + L * j / n
        return vert(min(max(x, x0), x1), j)

    for k in range(k0, k1):
        for j in range(n):
            f = (node(k, j), node(k, j + 1), node(k + 1, j + 1), node(k + 1, j))
            _quad(m, f[::-1] if inward else f)
    return [vert(x0, j) for j in range(n)], [vert(x1, j) for j in range(n)]


def _socket(m, q, x_mouth, sign):
    """Square socket going into the screw end at x_mouth; sign = +1 when it runs toward +X."""
    h, d = q["peg_half"], q["peg_length"]
    mouth = _square(m, x_mouth, h)
    floor = _square(m, x_mouth + sign * d, h)
    if sign > 0:
        _band(m, floor, mouth)
    else:
        _band(m, mouth, floor)
    _ngon(m, floor, (-sign, 0.0, 0.0))
    return mouth


def _peg(m, q, x_floor, sign):
    """Square peg standing on the cup floor at x_floor, pointing along sign. Returns its base loop."""
    h, d = q["peg_half"] - q["peg_clearance"], q["peg_length"]
    base = _square(m, x_floor, h)
    mid = _square(m, x_floor + sign, h)
    tip = _square(m, x_floor + sign * d, max(h - 0.2, 0.2))
    rows = [base, mid, tip] if sign > 0 else [tip, mid, base]
    _loft(m, rows)
    _ngon(m, tip, (sign, 0.0, 0.0))
    return base


# ── Parts ────────────────────────────────────────────────────────────────────

def build_screw(q):
    m = _Mesh()
    n, L, j = q["segments"], q["screw_length"], q["journal"]
    rj = q["r_journal"]
    lo, hi = _helix_wall(m, q, 0.0, L, 0.0, False)
    _loft(m, [_ring(m, -j, rj, n), _ring(m, 0.0, rj, n), lo])
    _loft(m, [hi, _ring(m, L, rj, n), _ring(m, L + j, rj, n)])
    m.face(_ring(m, -j, rj, n), [_socket(m, q, -j, 1)], (-1.0, 0.0, 0.0))
    m.face(_ring(m, L + j, rj, n), [_socket(m, q, L + j, -1)], (1.0, 0.0, 0.0))
    return m.verts, m.faces


def _knurl(m, x, r, flutes):
    """Zigzag loop: valley radius r, peak r + KNURL_DEPTH."""
    out = []
    for j in range(2 * flutes):
        a = math.pi * j / flutes
        rr = r + (KNURL_DEPTH if j % 2 else 0.0)
        out.append(m.v(x, rr * math.cos(a), rr * math.sin(a)))
    return out


def build_handle(q):
    m = _Mesh()
    n = q["segments"]
    x_tip, x_sh = q["x_in0"], q["x_out0"]
    x_floor = -q["journal"]
    x_neck = x_sh - NECK_LENGTH
    x_far = x_neck - KNOB_LENGTH
    R = q["knob_diameter"] / 2 - KNURL_DEPTH
    flutes = max(8, int(round(math.pi * q["knob_diameter"] / KNURL_PITCH)))
    far = _knurl(m, x_far, R - KNOB_CHAMFER, flutes)
    near = _knurl(m, x_neck, R - KNOB_CHAMFER, flutes)
    _loft(m, [far, _knurl(m, x_far + KNOB_CHAMFER, R, flutes), _knurl(m, x_neck - KNOB_CHAMFER, R, flutes), near])
    _ngon(m, far, (-1.0, 0.0, 0.0))
    neck = _ring(m, x_neck, q["r_neck"], n)
    m.face(near, [neck], (1.0, 0.0, 0.0))
    # neck, shoulder, sleeve, sleeve tip, cup wall
    _loft(m, [neck, _ring(m, x_sh, q["r_neck"], n), _ring(m, x_sh, q["r_maj"], n), _ring(m, x_tip, q["r_maj"], n),
              _ring(m, x_tip, q["r_cup"], n), _ring(m, x_floor, q["r_cup"], n)])
    base = _peg(m, q, x_floor, 1)
    m.face(_ring(m, x_floor, q["r_cup"], n), [base], (1.0, 0.0, 0.0))
    return m.verts, m.faces


def build_endstop(q):
    m = _Mesh()
    n = q["segments"]
    x_tip, x_sh = q["x_in1"], q["x_out1"]
    x_floor = q["screw_length"] + q["journal"] + ENDSTOP_FLOAT
    x_end = x_floor + ENDSTOP_CAP
    rc = q["knob_diameter"] / 2                # cap as wide as the knob: no tiny loose part
    ch = min(KNOB_CHAMFER, 0.3 * ENDSTOP_CAP)
    # cup wall, sleeve tip, sleeve, shoulder, cap (chamfered outer edge)
    _loft(m, [_ring(m, x_floor, q["r_cup"], n), _ring(m, x_tip, q["r_cup"], n), _ring(m, x_tip, q["r_maj"], n),
              _ring(m, x_sh, q["r_maj"], n), _ring(m, x_sh, rc, n), _ring(m, x_end - ch, rc, n),
              _ring(m, x_end, rc - ch, n)])
    _ngon(m, _ring(m, x_end, rc - ch, n), (1.0, 0.0, 0.0))
    base = _peg(m, q, x_floor, -1)
    m.face(_ring(m, x_floor, q["r_cup"], n), [base], (-1.0, 0.0, 0.0))
    return m.verts, m.faces


def _fillet(z0, r0, z1, r1, steps=4):
    """Quarter-round rows from (z0, r0) to (z1, r1): starts tangent to the wall, ends flat."""
    rows = []
    for i in range(steps + 1):
        a = 0.5 * math.pi * i / steps
        rows.append((z0 + (z1 - z0) * math.sin(a), r1 + (r0 - r1) * math.cos(a)))
    return rows


def build_nut(q):
    """Local frame: nut centred on the origin, meshing when its centre is on a whole pitch."""
    m = _Mesh()
    n = q["segments"]
    hl, hw, hh = q["nut_length"] / 2, q["nut_width"] / 2, q["nut_height"] / 2
    lo, hi = _helix_wall(m, q, -hl, hl, q["thread_clearance"], True)
    corners = lambda x: [m.v(x, hw, -hh), m.v(x, hw, hh), m.v(x, -hw, hh), m.v(x, -hw, -hh)]
    a, b = corners(-hl), corners(hl)
    for j in (0, 2, 3):     # the top side comes last, with the collar's hole in it
        m.faces.append((a[j], a[(j + 1) % 4], b[(j + 1) % 4], b[j]))
    m.face(a, [lo], (-1.0, 0.0, 0.0))
    m.face(b, [hi], (1.0, 0.0, 0.0))
    # collar and shaft stand on the top face
    rc, rs = q["r_collar"], q["r_shaft"]
    z0, z1, z_tip = q["z_top"], q["z_shaft0"], q["z_tip"]
    fc = min(COLLAR_FILLET, 0.5 * (rc - rs))
    ft = min(1.0, 0.5 * rs)
    rows = [(z0, rc)] + _fillet(z1 - fc, rc, z1, rc - fc)
    rows += [(z1, rs)] + _fillet(z_tip - ft, rs, z_tip, rs - ft)
    rings = [_ring_z(m, z, r, n) for z, r in rows]
    _loft(m, rings)
    _ngon(m, rings[-1], (0.0, 0.0, 1.0))
    top = [m.v(-hl, -hw, hh), m.v(hl, -hw, hh), m.v(hl, hw, hh), m.v(-hl, hw, hh)]
    m.face(top, [rings[0]], (0.0, 0.0, 1.0))
    return m.verts, m.faces


def build_body(q):
    """All the outer and channel faces stay one planar piece each, so the block keeps quads and
    n-gons; only the faces with a bore are triangulated."""
    m = _Mesh()
    n, c = q["segments"], q["count"]
    xa, xb, xc, xd = q["x_out0"], q["x_in0"], q["x_in1"], q["x_out1"]
    zb, zf, zt = q["z_bottom"], q["z_floor"], q["z_top"]
    ch = q["chan_half"]
    t = q["wall_thickness"]
    # y lines, low to high: wall edge, channel edge, channel edge, wall edge, ...
    ys = [carrier_y(q, 0) - ch - t]
    for i in range(c):
        y = carrier_y(q, i)
        ys += [y - ch, y + ch]
    ys.append(ys[-1] + t)

    # top deck: full-length strips over the walls, end pieces over the channels
    for w in range(len(ys) - 1):
        y0, y1 = ys[w], ys[w + 1]
        if w % 2 == 0:
            m.faces.append((m.v(xa, y0, zt), m.v(xb, y0, zt), m.v(xc, y0, zt), m.v(xd, y0, zt),
                            m.v(xd, y1, zt), m.v(xc, y1, zt), m.v(xb, y1, zt), m.v(xa, y1, zt)))
        else:
            for x0, x1 in ((xa, xb), (xc, xd)):
                m.faces.append((m.v(x0, y0, zt), m.v(x1, y0, zt), m.v(x1, y1, zt), m.v(x0, y1, zt)))

    # underside: an end piece at each end wall, strips between the floor slots
    hs = q["floor_slot"] / 2
    edges = [ys[0]]
    for i in range(c if hs > 0 else 0):
        y = carrier_y(q, i)
        edges += [y - hs, y + hs]
    edges.append(ys[-1])
    for xo, xi in ((xa, xb), (xd, xc)):
        _ngon(m, [m.v(xo, y, zb) for y in ys] + [m.v(xi, y, zb) for y in reversed(edges)], (0.0, 0.0, -1.0))
    for w in range(0, len(edges), 2):
        y0, y1 = edges[w], edges[w + 1]
        _ngon(m, [m.v(xb, y0, zb), m.v(xc, y0, zb), m.v(xc, y1, zb), m.v(xb, y1, zb)], (0.0, 0.0, -1.0))
    # the two long sides
    for y, s in ((ys[0], -1.0), (ys[-1], 1.0)):
        _ngon(m, [m.v(xa, y, zb), m.v(xb, y, zb), m.v(xc, y, zb), m.v(xd, y, zb),
                  m.v(xd, y, zt), m.v(xc, y, zt), m.v(xb, y, zt), m.v(xa, y, zt)], (0.0, s, 0.0))

    # end walls: a plain piece behind each wall strip, a bored piece behind each channel
    for x, s in ((xa, -1.0), (xd, 1.0)):
        for w in range(len(ys) - 1):
            y0, y1 = ys[w], ys[w + 1]
            rect = [m.v(x, y0, zb), m.v(x, y1, zb), m.v(x, y1, zt), m.v(x, y0, zt)]
            if w % 2 == 0:
                _ngon(m, rect, (s, 0.0, 0.0))
            else:
                cy = carrier_y(q, w // 2)
                m.face(rect, [_bore_ring(m, q, x, cy)], (s, 0.0, 0.0))

    for i in range(c):
        y = carrier_y(q, i)
        y0, y1 = y - ch, y + ch
        # floor line of the channel's end faces, broken at the slot
        fl = [y0, y1]
        if hs > 0:
            s0, s1 = y - hs, y + hs
            fl = [y0, s0, s1, y1]
            for a, b in ((y0, s0), (s1, y1)):
                _ngon(m, [m.v(xb, a, zf), m.v(xc, a, zf), m.v(xc, b, zf), m.v(xb, b, zf)], (0.0, 0.0, 1.0))
            # slot walls, facing into the slot
            _ngon(m, [m.v(xb, s0, zb), m.v(xc, s0, zb), m.v(xc, s0, zf), m.v(xb, s0, zf)], (0.0, 1.0, 0.0))
            _ngon(m, [m.v(xb, s1, zb), m.v(xc, s1, zb), m.v(xc, s1, zf), m.v(xb, s1, zf)], (0.0, -1.0, 0.0))
            _ngon(m, [m.v(xb, s0, zb), m.v(xb, s1, zb), m.v(xb, s1, zf), m.v(xb, s0, zf)], (1.0, 0.0, 0.0))
            _ngon(m, [m.v(xc, s0, zb), m.v(xc, s1, zb), m.v(xc, s1, zf), m.v(xc, s0, zf)], (-1.0, 0.0, 0.0))
        else:
            _ngon(m, [m.v(xb, y0, zf), m.v(xc, y0, zf), m.v(xc, y1, zf), m.v(xb, y1, zf)], (0.0, 0.0, 1.0))
        _ngon(m, [m.v(xb, y0, zf), m.v(xc, y0, zf), m.v(xc, y0, zt), m.v(xb, y0, zt)], (0.0, 1.0, 0.0))
        _ngon(m, [m.v(xb, y1, zf), m.v(xc, y1, zf), m.v(xc, y1, zt), m.v(xb, y1, zt)], (0.0, -1.0, 0.0))
        # the channel's end faces and the bore through the wall
        ra, rb = _bore_ring(m, q, xa, y), _bore_ring(m, q, xb, y)
        rc_, rd = _bore_ring(m, q, xc, y), _bore_ring(m, q, xd, y)
        _band(m, rb, ra)
        _band(m, rd, rc_)
        m.face([m.v(xb, a, zf) for a in fl] + [m.v(xb, y1, zt), m.v(xb, y0, zt)], [rb], (1.0, 0.0, 0.0))
        m.face([m.v(xc, a, zf) for a in fl] + [m.v(xc, y1, zt), m.v(xc, y0, zt)], [rc_], (-1.0, 0.0, 0.0))
    return m.verts, m.faces


def _bore_ring(m, q, x, y):
    n = q["segments"]
    r = q["r_bore"]
    return [m.v(x, y + r * math.cos(2 * math.pi * j / n), r * math.sin(2 * math.pi * j / n)) for j in range(n)]
