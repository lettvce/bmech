"""
rail_mechs.py - mechanisms that drop over the shaft rail's nut shafts (pure Python, no Blender).

Every mechanism is one closed solid fused to a capped hub: the hub's bore runs over the shaft with
`shaft_clearance`, its roof rests on the shaft tip, and its bottom floats HUB_LIFT above the collar.
Gravity holds it on (the shafts stand vertical); there is no separate stop cap to lose.

    spur           involute spur gear (bmech profile, geneva.gear_outline)
    spur (full)    full_height=True: the teeth run the hub's whole height with a flat top, like the Geneva wheel,
                   and the gear is never lifted, so it meets either gear of a cluster where that gear already is
                   (2026-11-12, for meshing the top of a top-heavy cluster)
    cluster        two spur gears fused on one hub, a bottom and a top gear (2026-11-11); the top gear runs up
                   to the hub's top, so the top is one flat face: print the bigger gear down, either way up
    geneva_driver  Geneva crank: plate (plain disc or spur teeth), locking disc and pin
    geneva_wheel   the slotted wheel, raised on its hub to sit over its driver's plate
    yoke           Scotch yoke (Blake's model, 2026-11-11): a holder over the nut's shaft (legs straddle the nut
                   so it can't spin, a dovetail channel along X on top) and a separate yoke that slides in it.
                   The yoke's slot runs across the rail and takes the pin of a pin gear on the other nut of
                   the same row (two-sided rails only), so turning the gear slides the yoke back and forth.
                   A spur gear with pin=True is flat-topped (it runs to the hub top) with the pin standing up
                   from that face through the slot.

Frames: a mechanism is built in its nut's local frame (shaft axis on x = y = 0, z as in
shaft_rail.build_nut), so it is placed at the same (x, y) as its nut.

Layout: rails can link to another rail so their mechanisms mesh. A linked rail's nut sits at
    x = x_parent + side * sqrt(d^2 - dy^2),    dy = rail gap x carrier spacing
where d is the centre distance (spur pair: m (z1 + z2) / 2 + backlash; Geneva: C). layout() returns the
rest positions, an angle program per rail (for animation), collision primitives and notes.

Gear layers: a spur has one gear, a cluster two (bottom, top), a toothed Geneva driver one (its plate).
A link names which of its own layers meshes ("layer") with which of the parent's ("their"; default the
parent's top gear). A spur or cluster is lifted on its hub so the two layers line up:
    lift = parent's lift + parent layer's z - own layer's z
Geneva parts can't be lifted.

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
CLUSTER_MIN_TOP = 2.0   # a cluster's top gear, at least this thick (it runs from the step to the hub top)

GEAR_DEFAULTS = dict(module=2.4, pressure_angle=20.0, backlash=0.25)    # chunky teeth, still many ratios
MECH_DEFAULTS = {
    "none": {},
    "spur": dict(teeth=16, thickness=5.0, full_height=False, pin=False, pin_radius=8.0, pin_diameter=5.0),
    "cluster": dict(teeth=30, thickness=5.0, teeth_top=14),       # the top gear fills up to the hub top
    "geneva_driver": dict(slots=6, center_distance=40.0, pin_diameter=5.0, clearance=0.2,
                          plate_thickness=3.0, layer_gap=0.4, wheel_thickness=5.0,
                          toothed=False, gear_teeth=24, tip_round=1.0),
    "geneva_wheel": {},     # takes every dimension from the driver it links to
    # length: pin gear centre to holder centre; tail: bar past the holder at the far end of the stroke
    "yoke": dict(length=60.0, tail=5.0, thickness=4.0, fit=0.3, slot_clearance=0.2),
}

# Scotch yoke (Blake's w_yoke.blend, 2026-11-11)
YOKE_GAP = 0.4          # hub tops to the yoke's underside (= the holder's channel floor)
LEG_GAP = 0.2           # nut end face to the holder's leg
LEG_T = 3.2             # leg thickness along X
LEG_DROP = 2.0          # legs reach this far below the body top, beside the nut
HOLDER_SIDE = 1.6       # holder wall beside the dovetail channel
DOVETAIL_TOP = 3.4      # yoke bar width at its top; the bar widens downward
DOVETAIL_SLOPE = 0.5    # per side, mm out per mm down (Blake: 3.4 top, 7.4 bottom over 4 mm)
FRAME_WALL = 3.2        # yoke frame around the slot
PIN_ROOT_GAP = 1.0      # pin edge inside the pin gear's root circle
PIN_FLAT_MIN = 2.0      # a pin gear raised on its hub keeps at least this much gear


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
    sq = dict(kind="spur", teeth=z, thickness=t, rp=rp, ra=rp + geneva.ADDENDUM * m,
              rd=rp - geneva.DEDENDUM * m, pin=bool(spec.get("pin", False)),
              full=bool(spec.get("full_height", False)))
    if sq["full"]:
        # full height (2026-11-12): teeth from the hub bottom to the hub top, flat top like the Geneva wheel;
        # never lifted, so it meets any layer of its partner where that layer already is
        sq["thickness"] = h["z_top"] - h["zb"]
        notes = [n for n in notes if n[0] != "thickness"]
    if sq["pin"]:
        # flat-topped: the gear runs to the hub top (layout() redoes this for a raised gear), the pin
        # stands on that face inside the root circle; its top is set by the yoke it drives
        sq["thickness"] = h["z_top"] - h["zb"]
        notes = [n for n in notes if n[0] != "thickness"]
        pd = float(spec.get("pin_diameter", 5.0))
        pd_c = min(max(pd, 2.0), 2 * (sq["rd"] - PIN_ROOT_GAP))
        if abs(pd_c - pd) > 1e-9:
            notes.append(("pin_diameter", pd, pd_c))
        pr = float(spec.get("pin_radius", 8.0))
        pr_c = min(max(pr, 1.0), sq["rd"] - PIN_ROOT_GAP - pd_c / 2)
        if abs(pr_c - pr) > 1e-9:
            notes.append(("pin_radius", pr, pr_c))
        sq.update(pin_diameter=pd_c, pin_radius=pr_c,
                  pin_top=h["z_top"] + YOKE_GAP + MECH_DEFAULTS["yoke"]["thickness"])
    return sq, notes


def cluster_q(q, g, spec):
    """Clamped cluster: two gears on one hub, each clamped like a spur. The smaller one's tips stay
    0.4 mm inside the larger one's root circle, so the step between them is a flat ring. The top gear
    runs from the step up to the hub's top (flat top, like the Geneva wheel), so only the bottom
    gear's thickness is asked for; thickness_top here is for an unlifted cluster (layout() redoes it)."""
    h = hub(q)
    m = g["module"]
    full = h["z_top"] - h["zb"]
    lo, nn = spur_q(q, g, dict(teeth=spec.get("teeth", 30), thickness=0.6))
    hi, nt = spur_q(q, g, dict(teeth=spec.get("teeth_top", 14), thickness=0.6))
    notes = [n for n in nn if n[0] == "teeth"] + [(n + "_top", a, b) for n, a, b in nt if n == "teeth"]
    t = float(spec.get("thickness", 5.0))
    tmax = full - CLUSTER_MIN_TOP
    if t < 0.6 or t > tmax:
        notes.append(("thickness", t, min(max(t, 0.6), tmax)))
        t = min(max(t, 0.6), tmax)
    lo["thickness"], hi["thickness"] = t, full - t
    zmin = max(geneva.MIN_TEETH, int(math.ceil(2 * (h["rh"] + 1.0) / m + 2 * geneva.DEDENDUM - 1e-9)))
    dz = int(math.ceil(2 * (geneva.ADDENDUM + geneva.DEDENDUM) + 0.8 / m - 1e-9))
    z1, z2 = lo["teeth"], hi["teeth"]
    if abs(z1 - z2) < dz:
        # move the top gear the short way past the bottom one, or the long way if that's under the minimum
        down, up = z1 - dz, z1 + dz
        z2n = up if (z2 >= z1 or down < zmin) and up <= 400 else down
        notes.append(("teeth_top", z2, z2n))
        hi, _n = spur_q(q, g, dict(teeth=z2n, thickness=hi["thickness"]))
    return dict(kind="cluster", teeth=lo["teeth"], thickness=lo["thickness"], rp=lo["rp"], ra=lo["ra"], rd=lo["rd"],
                teeth_top=hi["teeth"], thickness_top=hi["thickness"], rp_top=hi["rp"], ra_top=hi["ra"],
                rd_top=hi["rd"]), notes


def gear_layers(it):
    """[(teeth, z0, z1, tip radius)] of a mechanism's gear layers, z from its own base (before lift)."""
    k, mq = it["kind"], it.get("mq") or {}
    if k == "spur":
        return [(mq["teeth"], 0.0, mq["thickness"], mq["ra"])]
    if k == "cluster":
        t = mq["thickness"]
        return [(mq["teeth"], 0.0, t, mq["ra"]), (mq["teeth_top"], t, t + mq["thickness_top"], mq["ra_top"])]
    if k == "geneva_driver" and mq.get("driver_gear"):
        return [(mq["gear_teeth"], 0.0, mq["plate_thickness"], mq["plate_r"])]
    return []


LIFTABLE = ("spur", "cluster")


def is_full(it):
    """A full-height spur: its one gear layer runs the hub's whole height."""
    return it["kind"] == "spur" and bool((it.get("mq") or {}).get("full"))


def build_spur(q, g, sq):
    if sq.get("full") and not sq.get("pin"):
        return build_gear_stack(q, g, [(sq["teeth"], sq["thickness"])], 0.0, flat_top=True)
    if sq.get("pin"):
        return build_gear_stack(q, g, [(sq["teeth"], sq["thickness"])], sq.get("lift", 0.0), flat_top=True,
                                pin=(sq["pin_radius"], sq["pin_diameter"], sq["pin_top"]))
    return build_gear_stack(q, g, [(sq["teeth"], sq["thickness"])], sq.get("lift", 0.0))


def build_cluster(q, g, cq):
    return build_gear_stack(q, g, [(cq["teeth"], cq["thickness"]), (cq["teeth_top"], cq["thickness_top"])],
                            cq.get("lift", 0.0), flat_top=True)


def build_gear_stack(q, g, stack, lift=0.0, flat_top=False, pin=None):
    """Gears [(teeth, thickness)] stacked from the hub bottom + lift, fused to the hub. Below a lifted
    stack the hub runs down to its bottom; between two gears the step is a flat ring: the larger gear's
    ring in to a circle, then that circle to the smaller outline. flat_top: the last gear ends at the
    hub's top (its thickness has to say so) and covers it, one flat face. pin = (radius, diameter, top z):
    a round pin on +X standing on that flat top (flat_top only)."""
    h, segs = hub(q), q["segments"]
    m, pa = g["module"], g["pressure_angle"]
    B = _Build()
    outs = []
    for N, t in stack:
        out = geneva.gear_outline(m, N, pa, segs)
        outs.append((out, len(out) // N, N, _kb(N, segs), t))
    z = h["zb"] + lift
    out, U, N, kb, t = outs[0]
    ob, ib = B.ring(out, U, N, h["rh"], kb, z, False)
    bore0 = B.bore(h, segs, h["zb"])
    if lift > 1e-9:
        lo = B.loop(_circle(h["rh"], N * kb, -math.pi / N), h["zb"])
        B.wall(lo, ib)
        B.face(lo, [bore0], False)
    else:
        B.face(ib, [bore0], False)
    for k, (out, U, N, kb, t) in enumerate(outs):
        z1 = z + t
        if k == len(outs) - 1:
            if flat_top:
                z1 = h["z_top"]
            if pin:
                # the ring stops at a circle just outside the pin; the disc inside it has the pin's foot as a hole
                pr, pd, pz = pin
                ri = max(h["rh"], pr + pd / 2 + 0.5)
                ot, it_ = B.ring(out, U, N, ri, kb, z1, True)
                B.wall(ob, ot)
                pc = [(pr + x, y) for x, y in _circle(pd / 2, max(16, segs // 2))]
                pb, pt = B.loop(pc, z1), B.loop(pc, pz)
                B.wall(pb, pt)
                B.ngon(pt, True)
                B.face(it_, [pb], True)
                break
            ot, it_ = B.ring(out, U, N, h["rh"], kb, z1, True)
            B.wall(ob, ot)
            if flat_top:
                B.ngon(it_, True)
                break
            top = B.loop(_circle(h["rh"], N * kb, -math.pi / N), h["z_top"])
            B.wall(it_, top)
            B.ngon(top, True)
            break
        nout, nU, nN, nkb, _t = outs[k + 1]
        rs = m * min(N, nN) / 2 + geneva.ADDENDUM * m           # smaller gear's tips
        rl = m * max(N, nN) / 2 - geneva.DEDENDUM * m           # larger gear's roots
        r_mid = 0.5 * (rs + rl)
        if N > nN:          # step faces up: this gear's ring in to r_mid, then r_mid to the next outline
            ot, mid = B.ring(out, U, N, r_mid, kb, z1, True)
            nb = B.loop(nout, z1)
            B.face(mid, [nb], True)
        else:               # step faces down under the larger next gear
            ot = B.loop(out, z1)
            nb, mid = B.ring(nout, nU, nN, r_mid, nkb, z1, False)
            B.face(mid, [ot], False)
        B.wall(ob, ot)
        ob = nb
        z = z1
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


def yoke_q(q, spec):
    """Clamped yoke parameters and the holder's dimensions (nut frame). The length is clamped again by
    layout() once the pin gear is known (yoke_min_length)."""
    h = hub(q)
    notes = []
    p = {}
    for name, lo, hi in (("thickness", 2.0, 12.0), ("fit", 0.05, 0.8), ("slot_clearance", 0.05, 0.8),
                         ("tail", 0.0, 200.0), ("length", 1.0, 1000.0)):
        v = float(spec.get(name, MECH_DEFAULTS["yoke"][name]))
        c = min(max(v, lo), hi)
        if abs(c - v) > 1e-9:
            notes.append((name, v, c))
        p[name] = c
    T = p["thickness"]
    tw = DOVETAIL_TOP / 2
    bw = tw + DOVETAIL_SLOPE * T
    a = max(q["nut_length"] / 2 + LEG_GAP, h["rb"] + 0.8)        # legs' inner faces, either side of the nut
    X = a + LEG_T
    W = max(h["rh"], bw + p["fit"] + HOLDER_SIDE)
    zc = h["z_top"] + YOKE_GAP
    p.update(kind="yoke", tw=tw, bw=bw, a=a, X=X, W=W, z_leg=q["z_top"] - LEG_DROP, z_ch=zc, z_hi=zc + T,
             ch_bot=bw + p["fit"], ch_top=tw + p["fit"],
             # set by layout() from the pin gear
             pin_radius=0.0, pin_diameter=5.0, dir=1.0)
    return p, notes


def yoke_slot(yq):
    """(slot half width along X, slot half length along Y)."""
    sx = yq["pin_diameter"] / 2 + yq["slot_clearance"]
    return sx, yq["pin_radius"] + sx


def yoke_frame(yq):
    """(sx, sy, fx, fy): slot and frame half sizes. The frame stays at least 1 mm wider than the bar."""
    sx, sy = yoke_slot(yq)
    return sx, sy, sx + FRAME_WALL, max(sy + FRAME_WALL, yq["bw"] + 1.0)


def yoke_min_length(yq, gear_tip):
    """Shortest pin-gear-to-holder distance: the yoke's frame clears the holder at the near end of the
    stroke, and the pin gear's tips clear the holder."""
    sx, _sy = yoke_slot(yq)
    return max(sx + FRAME_WALL + yq["pin_radius"] + yq["X"] + COLLIDE_GAP,
               gear_tip + math.hypot(yq["X"], yq["W"]) + COLLIDE_GAP)     # as collisions() sees the holder


def build_yoke_holder(q, g, yq):
    """Box over the shaft from the hub bottom to the yoke's top, legs down past the nut's ends, a dovetail
    channel along X across the top. Lower part: z-extruded, with the hub bore and its roof; upper part:
    the two channel cheeks, X-extruded."""
    h, segs = hub(q), q["segments"]
    m = _Mesh()
    a, X, W = yq["a"], yq["X"], yq["W"]
    zl, zb, zc, zt = yq["z_leg"], h["zb"], yq["z_ch"], yq["z_hi"]
    cb, ct = yq["ch_bot"], yq["ch_top"]
    V = lambda x, y, z: m.v(x, y, z)       # noqa: E731
    # lower walls, zb..zc: the +-Y sides carry the legs' corners below and one edge above (the cheeks)
    for s in (-1.0, 1.0):
        pts = [(-X, s * W), (-a, s * W), (a, s * W), (X, s * W)]
        if s > 0:
            pts = pts[::-1]
        m.faces.append(tuple(V(x, y, zb) for x, y in pts) + (V(pts[-1][0], pts[-1][1], zc), V(pts[0][0], pts[0][1], zc)))
        ys = [-W, -cb, cb, W] if s > 0 else [W, cb, -cb, -W]
        for j in range(3):
            x = s * X
            m.faces.append((V(x, ys[j], zb), V(x, ys[j + 1], zb), V(x, ys[j + 1], zc), V(x, ys[j], zc)))
    # underside between the legs, around the bore
    bc = _circle(h["rb"], segs)[::-1]
    blo = [m.v(x, y, zb) for x, y in bc]
    bhi = [m.v(x, y, h["z_tip"]) for x, y in bc]
    for j in range(segs):
        m.faces.append((blo[j], blo[(j + 1) % segs], bhi[(j + 1) % segs], bhi[j]))
    m.faces.append(tuple(bhi))                                  # bore roof, facing down
    under = [m.v(-a, -W, zb), m.v(a, -W, zb), m.v(a, W, zb), m.v(-a, W, zb)]
    m.face(under, [blo], (0.0, 0.0, -1.0))
    # legs
    for lp in ([(-X, -W), (-a, -W), (-a, W), (-X, W), (-X, cb), (-X, -cb)],
               [(a, -W), (X, -W), (X, -cb), (X, cb), (X, W), (a, W)]):          # CCW, outer face split at +-cb
        top = [m.v(x, y, zb) for x, y in lp]
        bot = [m.v(x, y, zl) for x, y in lp]
        for j in range(6):
            m.faces.append((bot[j], bot[(j + 1) % 6], top[(j + 1) % 6], top[j]))
        m.faces.append(tuple(bot[::-1]))
    # channel floor
    m.faces.append((m.v(-X, -cb, zc), m.v(X, -cb, zc), m.v(X, cb, zc), m.v(-X, cb, zc)))
    # cheeks: profile in (y, z), extruded from -X to +X
    for s in (-1.0, 1.0):
        prof = [(-W, zc), (-cb, zc), (-ct, zt), (-W, zt)]          # CCW in (y, z) for s = -1
        prof = [(s * -y if s > 0 else y, z) for y, z in prof]
        if s > 0:
            prof = prof[::-1]
        L = [m.v(-X, y, z) for y, z in prof]
        R = [m.v(X, y, z) for y, z in prof]
        for j in range(4):
            if prof[j][1] == zc and prof[(j + 1) % 4][1] == zc:
                continue                                        # sits on the lower part's top
            m.faces.append((L[(j + 1) % 4], R[(j + 1) % 4], R[j], L[j]))
        m.faces.append(tuple(L[::-1]))
        m.faces.append(tuple(R))
    return m.verts, m.faces


def build_yoke(q, g, yq):
    """The sliding yoke in its own frame: slot centred on the origin (across the rail, along Y), the bar
    running toward -X through the holder; z as in the nut frame. Placed by the page at the pin gear's
    x + pin_radius cos(angle), turned 180 degrees when dir < 0."""
    m = _Mesh()
    sx, sy, fx, fy = yoke_frame(yq)
    tw, bw = yq["tw"], yq["bw"]
    z0, z1 = yq["z_ch"], yq["z_hi"]
    u_end = -(yq["length"] + yq["X"] + yq["pin_radius"] + yq["tail"])
    # frame: outer loop with the bar's corners on its -X side, slot as the hole
    def outer(z, w):
        return [m.v(-fx, -fy, z), m.v(fx, -fy, z), m.v(fx, fy, z), m.v(-fx, fy, z), m.v(-fx, w, z), m.v(-fx, -w, z)]
    ob, ot = outer(z0, bw), outer(z1, tw)
    slot = lambda z: [m.v(-sx, -sy, z), m.v(sx, -sy, z), m.v(sx, sy, z), m.v(-sx, sy, z)]
    sb, st = slot(z0), slot(z1)
    m.face(ob, [sb], (0.0, 0.0, -1.0))
    m.face(ot, [st], (0.0, 0.0, 1.0))
    for j in range(4):
        m.faces.append((sb[(j + 1) % 4], sb[j], st[j], st[(j + 1) % 4]))          # slot walls face in
    for j in range(3):
        m.faces.append((ob[j], ob[j + 1], ot[j + 1], ot[j]))                       # -Y, +X, +Y walls
    # -X wall: two pieces either side of the bar
    m.faces.append((ob[3], ob[4], ot[4], ot[3]))
    m.faces.append((ob[5], ob[0], ot[0], ot[5]))
    # bar: from the frame's -X face out to u_end
    e = [m.v(u_end, -bw, z0), m.v(u_end, bw, z0), m.v(u_end, tw, z1), m.v(u_end, -tw, z1)]
    f = [ob[5], ob[4], ot[4], ot[5]]           # (-fx, -bw, z0), (-fx, bw, z0), (-fx, tw, z1), (-fx, -tw, z1)
    for j in range(4):
        m.faces.append((f[(j + 1) % 4], f[j], e[j], e[(j + 1) % 4]))
    m.faces.append(tuple(e[::-1]))
    return m.verts, m.faces


# ── Rails (nuts) ─────────────────────────────────────────────────────────────
# One-sided: rail i is carrier row i. Two-sided: rails come in pairs, 2k = row k's left nut (its knob at
# -X), 2k + 1 = row k's right nut on the turned rail (its knob at +X). pos 0 is always at the nut's own
# knob end.

def rail_count(q):
    return q["count"] * (2 if q["two_sided"] else 1)


def row_side(q, i):
    return (i // 2, i % 2) if q["two_sided"] else (i, 0)


def rail_label(q, i):
    row, side = row_side(q, i)
    return "%d%s" % (row + 1, "LR"[side]) if q["two_sided"] else "%d" % (i + 1)


def rail_y(q, i):
    return sr.carrier_y(q, row_side(q, i)[0])


def rail_x(q, i, t):
    x = sr.nut_x(q, t)
    return 2 * q["x_mid"] - x if row_side(q, i)[1] else x


def x_range(q, i):
    if row_side(q, i)[1]:
        return 2 * q["x_mid"] - q["x_max"], 2 * q["x_mid"] - q["x_min"]
    return q["x_min"], q["x_max"]


# ── Layout ───────────────────────────────────────────────────────────────────

def _mech_spec(r):
    kind = r.get("mech", "none") or "none"
    if kind not in MECH_DEFAULTS:         # unknown: old saved page state (the retired yoke holder, 2026-11-06)
        kind = "none"
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
    rs = list(spec.get("rails", []))[:rail_count(q)]
    while len(rs) < rail_count(q):
        rs.append(dict(mech="none"))
    notes = [("rail", n, a, u) for n, a, u in rail_notes]
    issues = []
    h = hub(q)
    # shared pressure angle: low enough for the fewest teeth in use
    teeth = [int(r.get("teeth", MECH_DEFAULTS["spur"]["teeth"])) for r in rs if r.get("mech") in ("spur", "cluster")]
    teeth += [int(r.get("teeth_top", MECH_DEFAULTS["cluster"]["teeth_top"])) for r in rs if r.get("mech") == "cluster"]
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

    lab = lambda i: rail_label(q, i)       # noqa: E731
    out = []
    for i, r in enumerate(rs):
        kind, ms = _mech_spec(r)
        asked = r.get("mech", "none") or "none"
        if kind != asked:
            issues.append("Rail %s: %r isn't available any more, so it's left empty." % (lab(i), asked))
        row, side = row_side(q, i)
        item = dict(index=i, kind=kind, y=rail_y(q, i), row=row, side=side, label=lab(i), link=r.get("link"),
                    pos=float(r.get("pos", 0.5)))
        if kind == "spur":
            item["mq"], nn = spur_q(q, g, ms)
            notes += [("rail %s" % lab(i), a, b, c) for a, b, c in nn]
        elif kind == "cluster":
            item["mq"], nn = cluster_q(q, g, ms)
            notes += [("rail %s" % lab(i), a, b, c) for a, b, c in nn]
        elif kind == "geneva_driver":
            item["mq"], nn = geneva_q(q, g, ms)
            notes += [("rail %s" % lab(i), a, b, c) for a, b, c in nn]
        elif kind == "yoke":
            item["mq"], nn = yoke_q(q, ms)
            notes += [("rail %s" % lab(i), a, b, c) for a, b, c in nn]
        out.append(item)
    # wheels take their driver's dimensions
    for it in out:
        if it["kind"] == "geneva_wheel":
            L = it["link"]
            drv = out[L["rail"]] if L and 0 <= L["rail"] < len(out) else None
            if drv is None or drv["kind"] != "geneva_driver":
                issues.append("Rail %s: a Geneva wheel has to link to a rail with a Geneva driver." % lab(it["index"]))
                it["kind"] = "none"
            else:
                it["mq"] = drv["mq"]

    # placement: parents first; a link cycle or a bad link leaves the rail on its own slider
    def mesh_layers(it, par, L):
        """(own layer, parent's layer) indices for a link, clamped to what each has."""
        la, lb = gear_layers(it), gear_layers(par)
        mine = int(L.get("layer", 0) or 0)
        their = L.get("their")
        their = (len(lb) - 1) if their is None else int(their)       # default: the parent's top gear
        return min(max(mine, 0), max(len(la) - 1, 0)), min(max(their, 0), max(len(lb) - 1, 0))

    def centre_distance(a, b, L):
        ka, kb = a["kind"], b["kind"]
        pair = {ka, kb}
        if pair == {"geneva_driver", "geneva_wheel"}:
            return a["mq"]["center_distance"] if ka == "geneva_driver" else b["mq"]["center_distance"]
        la, lb = gear_layers(a), gear_layers(b)
        if not la or not lb or "geneva_wheel" in pair or pair == {"geneva_driver"}:
            return None
        mi, ti = mesh_layers(a, b, L)
        return g["module"] * (la[mi][0] + lb[ti][0]) / 2 + g["backlash"]

    placed = {}
    room = h["room"] - 0.5

    def place(i, seen=()):
        if i in placed:
            return placed[i]
        it = out[i]
        it["lift"] = 0.0
        x = rail_x(q, i, min(max(it["pos"], 0.0), 1.0))
        L = it["link"]
        it["parent"] = None
        if L and L.get("rail") is not None and 0 <= L["rail"] < len(out) and L["rail"] != i:
            j = L["rail"]
            if j in seen:
                issues.append("Rails %s and %s link to each other in a loop." % (lab(i), lab(j)))
            elif it["kind"] == "none" or out[j]["kind"] == "none":
                issues.append("Rail %s links to rail %s, but one of them has no mechanism." % (lab(i), lab(j)))
            elif it["kind"] == "yoke" or out[j]["kind"] == "yoke":
                x = place_yoke(i, j, seen, x)
            else:
                d = centre_distance(it, out[j], L)
                if d is None:
                    issues.append("Rail %s (%s) can't mesh with rail %s (%s)."
                                  % (lab(i), LABELS[it["kind"]], lab(j), LABELS[out[j]["kind"]]))
                else:
                    dy = abs(it["y"] - out[j]["y"])
                    xp = place(j, seen + (i,))
                    if d < dy - 1e-9:
                        issues.append("Rail %s can't reach rail %s: the centre distance is %.1f mm but the rails "
                                      "are %.1f mm apart." % (lab(i), lab(j), d, dy))
                    else:
                        side = 1.0 if L.get("side", 1) >= 0 else -1.0
                        if it["side"]:          # a right nut's endstop is toward -X
                            side = -side
                        it["offset"] = side * math.sqrt(max(0.0, d * d - dy * dy))
                        it["parent"] = j
                        it["distance"] = d
                        x = xp + it["offset"]
                        la, lb = gear_layers(it), gear_layers(out[j])
                        if la and lb:
                            mi, ti = mesh_layers(it, out[j], L)
                            it["mesh_layers"] = (mi, ti)
                            it["mesh_teeth"] = (la[mi][0], lb[ti][0])
                            lift = out[j]["lift"] + lb[ti][1] - la[mi][1]
                            if is_full(it) or is_full(out[j]):
                                lift = 0.0          # a full-height gear spans every layer: nothing moves up
                            if it["kind"] == "cluster":     # its top gear runs to the hub top, keeping CLUSTER_MIN_TOP
                                top, limit = lift + la[0][2] + CLUSTER_MIN_TOP, h["z_top"] - h["zb"]
                            elif it["mq"].get("pin") or it["mq"].get("full"):   # flat-topped: some gear under the top
                                top, limit = lift + PIN_FLAT_MIN, h["z_top"] - h["zb"]
                            else:
                                top, limit = lift + la[-1][2], room
                            if abs(lift) > 1e-9 and it["kind"] not in LIFTABLE:
                                issues.append("Rail %s's %s can't be raised on its hub to meet rail %s's %s gear."
                                              % (lab(i), LABELS[it["kind"]], lab(j),
                                                 "top" if ti else "bottom"))
                            elif lift < -1e-9 or top > limit + 1e-9:
                                issues.append("Rail %s's gear would sit %.1f mm %s its hub to meet rail %s; %s."
                                              % (lab(i), -lift if lift < 0 else top - limit,
                                                 "below" if lift < 0 else "above", lab(j),
                                                 "mesh another layer" if lift < 0 else "lengthen the shafts"))
                            else:
                                it["lift"] = lift
        placed[i] = x
        it["x"] = x
        if it["kind"] in LIFTABLE:
            it["mq"] = dict(it["mq"], lift=it["lift"])
        if it["kind"] == "cluster":
            it["mq"]["thickness_top"] = h["z_top"] - h["zb"] - it["lift"] - it["mq"]["thickness"]
        if it["kind"] == "spur" and (it["mq"].get("pin") or it["mq"].get("full")):
            it["mq"]["thickness"] = h["z_top"] - h["zb"] - it["lift"]
        lo, hi = x_range(q, i)
        if x < lo - 1e-6 or x > hi + 1e-6:
            fix = ""
            if it["kind"] == "yoke" and it.get("parent") is not None:
                too_long = x < lo if it["mq"]["dir"] > 0 else x > hi        # the holder ran off its far end
                fix = " Make the yoke %s or move rail %s's gear." % ("shorter" if too_long else "longer",
                                                                     lab(it["parent"]))
            issues.append("Rail %s's nut would be %.1f mm past the end of its travel.%s"
                          % (lab(i), max(lo - x, x - hi), fix))
        elif it["kind"] == "yoke":
            leg = it["mq"]["X"] - q["nut_length"] / 2          # the legs reach past the nut's ends
            if x < lo + leg - 1e-6 or x > hi - leg + 1e-6:
                issues.append("Rail %s's yoke holder legs would hit the end wall; move it %.1f mm in."
                              % (lab(i), max(lo + leg - x, x - hi + leg)))
        return x

    def place_yoke(i, j, seen, x):
        """A yoke (on i) links to the pin gear on the other nut of its row (j); the holder sits `length`
        from the gear, on its own side. Returns the holder's x (or x unchanged if the link can't work)."""
        it, par = out[i], out[j]
        if it["kind"] != "yoke":
            issues.append("Rail %s can't link to rail %s's yoke; link the yoke to this rail instead."
                          % (lab(i), lab(j)))
            return x
        if par["kind"] != "spur" or not par["mq"].get("pin"):
            issues.append("Rail %s's yoke has to link to a spur gear with a pin; turn on Pin for rail %s."
                          % (lab(i), lab(j)))
            return x
        if par["row"] != it["row"] or par["side"] == it["side"]:
            issues.append("Rail %s's yoke has to link to the other nut of its own row (Two-Sided)." % lab(i))
            return x
        xp = place(j, seen + (i,))
        s = 1.0 if par["side"] > it["side"] else -1.0         # the gear is toward +X from a left holder
        yq = it["mq"]
        yq.update(pin_radius=par["mq"]["pin_radius"], pin_diameter=par["mq"]["pin_diameter"], dir=s)
        lmin = math.ceil(yoke_min_length(yq, par["mq"]["ra"]) * 10 - 1e-6) / 10
        if yq["length"] < lmin:
            notes.append(("rail %s" % lab(i), "length", yq["length"], lmin))
            yq["length"] = lmin
        par["mq"]["pin_top"] = yq["z_hi"]                      # the pin comes up flush with the yoke's top
        it["parent"] = j
        it["offset"] = -s * yq["length"]
        it["distance"] = yq["length"]
        return xp + it["offset"]

    for i in range(len(out)):
        place(i)
    for it in out:
        if it["kind"] == "yoke" and not it.get("link"):
            issues.append("Rail %s's yoke needs a pin gear: link it to the other nut of its row." % lab(it["index"]))

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
        issues.append("Rail %s: a Geneva wheel can't be the input; turn its driver instead." % lab(k_in))
        progs[k_in] = dict(kind="static")
    if out[k_in]["kind"] == "yoke":
        issues.append("Rail %s: a yoke can't be the input; turn its pin gear instead." % lab(k_in))
        progs[k_in] = dict(kind="static")
    queue = [k_in] if progs[k_in]["kind"] == "input" else []
    while queue:
        p = queue.pop(0)
        for c in adj[p]:
            if c in progs:
                continue
            P, C = out[p], out[c]
            phi = math.atan2(C["y"] - P["y"], C["x"] - P["x"])
            if C["kind"] == "yoke":
                # the holder doesn't turn; the yoke slides with the pin (yoke_x)
                progs[c] = dict(kind="yoke", parent=p, r=P["mq"]["pin_radius"], dir=C["mq"]["dir"])
                continue
            if P["kind"] == "yoke":
                issues.append("Rail %s: the yoke on rail %s can't drive it." % (lab(c), lab(p)))
                continue
            if C["kind"] == "geneva_wheel" and P["kind"] == "geneva_driver":
                gq = P["mq"]
                progs[c] = dict(kind="geneva", parent=p, phi=phi, a=gq["a"], C=gq["center_distance"],
                                alpha=gq["alpha"], beta=gq["beta"])
            elif P["kind"] == "geneva_wheel":
                issues.append("Rail %s: the Geneva wheel on rail %s can't drive it." % (lab(c), lab(p)))
                continue
            else:
                # the layers that mesh: stored on whichever of the two is the linked child
                if C.get("parent") == p:
                    n2, n1 = C["mesh_teeth"]
                else:
                    n1, n2 = P["mesh_teeth"]
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


LABELS = dict(none="nothing", spur="spur gear", cluster="cluster gear", geneva_driver="Geneva driver",
              geneva_wheel="Geneva wheel", yoke="yoke")


def yoke_x(lay, i, an, xs=None):
    """World x of rail i's yoke slot centre for mechanism angles `an` (None if it isn't linked)."""
    it = lay["rails"][i]
    if it["kind"] != "yoke" or it.get("parent") is None:
        return None
    j = it["parent"]
    xc = (xs or [r["x"] for r in lay["rails"]])[j]
    return xc + it["mq"]["pin_radius"] * math.cos(an.get(j, 0.0))


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

GEAR_TAGS = ("gear", "gear_top")      # collision tags of a mechanism's gear layers, bottom up


def primitives(q, it):
    """[(tag, r, z0, z1)] about the rail's shaft axis (nut frame z)."""
    h = hub(q)
    k = it["kind"]
    if k == "none":
        return [("shaft", q["r_collar"], q["z_top"], q["z_tip"])]
    out = [("hub", h["rh"], q["z_top"], h["z_top"])]
    mq = it["mq"]
    zb = h["zb"]
    if k == "yoke":
        # the holder's box, as its corner circle; the sliding yoke is checked as swept rectangles
        return [("holder", math.hypot(mq["X"], mq["W"]), q["z_top"], mq["z_hi"])]
    if k in LIFTABLE:
        zl = zb + it.get("lift", 0.0)
        for n, (_z, z0, z1, ra) in enumerate(gear_layers(it)):
            out.append((GEAR_TAGS[n], ra, zl + z0, zl + z1))
        if mq.get("pin"):
            out.append(("pin", mq["pin_radius"] + mq["pin_diameter"] / 2, h["z_top"], mq["pin_top"]))
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
    Re = q["r_cap"]
    xm2 = 2 * q["x_mid"]
    for i in range(q["count"]):
        y = sr.carrier_y(q, i)
        out.append((x_far, x_neck, y - R, y + R, R))
        out.append((x_e0, x_e1, y - Re, y + Re, Re))
        if q["two_sided"]:          # the turned rail's knob and endstop
            out.append((xm2 - x_neck, xm2 - x_far, y - R, y + R, R))
            out.append((xm2 - x_e1, xm2 - x_e0, y - Re, y + Re, Re))
    return out


def collisions(q, rails, prims, knobs, xs=None):
    """Plain-language list of parts that would touch. Meshing pairs are skipped where they are meant
    to overlap in plan (gear tips with gear tips, a Geneva driver with its wheel)."""
    xs = xs or [it["x"] for it in rails]
    msgs = []
    linked = set()
    meshing = set()         # (rail, tag) pairs whose plan circles are meant to overlap their mate's
    for it in rails:
        if it.get("parent") is not None:
            linked.add(frozenset((it["index"], it["parent"])))
            mi, ti = it.get("mesh_layers", (0, 0))
            meshing.add((it["index"], it["parent"], GEAR_TAGS[mi], GEAR_TAGS[ti]))
    for i in range(len(rails)):
        for j in range(i + 1, len(rails)):
            pair = frozenset((i, j))
            d = math.hypot(xs[i] - xs[j], rails[i]["y"] - rails[j]["y"])
            geneva_pair = pair in linked and {rails[i]["kind"], rails[j]["kind"]} == {"geneva_driver", "geneva_wheel"}
            for ta, ra, a0, a1 in prims[i]:
                for tb, rb, b0, b1 in prims[j]:
                    if min(a1, b1) - max(a0, b0) <= 1e-6:
                        continue
                    if pair in linked and (geneva_pair or (i, j, ta, tb) in meshing or (j, i, tb, ta) in meshing):
                        continue
                    if d < ra + rb + COLLIDE_GAP - 1e-9:
                        msgs.append("Rail %s's %s hits rail %s's %s (%.1f mm short)."
                                    % (rail_label(q, i), _part(ta), rail_label(q, j), _part(tb),
                                       ra + rb + COLLIDE_GAP - d))
                        break
                else:
                    continue
                break
    for i, it in enumerate(rails):
        for tag, r, z0, z1 in prims[i]:
            if tag in ("shaft",):
                continue
            for x0, x1, y0, y1, ztop in knobs:
                if z0 >= ztop:
                    continue
                dx = max(x0 - xs[i], 0.0, xs[i] - x1)
                dy = max(y0 - it["y"], 0.0, it["y"] - y1)
                if math.hypot(dx, dy) < r + COLLIDE_GAP:
                    msgs.append("Rail %s's %s hits a knob or endstop; move the nut in." % (rail_label(q, i), _part(tag)))
                    break
            else:
                continue
            break
    msgs += yoke_collisions(q, rails, prims, xs)
    return msgs


def yoke_rects(q, it, xs):
    """Plan rectangles [(name, x0, x1, y0, y1, z0, z1)] the sliding yoke sweeps over a full turn."""
    yq = it["mq"]
    xc, r, s = xs[it["parent"]], yq["pin_radius"], yq["dir"]
    sx, sy, fx, fy = yoke_frame(yq)
    u_end = -(yq["length"] + yq["X"] + r + yq["tail"])
    y, z0, z1 = it["y"], yq["z_ch"], yq["z_hi"]
    bar = sorted((xc - s * r + s * u_end, xc + s * r - s * fx))
    return [("yoke frame", xc - r - fx, xc + r + fx, y - fy, y + fy, z0, z1),
            ("yoke bar", bar[0], bar[1], y - yq["bw"], y + yq["bw"], z0, z1)]


def yoke_collisions(q, rails, prims, xs):
    """The sliding yokes against everything else that reaches their height (holders, pins, other
    yokes); a yoke's own holder and pin gear are what it's meant to touch."""
    msgs = []
    sweeps = {it["index"]: yoke_rects(q, it, xs) for it in rails if it["kind"] == "yoke" and it.get("parent") is not None}
    for i, rects in sweeps.items():
        own = (i, rails[i]["parent"])
        hit = None
        for j, it in enumerate(rails):
            if j in own:
                continue
            for tag, r, z0, z1 in prims[j]:
                for name, x0, x1, y0, y1, a0, a1 in rects:
                    if min(z1, a1) - max(z0, a0) <= 1e-6:
                        continue
                    dx = max(x0 - xs[j], 0.0, xs[j] - x1)
                    dy = max(y0 - it["y"], 0.0, it["y"] - y1)
                    if math.hypot(dx, dy) < r + COLLIDE_GAP - 1e-9:
                        hit = "Rail %s's %s hits rail %s's %s as it slides." % (
                            rail_label(q, i), name, rail_label(q, j), _part(tag))
                        break
                if hit:
                    break
            if hit:
                break
            if j in sweeps and j > i:
                for name, x0, x1, y0, y1, a0, a1 in rects:
                    for nb, u0, u1, v0, v1, b0, b1 in sweeps[j]:
                        if min(x1, u1) - max(x0, u0) > -COLLIDE_GAP and min(y1, v1) - max(y0, v0) > -COLLIDE_GAP:
                            hit = "Rail %s's yoke and rail %s's yoke run into each other." % (
                                rail_label(q, i), rail_label(q, j))
                            break
                    if hit:
                        break
            if hit:
                break
        if hit:
            msgs.append(hit)
    return msgs


def _part(tag):
    return dict(shaft="shaft", hub="hub", gear="gear", gear_top="top gear", plate="crank plate", crank="pin and locking disc",
                wheel="wheel", pin="pin", holder="yoke holder").get(tag, tag)


# ── Meshes ───────────────────────────────────────────────────────────────────

def build_mech(lay, i):
    it = lay["rails"][i]
    q, g = lay["q"], lay["gear"]
    if it["kind"] == "spur":
        return build_spur(q, g, it["mq"])
    if it["kind"] == "cluster":
        return build_cluster(q, g, it["mq"])
    if it["kind"] == "geneva_driver":
        return build_geneva_driver(q, g, it["mq"])
    if it["kind"] == "geneva_wheel":
        return build_geneva_wheel(q, g, it["mq"])
    if it["kind"] == "yoke":
        return build_yoke_holder(q, g, it["mq"])
    return None


def build_yoke_part(lay, i):
    """Rail i's sliding yoke (its own frame, see build_yoke), or None."""
    it = lay["rails"][i]
    if it["kind"] != "yoke":
        return None
    return build_yoke(lay["q"], lay["gear"], it["mq"])
