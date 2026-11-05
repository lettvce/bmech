"""mesh2d.py - welded mesh + hole-bridging triangulator, vendored from chain_link/chain_link.py."""

import math

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
            for j in refl:
                if j in (a, c) or P[j] in (A, B, Cc):
                    continue
                if _in_tri(P[j], A, B, Cc):
                    ear = False
                    break
        if not ear:
            i = nxt[i]
            stall += 1
            if stall <= left:
                continue
            # only degenerate (collinear) vertices left: drop the flattest one
            j, best = i, None
            for _ in range(left):
                o = abs(_orient(P[prv[j]], P[j], P[nxt[j]]))
                if best is None or o < best[0]:
                    best = (o, j)
                j = nxt[j]
            b = best[1]
            a, c = prv[b], nxt[b]
        else:
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
            if _orient(pts[u], pts[v], pts[w]) < 0:
                u, v = v, u
            if not _incircle(pts[u], pts[v], pts[w], pts[x]):
                continue
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
    ax, ay = a[0] - d[0], a[1] - d[1]
    bx, by = b[0] - d[0], b[1] - d[1]
    cx, cy = c[0] - d[0], c[1] - d[1]
    det = ((ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
           + (cx * cx + cy * cy) * (ax * by - bx * ay))
    scale = max(abs(ax), abs(ay), abs(bx), abs(by), abs(cx), abs(cy), 1.0) ** 4
    return det > 1e-10 * scale


# â”€â”€ Solid â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class _Mesh:
    """Vertices are welded by position, so faces built separately share their boundary vertices
    as long as both sides compute the same points."""

    def __init__(self):
        self.verts, self.faces, self._key = [], [], {}

    def v(self, x, y, z):
        key = (round(x, 7), round(y, 7), round(z, 7))
        i = self._key.get(key)
        if i is None:
            i = self._key[key] = len(self.verts)
            self.verts.append((x, y, z))
        return i

    def face(self, outer, holes, normal):
        """Planar face over index loops, any winding; triangles face `normal`."""
        n = normal
        a = (1.0, 0.0, 0.0) if abs(n[0]) < 0.9 else (0.0, 1.0, 0.0)
        u = (a[1] * n[2] - a[2] * n[1], a[2] * n[0] - a[0] * n[2], a[0] * n[1] - a[1] * n[0])
        w = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0])
        pts = {}
        for i in list(outer) + [i for h in holes for i in h]:
            p = self.verts[i]
            pts[i] = (p[0] * u[0] + p[1] * u[1] + p[2] * u[2], p[0] * w[0] + p[1] * w[1] + p[2] * w[2])

        def area(loop):
            return _area([pts[i] for i in loop])

        outer = list(outer) if area(outer) > 0 else list(outer)[::-1]
        holes = [list(h) if area(h) < 0 else list(h)[::-1] for h in holes]
        if not holes and len(outer) <= 4 and all(
                _orient(pts[outer[i - 2]], pts[outer[i - 1]], pts[outer[i]]) > 1e-12 for i in range(len(outer))):
            self.faces.append(tuple(outer))                           # convex quad/tri as is
            return
        self.faces += triangulate(pts, outer, holes)

    def strip(self, a, b):
        """Quads between two matching open polylines; outward when a runs CCW (seen from the
        outside) below b."""
        for i in range(len(a) - 1):
            self.faces.append((a[i], a[i + 1], b[i + 1], b[i]))

    def band(self, a, b):
        self.strip(a + a[:1], b + b[:1])


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

