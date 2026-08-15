"""
Pure geometry core for elliptical_mouth.py - every function here is
plain Python + geo_helpers math, zero bpy/bmesh/mathutils dependency.
Extracted verbatim (not rewritten) from elliptical_mouth.py so this
exact logic can run outside Blender too (the web configurator, via
Pyodide) with a single source of truth - elliptical_mouth.py itself
now just imports build_elliptical_mouth_mesh from here and wraps it
in a bpy object/operator.

If this file changes, elliptical_mouth.py's own behavior changes too -
there is no separate copy of this logic left in that file. The web
configurator's own copy of this file is a vendored, synced copy (see
dev/sync_web_configurator.py), same discipline as geo_helpers.py's own
vendoring elsewhere in this project - re-sync after editing this file.

Parametric elliptical mouth generator - the same circle-neck / loft /
straight-run-mouth architecture as rectangular_mouth_core.py, just with
an ellipse instead of a rounded rectangle for the mouth's own
cross-section. Same parameter names carry over directly (id_mm, wall_mm,
mouth_width_mm/mouth_depth_mm, neck/loft/mouth heights, mouth_tilt_deg,
segments) - there's just no mouth_corner_radius_mm, since an ellipse has
no corners to round.

Every cross-section is sampled at the SAME set of angles (derived once
per outer/inner family from the mouth's own shape, evenly spaced by arc
length) for the same reason as the rectangular version: it's what keeps
the neck-to-mouth loft from twisting while still landing perimeter-even
spacing on the mouth's own boundary. geo_helpers.py DOES ship a plain
ellipse() primitive, but it samples angle-uniformly (dense near the
tight/minor-axis ends of an elongated ellipse, sparse near the flatter
major-axis ends) - the same density problem the rectangle mouth had, so
it's not used here; perimeter_uniform_thetas_ellipse below replaces it.

Unlike the rounded-rect's boundary (built from exact straight-edge/arc
pieces), an ellipse's arc length has no elementary closed form, so its
perimeter-uniform thetas are derived numerically - fine-sampled cumulative
arc length, then linear interpolation - rather than exactly.
"""

import sys
import os
import math

# Pre-set to this project's own folder, where geo_helpers.py already
# lives - makes pasting this into a blank Text Editor tab in an unsaved
# .blend work with no manual step. Only change this if the project
# folder ever moves. When vendored into the web configurator, geo_helpers.py
# sits alongside this file, so the plain sys.path fallback below finds it
# there too without needing GEO_HELPERS_DIR set at all.
GEO_HELPERS_DIR = r"C:\Users\Safu\OneDrive\Desktop\cuum"


def _script_dir():
    if GEO_HELPERS_DIR and os.path.isfile(os.path.join(GEO_HELPERS_DIR, "geo_helpers.py")):
        return GEO_HELPERS_DIR
    candidate = os.path.dirname(os.path.abspath(__file__))
    if os.path.isfile(os.path.join(candidate, "geo_helpers.py")):
        return candidate
    raise RuntimeError(
        "Can't find geo_helpers.py - either place it next to this file, "
        "or set GEO_HELPERS_DIR near the top of this script to wherever "
        "you saved it."
    )


sys.path.insert(0, _script_dir())
import geo_helpers as gh


def ellipse_point_at_angle(rx, ry, theta):
    """Point where the ray from center at angle theta crosses an ellipse
    (semi-axes rx, ry) - NOT the standard parametric ellipse point
    (rx*cos(theta), ry*sin(theta)), which is a different, subtly wrong
    thing here: for rx != ry that parametric angle is NOT the same as
    the point's own actual geometric angle from center, so sharing those
    thetas between a circle (the neck) and an ellipse (the mouth) would
    silently misalign them and twist the loft between them - exactly the
    bug this ray-casting definition avoids, the same way
    rounded_rect_point_at_angle already does for the rectangular mouth.
    atan2(y, x) of this function's own result equals theta exactly, for
    any rx, ry - solve t^2*(cos^2(theta)/rx^2 + sin^2(theta)/ry^2) = 1
    for the ray parameter t, then scale (cos theta, sin theta) by it.
    Reduces exactly to a circle of radius r when rx == ry == r."""
    c, s = math.cos(theta), math.sin(theta)
    t = 1.0 / math.sqrt((c * c) / (rx * rx) + (s * s) / (ry * ry))
    return (t * c, t * s)


def perimeter_uniform_thetas_ellipse(rx, ry, segments, samples=4000):
    """Angles of `segments` points evenly spaced by ARC LENGTH around an
    ellipse (rx, ry) - not by angle, which packs points densely near the
    tight (minor-axis) ends of an elongated ellipse and sparsely near the
    flatter (major-axis) ends.

    An ellipse's arc length has no elementary closed form (unlike the
    rounded-rect's own exact straight-edge/arc pieces), so this walks a
    fine angular sampling (`samples`, cheap - pure Python floats, no mesh
    data involved), accumulates straight-line distance between
    consecutive fine points as an arc-length approximation, then
    linearly interpolates the angle for each of the `segments` target
    arc-length positions. At 4000 fine samples for a typical 128-segment
    target this is precise well beyond what a print or even a screen can
    show - each output segment is backed by ~31 fine samples.

    Returns ANGLES, not points, same reason as
    rounded_rect's own perimeter_uniform_thetas: reused verbatim to
    sample every OTHER cross-section in the same loft family too."""
    if abs(rx - ry) < 1e-9:
        return [2.0 * math.pi * i / segments for i in range(segments)]

    fine_thetas = [2.0 * math.pi * k / samples for k in range(samples + 1)]
    fine_pts = [ellipse_point_at_angle(rx, ry, t) for t in fine_thetas]
    cum = [0.0]
    for k in range(samples):
        x0, y0 = fine_pts[k]
        x1, y1 = fine_pts[k + 1]
        cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
    total = cum[-1]

    thetas = []
    j = 0
    for i in range(segments):
        target = total * i / segments
        while j < samples - 1 and cum[j + 1] < target:
            j += 1
        seg_len = cum[j + 1] - cum[j]
        t = (target - cum[j]) / seg_len if seg_len > 1e-12 else 0.0
        thetas.append(fine_thetas[j] + (fine_thetas[j + 1] - fine_thetas[j]) * t)
    return thetas


def ring_at_thetas(rx, ry, thetas):
    return [ellipse_point_at_angle(rx, ry, theta) for theta in thetas]


def _tilt_z_fn(tilt_deg):
    """None (flat, the default) or a function adding an X-proportional Z
    offset - tilts a ring's own plane around the Y axis without touching
    its (x, y) shape at all. Symmetric about x=0, so the ring's nominal Z
    stays its average height, not its minimum.

    Tilts around Y (not X) specifically so this lands in the SAME plane
    neck_connector_core.py's own bend curves in - see
    rectangular_mouth_core.py's own matching _tilt_z_fn comment for the
    full reasoning (identical here)."""
    if tilt_deg == 0.0:
        return None
    slope = math.tan(math.radians(tilt_deg))
    return lambda x, y: slope * x


# Minimum clearance the tilted opening ring must keep above the mouth's
# own flat base - same 0.4-1mm floor-margin convention this project uses
# everywhere else (merch_core.py's own hole/bevel/wall clamps), just
# applied here as a reject condition instead of a clamp (see
# _validate_mouth_tilt's own docstring for why this one rejects rather
# than silently substitutes a value).
MOUTH_TILT_MARGIN_MM = 0.5


def _validate_mouth_tilt(mouth_height_mm, mouth_width_mm, mouth_tilt_deg):
    """Raises a clear ValueError if mouth_tilt_deg would tilt the opening
    ring at or below the mouth's own flat base on one side, rather than
    silently clamping to whatever tilt IS safe. Clamping is the right
    call for a value like Wall Thickness or Loft Height, where the
    clamped result is still basically what was asked for - but Mouth
    Tilt is a deliberate, specific choice, and silently downgrading a
    requested 45 degrees to some much smaller safe value would produce a
    result the caller didn't ask for and might not notice. Same "reject
    with an actionable message" pattern label_maker's own text-overflow
    check already uses, for the same reason.

    Checked against mouth_width_mm (the X extent), not mouth_depth_mm -
    the tilt is around the Y axis, so the opening's lowest point is at
    the +/-X extreme."""
    if mouth_tilt_deg == 0.0:
        return
    slope = math.tan(math.radians(abs(mouth_tilt_deg)))
    lowest_drop = slope * (mouth_width_mm / 2.0)
    clearance = mouth_height_mm - lowest_drop
    if clearance < MOUTH_TILT_MARGIN_MM:
        max_tilt = math.degrees(math.atan(
            2.0 * (mouth_height_mm - MOUTH_TILT_MARGIN_MM) / mouth_width_mm
        )) if mouth_height_mm > MOUTH_TILT_MARGIN_MM else 0.0
        raise ValueError(
            f"Mouth Tilt of {mouth_tilt_deg:.1f} deg drops the opening "
            f"{lowest_drop:.2f}mm below Mouth Height ({mouth_height_mm:.2f}mm) on one side, "
            f"leaving only {clearance:.2f}mm of clearance above the mouth's own base - under "
            f"the {MOUTH_TILT_MARGIN_MM:.1f}mm minimum. Increase Mouth Height, reduce Mouth "
            f"Width, or keep Mouth Tilt under {max_tilt:.1f} deg."
        )


def build_elliptical_mouth_mesh(
    id_mm, wall_mm,
    mouth_width_mm, mouth_depth_mm,
    neck_height_mm, loft_height_mm, mouth_height_mm,
    mouth_tilt_deg=0.0,
    segments=128, cap_neck=True,
):
    """Returns a geo_helpers Mesh - pure geometry, no bpy. Sealed at both
    ends by default (a full solid, printable/previewable on its own -
    see this project's own established convention). z=0 is the neck's
    own free end; z=total_height is the mouth opening.

    cap_neck=False skips the neck-end annular cap - for assembling onto
    a neck connector's own matching (uncapped) end via geo_helpers'
    automatic vertex welding, not a boolean union - see
    rectangular_mouth_core.py's build_rectangular_mouth_mesh docstring
    for the full reasoning (identical here).

    neck_outer_r is DERIVED, not independently specified - id_mm is what a
    caller actually knows (it has to match a real shaft), and wall_mm is
    the other real design constraint, so neck_outer_r = id_mm/2 + wall_mm
    follows from those two rather than being its own free parameter that
    could silently drift out of sync with them.

    mouth_tilt_deg tilts ONLY the opening ring (the top of mouth_height_mm's
    own straight run) around the Y axis - the mouth's bottom ring (where
    the loft from the neck ends) stays flat. The straight side walls
    already connecting a flat ring to a tilted one come out taller on one
    side and shorter on the other with no other change needed - exactly
    what a tube sliced at an angle looks like. 0 = flat (square-cut).
    Tilts around Y (not X) so it leans in the SAME plane
    neck_connector_core.py's own bend curves in - see _tilt_z_fn's own
    docstring for why. Raises ValueError (does not clamp) if the tilt
    would drop the opening at or below the mouth's own flat base - see
    _validate_mouth_tilt's own docstring for why this one rejects."""
    _validate_mouth_tilt(mouth_height_mm, mouth_width_mm, mouth_tilt_deg)
    neck_inner_r = id_mm / 2.0
    neck_outer_r = neck_inner_r + wall_mm
    mouth_rx, mouth_ry = mouth_width_mm / 2.0, mouth_depth_mm / 2.0

    z_neck_bottom = 0.0
    z_neck_top = neck_height_mm
    z_mouth_bottom = neck_height_mm + loft_height_mm
    z_mouth_top = neck_height_mm + loft_height_mm + mouth_height_mm

    tilt = _tilt_z_fn(mouth_tilt_deg)
    mouth_inner_rx = max(mouth_rx - wall_mm, 0.05)
    mouth_inner_ry = max(mouth_ry - wall_mm, 0.05)

    # Both neck levels (z=0, the actual external joint, and
    # z=neck_height_mm) use plain even-angle thetas - the same formula
    # gh.circle() itself uses, and also the ideal spacing for a plain
    # circle regardless. Sharing the identical array between these two
    # rings makes the straight run a single, genuinely untwisted
    # cylindrical band.
    #
    # The mouth's own rim (mouth-bottom, mouth-top) shares its own
    # perimeter_uniform_thetas_ellipse array instead - required to keep
    # points evenly spread around the ellipse's own perimeter rather
    # than bunched near the tight ends.
    #
    # Unlike rectangular_mouth_core.py's own perimeter_uniform_thetas,
    # perimeter_uniform_thetas_ellipse needed no re-anchoring fix here -
    # its own arc-length walk already starts at fine_thetas[0] == 0 by
    # construction, so index 0 already sits at angle 0, matching the
    # neck's own convention. The residual mismatch between neck-top
    # (even-angle) and mouth-bottom (perimeter-uniform) is real but
    # small enough - ~16 degrees worst case for this project's own
    # default mouth, smoothly graded around the ring - to read as a
    # normal loft rather than a twist in a single plain band, no
    # subdivision needed (matches rectangular_mouth_core.py's own
    # equivalent fix, and a hand-cleaned reference assembly's own plain
    # single-band structure).
    neck_thetas = [2.0 * math.pi * k / segments for k in range(segments)]
    outer_thetas = perimeter_uniform_thetas_ellipse(mouth_rx, mouth_ry, segments)
    inner_thetas = perimeter_uniform_thetas_ellipse(mouth_inner_rx, mouth_inner_ry, segments)

    def ell_loop(rx, ry, z, thetas, tilt_fn, loop_name):
        pts = ring_at_thetas(rx, ry, thetas)
        if tilt_fn is None:
            return gh.Loop([(x, y, z) for x, y in pts], True, loop_name)
        return gh.Loop([(x, y, z + tilt_fn(x, y)) for x, y in pts], True, loop_name)

    levels = [
        (z_neck_bottom, neck_outer_r, neck_outer_r, None),
        (z_neck_top, neck_outer_r, neck_outer_r, None),
        (z_mouth_bottom, mouth_rx, mouth_ry, None),
        (z_mouth_top, mouth_rx, mouth_ry, tilt),
    ]
    outer_loops = [
        ell_loop(rx, ry, z, neck_thetas if i < 2 else outer_thetas, tilt_fn, "outer_%d" % i)
        for i, (z, rx, ry, tilt_fn) in enumerate(levels)
    ]
    inner_loops = [
        ell_loop(max(rx - wall_mm, 0.05), max(ry - wall_mm, 0.05), z,
                 neck_thetas if i < 2 else inner_thetas, tilt_fn, "inner_%d" % i)
        for i, (z, rx, ry, tilt_fn) in enumerate(levels)
    ]

    # Both loop families wind CCW-from-+Z (same convention
    # ellipse_point_at_angle/perimeter_uniform_thetas_ellipse trace in),
    # and every level sits at increasing Z, so gh.loft() on the outer
    # loops alone already gives outward-facing side walls (geo_helpers'
    # own convention: "outward when both rings are CCW and top sits
    # along bottom's own normal" - true here since higher Z IS "along
    # +Z" from a lower CCW-from-+Z ring). The inner loops loft the same
    # way but need flip_normals: "outward from the inner tube's own
    # axis" there actually means "into the solid wall", the wrong side
    # for a proper manifold, where every face must point away from
    # material.
    outer_shell = gh.loft(outer_loops)
    inner_shell = gh.flip_normals(gh.loft(inner_loops))
    # Bottom (z=0) cap needs to face -Z (out through the neck's own free
    # end); a CCW-from-+Z loop's cap naturally faces +Z, so reverse=True.
    # Top (mouth opening) cap already wants +Z, its own natural facing.
    mesh = outer_shell + inner_shell
    if cap_neck:
        mesh = mesh + gh.cap_annulus(outer_loops[0], inner_loops[0], reverse=True)
    mesh = mesh + gh.cap_annulus(outer_loops[-1], inner_loops[-1])

    report = mesh.validate(require_closed=cap_neck)
    if not report.is_valid:
        raise ValueError("build_elliptical_mouth_mesh produced an invalid mesh: %s" % report.errors)
    return mesh, dict(neck_outer_loop=outer_loops[0], neck_inner_loop=inner_loops[0])
