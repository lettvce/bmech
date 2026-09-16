"""
Pure geometry core for stadium_utils.py - zero bpy/bmesh/mathutils
dependency already (this module never needed extraction so much as
relocation: stadium_utils.py's own math was always bpy-free, only its
`from .. import geo_helpers as gh` relative import needed the same
self-locating boilerplate every other *_core.py file in this project
uses). Shared by gearbox_housing_core.py and gearbox_lid_core.py - see
stadium_utils.py's own module docstring for the full geometric
reasoning (seam points, why stadium_half_loops caps correctly at any
half_span/radius ratio, why rotate_loop_start is needed on one side and
not the other).
"""

import sys
import os
from math import pi, cos, sin

GEO_HELPERS_DIR = r"C:\Users\Safu\OneDrive\Desktop\Blender\Mechanism Library\mechanisms_core"


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


def stadium_points(half_span, radius, arc_segments):
    """CCW boundary of a stadium, WITH an explicit seam vertex at
    (0, +radius) and (0, -radius). See stadium_utils.py's own docstring
    for why the seam points exist."""
    pts = []
    for i in range(arc_segments + 1):
        theta = -pi / 2.0 + pi * i / arc_segments
        pts.append((half_span + radius * cos(theta), radius * sin(theta)))
    pts.append((0.0, radius))
    for i in range(arc_segments + 1):
        theta = pi / 2.0 + pi * i / arc_segments
        pts.append((-half_span + radius * cos(theta), radius * sin(theta)))
    pts.append((0.0, -radius))
    return pts


def stadium_loop(half_span, radius, arc_segments, z, name=""):
    pts = stadium_points(half_span, radius, arc_segments)
    return gh.Loop([(x, y, z) for x, y in pts], True, name)


def stadium_half_loops(half_span, radius, arc_segments, z):
    """Right-half and left-half CLOSED loops - each [seam point, one
    full cap sweep, the other seam point] - bit-identical slices of the
    SAME stadium_points() call used everywhere else."""
    pts = stadium_points(half_span, radius, arc_segments)
    n_cap = arc_segments + 1
    seam_bottom = pts[-1]
    seam_top = pts[n_cap]
    right_cap = pts[0:n_cap]
    left_cap = pts[n_cap + 1:n_cap + 1 + n_cap]
    right = gh.Loop([(x, y, z) for x, y in ([seam_bottom] + right_cap + [seam_top])], True, "right_half")
    left = gh.Loop([(x, y, z) for x, y in ([seam_top] + left_cap + [seam_bottom])], True, "left_half")
    return right, left


def rotate_loop_start(loop, offset):
    """Cyclically shift a closed loop's own starting index - needed
    before bridging a boss/bore ring (gh.circle, whose index 0 always
    sits at angle 0 relative to ITS OWN center) to a half-stadium
    boundary (whose own index 0 always starts at the seam)."""
    n = len(loop.verts)
    offset = offset % n
    return gh.Loop(loop.verts[offset:] + loop.verts[:offset], loop.is_closed, loop.name)
