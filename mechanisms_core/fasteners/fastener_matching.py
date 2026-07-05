"""
Shared Match Target helpers for the fastener family.

[EXCEPTION TO THE FAMILY CONVENTION] docs/fasteners/README.md states this
family deliberately does NOT share a helper module — every generator
duplicates its own thread-geometry math verbatim rather than importing it.
This file is a deliberate exception, not an inconsistency: it holds no
thread geometry math at all, only the Match Target picker/sync/stamp
machinery, which is infrastructure in the same category as menu.py (already
shared across every family regardless of their geometry-code philosophy),
not the kind of per-generator domain math that convention is about. The
concrete, mechanical reason it can't be duplicated even in principle: the
Match Target picker is a single `WindowManager` property. Two independent
copies of a `register()` that each assign
`bpy.types.WindowManager.bmech_fastener_target = PointerProperty(...)`
wouldn't coexist — the second one to run silently replaces the first's
poll/update functions, so only one generator's "copy" would actually ever
be live, an entirely different failure mode than duplicating a pure
function like `_thread_params`. See gears/gear_matching.py's own module
docstring for the general Blender-API background this mirrors (a picker
can't live on the operator itself either, for a separate reason).

hex_bolt.py and hex_nut.py both stamp bmech_thread_diameter/bmech_pitch/
bmech_flank_angle_deg/bmech_truncation identically (already true before
this file existed) — a bolt and the nut that fits it need all four values
to match exactly for the threads to physically engage, unlike the gear
family's kind-dependent freezing (spur vs. helical targets, hand
ambiguity, etc.) — there's no partial-match nuance here, so this module
is much smaller than gear_matching.py.
"""

import bpy
from bpy.props import PointerProperty


def stamp_thread(obj, kind, thread_diameter, pitch, flank_angle_deg, truncation):
    obj["bmech_kind"]             = kind
    obj["bmech_thread_diameter"]  = thread_diameter
    obj["bmech_pitch"]            = pitch
    obj["bmech_flank_angle_deg"]  = flank_angle_deg
    obj["bmech_truncation"]       = truncation


def fastener_target_poll(_self, obj):
    return obj.type == 'MESH' and "bmech_thread_diameter" in obj.keys()


def sync_thread_dims(op, target):
    """Copy thread_diameter/pitch/flank_angle/truncation from target — the
    full set every thread-based fastener generator needs, all at once,
    since a bolt and its mating nut must match on all four simultaneously."""
    if target is None:
        return
    if "bmech_thread_diameter" in target.keys():
        op.thread_diameter_mm = target["bmech_thread_diameter"]
    if "bmech_pitch" in target.keys():
        op.pitch_mm = target["bmech_pitch"]
    if "bmech_flank_angle_deg" in target.keys():
        op.flank_angle_deg = target["bmech_flank_angle_deg"]
    if "bmech_truncation" in target.keys():
        op.truncation = target["bmech_truncation"]


def _on_target_change(wm, context):
    if wm.bmech_fastener_target is None:
        # Same reset-vs-pick distinction gear_matching.py's own
        # _on_target_change documents: reset_target() clearing the picker
        # at the start of every new part's invoke() ALSO fires this update
        # callback, and treating that as a real pick would re-run whatever
        # operator context.active_operator still points at (the PREVIOUS
        # part) at the 3D cursor's now-moved position.
        return
    op = context.active_operator
    if op is not None and hasattr(op, "bmech_sync_target"):
        # Mirrors gear_matching.py's own _on_target_change exactly — see
        # that function's docstring for why this specific delete-and-
        # re-execute approach is used instead of bpy.ops.ed.undo().
        stale = list(context.selected_objects)
        active = context.view_layer.objects.active
        if active is not None and active not in stale:
            stale.append(active)
        for obj in stale:
            mesh = obj.data if obj.type == 'MESH' else None
            bpy.data.objects.remove(obj, do_unlink=True)
            if mesh is not None and mesh.users == 0:
                bpy.data.meshes.remove(mesh)

        op.bmech_sync_target(context)
        op.execute(context)


def reset_target(context):
    """Clear the Match Target picker — call from each operator's invoke(),
    never execute(). See gear_matching.reset_target's docstring for why."""
    context.window_manager.bmech_fastener_target = None


def register():
    bpy.types.WindowManager.bmech_fastener_target = PointerProperty(
        name="Match Target", type=bpy.types.Object,
        poll=fastener_target_poll, update=_on_target_change,
        description="Pick an existing threaded fastener to copy thread dimensions from",
    )


def unregister():
    del bpy.types.WindowManager.bmech_fastener_target
