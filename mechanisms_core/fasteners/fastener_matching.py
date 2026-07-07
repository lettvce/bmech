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

[ORIENTATION-RESTRICTED POLL — a deliberate divergence from the gear
family] gear_target_poll is intentionally loose (any object with
bmech_module, no kind check — see gears/README.md's Match Target section
for why). fastener_target_poll here is deliberately NOT loose: an
external thread can only ever mate with an internal one — there is no
physical case where two external threads or two internal threads fit
together, unlike the gear family's legitimate cross-kind cases (a helical
gear sharing a hub with a spur gear on a compound gear, for instance).
This is a hard constraint of the domain, not an arbitrary tightening, so
it's enforced in the poll itself rather than left to the user the way
gears leave meshing correctness up to the user.

Enforcing that requires the poll to know which orientation the ASKING
operator has — hex_bolt.py and threaded_container.py are always EXTERNAL,
hex_nut.py and threaded_lid.py are always INTERNAL, but
threaded_fastener.py can be either, decided live by its own `thread_type`
property. A PointerProperty's poll callback only receives
(self, object) — no operator reference — so this reads
`bpy.context.active_operator` directly (the same global-context pattern
gear_matching.py's own update callback already relies on) rather than
being handed it as a parameter. Like every other pick/reset/rebuild
behavior in this pattern, this cannot be exercised by a headless test —
`context.active_operator` is always None in `--background` mode — budget
for manual GUI verification specifically of "does the target dropdown
correctly exclude same-orientation objects" before trusting this.
"""

import bpy
from bpy.props import PointerProperty


def stamp_thread(obj, kind, thread_diameter, pitch, flank_angle_deg, truncation):
    obj["bmech_kind"]             = kind
    obj["bmech_thread_diameter"]  = thread_diameter
    obj["bmech_pitch"]            = pitch
    obj["bmech_flank_angle_deg"]  = flank_angle_deg
    obj["bmech_truncation"]       = truncation


def fastener_orientation(obj):
    """
    'EXTERNAL' or 'INTERNAL' for a stamped fastener object, or None if it
    isn't one. hex_bolt.py always stamps kind="hex_bolt" (fixed EXTERNAL);
    hex_nut.py always stamps kind="hex_nut" (fixed INTERNAL);
    threaded_fastener.py stamps kind="external_thread"/"internal_thread"
    depending on its own live thread_type at build time, since a raw
    thread can be built as either.
    """
    if "bmech_thread_diameter" not in obj.keys():
        return None
    kind = obj.get("bmech_kind", "")
    if kind in ("hex_bolt", "external_thread", "threaded_container"):
        return "EXTERNAL"
    if kind in ("hex_nut", "internal_thread", "threaded_lid"):
        return "INTERNAL"
    return None


def fastener_target_poll(_self, obj):
    if obj.type != 'MESH':
        return False
    target_orientation = fastener_orientation(obj)
    if target_orientation is None:
        return False

    # Figure out which orientation the ASKING operator needs — see the
    # module docstring's [ORIENTATION-RESTRICTED POLL] section. Falls back
    # to permissive (any correctly-stamped fastener) if the asking
    # operator can't be identified, e.g. headless mode or a future
    # fastener generator this poll doesn't know about yet — matching this
    # pattern's own established fallback of "don't hard-fail when context
    # is unavailable, degrade to the old loose behavior instead."
    op = bpy.context.active_operator
    if op is None:
        return True
    idname = getattr(op, "bl_idname", "")
    if idname in ("object.hex_bolt", "object.threaded_container"):
        return target_orientation == "INTERNAL"
    if idname in ("object.hex_nut", "object.threaded_lid"):
        return target_orientation == "EXTERNAL"
    if idname == "object.add_threaded_fastener":
        self_orientation = "EXTERNAL" if op.thread_type == 'EXTERNAL' else "INTERNAL"
        return target_orientation != self_orientation
    return True


def sync_thread_dims(op, target):
    """Copy thread_diameter/pitch/flank_angle/truncation from target — the
    full set every thread-based fastener generator needs, all at once,
    since a bolt and its mating nut must match on all four simultaneously.
    Used by hex_bolt.py/hex_nut.py, whose orientation is fixed by kind —
    see sync_raw_thread for threaded_fastener.py, whose orientation is a
    live property this also has to drive."""
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


def sync_raw_thread(op, target):
    """Sync for threaded_fastener.py specifically. Two differences from
    sync_thread_dims: its diameter property is named diameter_mm, not
    thread_diameter_mm (pre-existing naming, unrelated to this feature),
    and it must ALSO force its own thread_type to the opposite of the
    target's orientation — unlike hex_bolt/hex_nut, which are always one
    fixed orientation, a raw thread can be built as either, so picking a
    target has to pin down which one it must now be. operation
    (additive/subtractive) is deliberately left alone: it only controls
    HOW the internal or external thread gets built, not whether it fits,
    so it stays a free user choice even with a target set."""
    if target is None:
        return
    if "bmech_thread_diameter" in target.keys():
        op.diameter_mm = target["bmech_thread_diameter"]
    if "bmech_pitch" in target.keys():
        op.pitch_mm = target["bmech_pitch"]
    if "bmech_flank_angle_deg" in target.keys():
        op.flank_angle_deg = target["bmech_flank_angle_deg"]
    if "bmech_truncation" in target.keys():
        op.truncation = target["bmech_truncation"]

    orientation = fastener_orientation(target)
    if orientation == "EXTERNAL":
        op.thread_type = 'INTERNAL'
    elif orientation == "INTERNAL":
        op.thread_type = 'EXTERNAL'


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
