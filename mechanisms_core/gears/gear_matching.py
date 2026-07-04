"""
Shared gear-matching helpers.

Any gear operator that stamps its output object with bmech_* custom
properties becomes selectable as a "Match Target" by any other gear
operator in this package. Picking a target copies module, pressure
angle, and (where applicable) helix angle / hand / tooth count so two
parts are guaranteed to mesh instead of being typed in twice by hand.

The picker itself lives on WindowManager rather than on the operators:
bpy.types.Operator does not support PointerProperty to ID types like
Object (registration silently drops that property and every property
declared after it in the class), so each operator instead reads the
shared bmech_gear_target and implements a bmech_sync_target(context)
method that the WindowManager property's update callback dispatches to.
"""

import bpy
from bpy.props import PointerProperty
from math import pi, cos, tan, atan, sqrt, radians, degrees


def tooth_profile_ok(tooth_count, pressure_angle_deg, tip_addendum_coeff):
    """
    False if an involute tooth with this tooth count / pressure angle / tip
    reach self-intersects (the classic "pointed tooth" limit) — the tooth's
    two flanks cross before reaching the tip, which produces a self-crossing
    2D profile that the boolean solver can't process.

    tip_addendum_coeff is the coefficient controlling how far the profile's
    tip extends past the pitch circle: ADDENDUM_COEFF for an external gear
    (tip reaches outward to the tooth crest), or DEDENDUM_COEFF for an
    annulus/internal cutter (whose "tip" in the same t-parameter sense is the
    cutter reaching outward to the annulus tooth's root).

    The limit is independent of module — only tooth count and pressure angle
    matter — so this can be checked purely from those two operator props.
    """
    pa_rad = radians(pressure_angle_deg)
    ratio  = (1.0 + 2.0 * tip_addendum_coeff / tooth_count) / cos(pa_rad)
    if ratio < 1.0:
        return True  # tip radius inside the base circle — degenerate, not this failure mode

    t_pitch = tan(pa_rad)
    t_tip   = sqrt(ratio * ratio - 1.0)
    angle_pitch = t_pitch - atan(t_pitch)
    angle_tip   = t_tip - atan(t_tip)
    half_tooth_ang = pi / (2.0 * tooth_count)
    return half_tooth_ang > (angle_tip - angle_pitch)


def max_pressure_angle_deg(tooth_count, tip_addendum_coeff, _iterations=60):
    """
    Largest pressure angle (degrees) for which tooth_profile_ok holds — the
    ceiling operators clamp pressure_angle_deg to. No closed form exists, so
    this bisects on the same self-intersection condition.
    """
    lo, hi = 0.0, radians(89.0)
    for _ in range(_iterations):
        mid = (lo + hi) / 2.0
        if tooth_profile_ok(tooth_count, degrees(mid), tip_addendum_coeff):
            lo = mid
        else:
            hi = mid
    return degrees(lo)


def clamp_pressure_angle(op, *tooth_coeff_pairs):
    """
    Clamp op.pressure_angle_deg down to the tightest max_pressure_angle_deg
    across every (tooth_count, tip_addendum_coeff) pair involved — e.g. a
    planetary set passes sun/planet (ADDENDUM_COEFF) and the derived ring
    tooth count (DEDENDUM_COEFF) together, since any one of them self-
    intersecting first is what would break the build. Returns the resulting
    (possibly unchanged) pressure angle for convenience.
    """
    pa_max = min(max_pressure_angle_deg(n, coeff) for n, coeff in tooth_coeff_pairs)
    if op.pressure_angle_deg > pa_max:
        op.pressure_angle_deg = pa_max
    return op.pressure_angle_deg


def stamp_gear(obj, kind, module, pressure_angle_deg,
               tooth_count=None, helix_angle_deg=None, hand=None):
    obj["bmech_kind"]               = kind
    obj["bmech_module"]             = module
    obj["bmech_pressure_angle_deg"] = pressure_angle_deg
    if tooth_count is not None:
        obj["bmech_tooth_count"] = tooth_count
    if helix_angle_deg is not None:
        obj["bmech_helix_angle_deg"] = helix_angle_deg
    if hand is not None:
        obj["bmech_hand"] = hand


def gear_target_poll(_self, obj):
    return obj.type == 'MESH' and "bmech_module" in obj.keys()


def sync_module_pa(op, target):
    """Copy module + pressure angle from target — common to every gear kind."""
    if target is None:
        return
    if "bmech_module" in target.keys():
        op.module = target["bmech_module"]
    if "bmech_pressure_angle_deg" in target.keys():
        op.pressure_angle_deg = target["bmech_pressure_angle_deg"]


def sync_helical_opposite(op, target):
    """
    For external-external meshing pairs (spur/helical/herringbone):
    same module, pressure angle, helix angle — OPPOSITE hand.
    """
    sync_module_pa(op, target)
    if target is None:
        return
    if "bmech_helix_angle_deg" in target.keys() and hasattr(op, "helix_angle_deg"):
        op.helix_angle_deg = target["bmech_helix_angle_deg"]
    if "bmech_hand" in target.keys() and hasattr(op, "hand"):
        op.hand = 'LEFT' if target["bmech_hand"] == 'RIGHT' else 'RIGHT'


def sync_helical_same(op, target):
    """
    For external-internal meshing pairs (pinion <-> annulus):
    same module, pressure angle, helix angle, and SAME hand.
    """
    sync_module_pa(op, target)
    if target is None:
        return
    if "bmech_helix_angle_deg" in target.keys() and hasattr(op, "helix_angle_deg"):
        op.helix_angle_deg = target["bmech_helix_angle_deg"]
    if "bmech_hand" in target.keys() and hasattr(op, "hand"):
        op.hand = target["bmech_hand"]


def sync_bevel(op, target):
    """
    For bevel gear pairs: same module, pressure angle. The target's own
    tooth_count becomes this gear's mate_teeth, so the two cone angles
    stay exactly complementary (delta + delta_mate = 90 deg).
    """
    sync_module_pa(op, target)
    if target is None:
        return
    if "bmech_tooth_count" in target.keys() and hasattr(op, "mate_teeth"):
        op.mate_teeth = target["bmech_tooth_count"]


def _on_target_change(wm, context):
    if wm.bmech_gear_target is None:
        # Only a real target pick should trigger a sync + rebuild. This
        # update callback also fires when reset_target() clears the picker
        # back to empty at the start of every new gear's invoke() — treating
        # THAT as a target change too was a real bug: it re-ran whatever
        # operator context.active_operator still pointed at (the PREVIOUS
        # gear, since the new one's operator hasn't started yet) using the
        # 3D cursor's now-moved position, deleting and rebuilding the
        # previous gear at the new cursor location. There's nothing
        # meaningful to sync from an empty target anyway, so just return.
        return
    op = context.active_operator
    if op is not None and hasattr(op, "bmech_sync_target"):
        # bmech_gear_target lives on WindowManager, not on the operator, so
        # Blender's own redo-panel auto-rerun (which only watches the
        # ACTIVE OPERATOR'S OWN properties for changes) never fires just
        # because this property changed — bmech_sync_target below updates
        # op.module etc. in memory, but the object in the viewport is left
        # showing whatever it looked like before the target was picked.
        #
        # First attempt: bpy.ops.ed.undo() then op.execute(context) again,
        # mirroring what Blender's redo panel does internally when you drag
        # one of the operator's own property widgets. That's unsafe here —
        # ed.undo() operates on the GLOBAL undo stack from inside a
        # property update callback, a known-fragile combination, and
        # testing confirmed it: it deleted the gear being created without
        # rebuilding it.
        #
        # This is more targeted: every gear operator in this family ends
        # its own execute() by deselecting everything else and selecting
        # + activating exactly what it just built (`select_all(DESELECT)`,
        # `obj.select_set(True)`, `objects.active = obj`) — so whatever is
        # currently selected/active IS that operator's own prior result,
        # nothing else, as long as nothing else has been clicked in the
        # viewport since (the normal redo-panel workflow: adjust properties
        # immediately after creation). Delete exactly that, then re-run
        # execute() to rebuild in place instead of leaving stale geometry
        # or creating a duplicate.
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
    """
    Clear the Match Target picker. bmech_gear_target lives on WindowManager
    (see the module docstring for why), which means its value otherwise
    persists across unrelated operator invocations within the same
    session — pick a target for one gear, and the NEXT, unrelated gear you
    create would silently start pre-matched to it too.

    Call this from each gear operator's invoke() specifically, not
    execute(): invoke() only runs once, when the operator starts fresh
    (e.g. from the Add menu), while execute() also re-runs on every
    redo-panel property tweak — resetting there would wipe out the user's
    own target selection the moment they adjusted any other property.
    """
    context.window_manager.bmech_gear_target = None


def register():
    bpy.types.WindowManager.bmech_gear_target = PointerProperty(
        name="Match Target", type=bpy.types.Object,
        poll=gear_target_poll, update=_on_target_change,
        description="Pick an existing gear to copy module & pressure angle from",
    )


def unregister():
    del bpy.types.WindowManager.bmech_gear_target
