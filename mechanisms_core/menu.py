# mechanisms_core/menu.py
#
# ---------------------------------------------------------------------------
# ARCHITECTURE DECISION (spec §2.4 / §5): the spec explicitly flagged
# hardcoded-vs-dynamic-registry as an open question and asked me to confirm
# before implementing. Per my instructions I'm not allowed to stop and ask,
# so I went with the spec's own stated recommendation: HARDCODED. You're a
# one-person shop adding generators a few times a quarter, not shipping this
# to strangers -- the registry pattern in §2.3 buys you decoupling you don't
# need yet at the cost of enable-order footguns you definitely don't want.
#
# Practical consequence: every time a new generator graduates to "ready for
# menu," you add ONE layout.operator(...) line below by hand. That's it.
# No register_mechanism()/unregister_mechanism() calls needed in any other
# addon's __init__.py. If this list ever creeps past ~15-20 entries, or you
# decide to redistribute the suite to other humans, that's your signal to
# go build the §2.3 registry for real -- don't do it preemptively.
#
# OTHER OPEN ASSUMPTIONS FROM THE SPEC I COULDN'T CONFIRM (§5):
#   1. Menu label "Mechanisms" -- kept as-is, it's a one-string change
#      (bl_label below) if you want "FDM Parts" or "Parametric" instead.
#   2. Operator idnames -- the spec never gave me the actual bl_idname for
#      each generator's operator, so the strings below are my best guess at
#      your naming convention (object.add_<thing>). VERIFY THESE against
#      each generator's real operator class before trusting this menu --
#      a wrong idname just gives Blender a "not found" or quietly does
#      nothing, it won't break registration of this addon.
#   3. Shared common.py/utils.py -- spec asks to confirm none already
#      exists across your generator addons before building this as a new
#      addon. I have no visibility into your other addons' folders, so
#      proceeding per spec's literal instruction (build mechanisms_core
#      fresh). If a shared module already exists, this menu logic could
#      probably live there instead and you can fold this file in.
# ---------------------------------------------------------------------------

import bpy


class VIEW3D_MT_mechanisms_ratchet(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_ratchet"
    bl_label  = "Ratchet"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_add_ratchet_mechanism'):
            layout.operator("object.add_ratchet_mechanism", text="External Ratchet & Pawl")
        if hasattr(bpy.types, 'OBJECT_OT_add_internal_ratchet'):
            layout.operator("object.add_internal_ratchet",  text="Internal Freewheel Ratchet")


class VIEW3D_MT_mechanisms_add(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_add"
    bl_label  = "Mechanisms"

    def draw(self, context):
        layout    = self.layout
        found_any = False

        if hasattr(bpy.types, 'OBJECT_OT_add_hairspring'):
            layout.operator("object.add_hairspring", text="Hairspring")
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_serpentine_spring'):
            layout.operator("object.add_serpentine_spring", text="Serpentine Spring")
            found_any = True

        ratchet_any = (hasattr(bpy.types, 'OBJECT_OT_add_ratchet_mechanism') or
                       hasattr(bpy.types, 'OBJECT_OT_add_internal_ratchet'))
        if ratchet_any:
            layout.menu(VIEW3D_MT_mechanisms_ratchet.bl_idname)
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_press_pin'):
            layout.operator("object.add_press_pin", text="Press-Fit Pin")
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_spur_gear'):
            layout.operator("object.add_spur_gear", text="Involute Gear")
            layout.operator("object.add_rack",      text="Gear Rack")
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_ball_bearing'):
            layout.operator("object.add_ball_bearing", text="Ball Bearing")
            found_any = True

        if not found_any:
            layout.label(text="No generators enabled", icon='INFO')


def menu_draw(self, context):
    self.layout.menu(VIEW3D_MT_mechanisms_add.bl_idname)


classes = (
    VIEW3D_MT_mechanisms_ratchet,
    VIEW3D_MT_mechanisms_add,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    # Hook into the built-in Shift-A Add menu (top level, alongside Mesh/Curve/Volume).
    bpy.types.VIEW3D_MT_add.prepend(menu_draw)


def unregister():
    # Detach from the Add menu first -- skip this and re-enabling the addon
    # gives you a delightful stack of duplicate "Mechanisms" entries.
    bpy.types.VIEW3D_MT_add.remove(menu_draw)

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
