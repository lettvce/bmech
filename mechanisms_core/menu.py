# mechanisms_core/menu.py
#
# Gear submenu hierarchy:
#   Mechanisms → Gears → External   (spur, cluster, compound, helical, herringbone)
#                      → Ring       (annulus, helical annulus, herringbone annulus)
#                      → Planetary  (spur, helical, herringbone planetary sets)
#                      → Bevel      (straight bevel — helical/spiral bevel planned)
#                      → Rack       (straight, helical, herringbone)
#                        Crown Gear
#                        Sprocket
#            → Fasteners (hex bolt, hex nut, raw threaded fastener)
#
# All entries are guarded by hasattr so a missing/unregistered operator is
# simply invisible — no error on load.  Add one layout.operator() line when a
# new generator graduates; no other file needs touching.

import bpy


# ── Ratchet submenu ───────────────────────────────────────────────────────────

class VIEW3D_MT_mechanisms_ratchet(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_ratchet"
    bl_label  = "Ratchet"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_add_ratchet_mechanism'):
            layout.operator("object.add_ratchet_mechanism", text="External Ratchet & Pawl")
        if hasattr(bpy.types, 'OBJECT_OT_add_internal_ratchet'):
            layout.operator("object.add_internal_ratchet",  text="Internal Freewheel Ratchet")


# ── Gear submenus ─────────────────────────────────────────────────────────────

class VIEW3D_MT_mechanisms_gears_external(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears_external"
    bl_label  = "External"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_add_spur_gear'):
            layout.operator("object.add_spur_gear",      text="Spur Gear")
        if hasattr(bpy.types, 'OBJECT_OT_add_cluster_gear'):
            layout.operator("object.add_cluster_gear",   text="Cluster Gear")
        if hasattr(bpy.types, 'OBJECT_OT_add_compound_gear'):
            layout.operator("object.add_compound_gear",  text="Compound Gear")
        if hasattr(bpy.types, 'OBJECT_OT_helical_gear') or hasattr(bpy.types, 'OBJECT_OT_herringbone_gear'):
            layout.separator()
        if hasattr(bpy.types, 'OBJECT_OT_helical_gear'):
            layout.operator("object.helical_gear",       text="Helical Gear")
        if hasattr(bpy.types, 'OBJECT_OT_herringbone_gear'):
            layout.operator("object.herringbone_gear",   text="Herringbone Gear")


class VIEW3D_MT_mechanisms_gears_ring(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears_ring"
    bl_label  = "Ring"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_annulus_gear'):
            layout.operator("object.annulus_gear",              text="Annulus Gear")
        if hasattr(bpy.types, 'OBJECT_OT_helical_annulus_gear'):
            layout.operator("object.helical_annulus_gear",      text="Helical Annulus")
        if hasattr(bpy.types, 'OBJECT_OT_herringbone_annulus_gear'):
            layout.operator("object.herringbone_annulus_gear",  text="Herringbone Annulus")


class VIEW3D_MT_mechanisms_gears_planetary(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears_planetary"
    bl_label  = "Planetary"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_planetary_gear_set'):
            layout.operator("object.planetary_gear_set",              text="Planetary Gear Set")
        if hasattr(bpy.types, 'OBJECT_OT_helical_planetary_gear_set'):
            layout.operator("object.helical_planetary_gear_set",      text="Helical Planetary")
        if hasattr(bpy.types, 'OBJECT_OT_herringbone_planetary_gear_set'):
            layout.operator("object.herringbone_planetary_gear_set",  text="Herringbone Planetary")


class VIEW3D_MT_mechanisms_gears_bevel(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears_bevel"
    bl_label  = "Bevel"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_bevel_gear'):
            layout.operator("object.bevel_gear", text="Bevel Gear")


class VIEW3D_MT_mechanisms_gears_rack(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears_rack"
    bl_label  = "Rack"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_add_rack'):
            layout.operator("object.add_rack",             text="Straight Rack")
        if hasattr(bpy.types, 'OBJECT_OT_helical_rack'):
            layout.operator("object.helical_rack",         text="Helical Rack")
        if hasattr(bpy.types, 'OBJECT_OT_herringbone_rack'):
            layout.operator("object.herringbone_rack",     text="Herringbone Rack")


class VIEW3D_MT_mechanisms_gears(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_gears"
    bl_label  = "Gears"

    def draw(self, context):
        layout = self.layout

        external_any = (hasattr(bpy.types, 'OBJECT_OT_add_spur_gear')     or
                        hasattr(bpy.types, 'OBJECT_OT_add_cluster_gear')   or
                        hasattr(bpy.types, 'OBJECT_OT_add_compound_gear')  or
                        hasattr(bpy.types, 'OBJECT_OT_helical_gear')       or
                        hasattr(bpy.types, 'OBJECT_OT_herringbone_gear'))
        if external_any:
            layout.menu(VIEW3D_MT_mechanisms_gears_external.bl_idname)

        ring_any = (hasattr(bpy.types, 'OBJECT_OT_annulus_gear')              or
                    hasattr(bpy.types, 'OBJECT_OT_helical_annulus_gear')       or
                    hasattr(bpy.types, 'OBJECT_OT_herringbone_annulus_gear'))
        if ring_any:
            layout.menu(VIEW3D_MT_mechanisms_gears_ring.bl_idname)

        planetary_any = (hasattr(bpy.types, 'OBJECT_OT_planetary_gear_set')             or
                         hasattr(bpy.types, 'OBJECT_OT_helical_planetary_gear_set')     or
                         hasattr(bpy.types, 'OBJECT_OT_herringbone_planetary_gear_set'))
        if planetary_any:
            layout.menu(VIEW3D_MT_mechanisms_gears_planetary.bl_idname)

        bevel_any = hasattr(bpy.types, 'OBJECT_OT_bevel_gear')
        if bevel_any:
            layout.menu(VIEW3D_MT_mechanisms_gears_bevel.bl_idname)

        rack_any = (hasattr(bpy.types, 'OBJECT_OT_add_rack')          or
                    hasattr(bpy.types, 'OBJECT_OT_helical_rack')      or
                    hasattr(bpy.types, 'OBJECT_OT_herringbone_rack'))
        if rack_any:
            layout.menu(VIEW3D_MT_mechanisms_gears_rack.bl_idname)

        special_any = (hasattr(bpy.types, 'OBJECT_OT_crown_gear') or
                       hasattr(bpy.types, 'OBJECT_OT_add_sprocket'))
        if special_any and (external_any or ring_any or planetary_any or bevel_any or rack_any):
            layout.separator()
        if hasattr(bpy.types, 'OBJECT_OT_crown_gear'):
            layout.operator("object.crown_gear",   text="Crown Gear")
        if hasattr(bpy.types, 'OBJECT_OT_add_sprocket'):
            layout.operator("object.add_sprocket", text="Sprocket")


# ── Fasteners submenu ─────────────────────────────────────────────────────────

class VIEW3D_MT_mechanisms_fasteners(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_fasteners"
    bl_label  = "Fasteners"

    def draw(self, context):
        layout = self.layout
        if hasattr(bpy.types, 'OBJECT_OT_hex_bolt'):
            layout.operator("object.hex_bolt",             text="Hex Bolt")
        if hasattr(bpy.types, 'OBJECT_OT_hex_nut'):
            layout.operator("object.hex_nut",               text="Hex Nut")
        if hasattr(bpy.types, 'OBJECT_OT_add_threaded_fastener'):
            layout.operator("object.add_threaded_fastener", text="Threaded Fastener (raw)")
        if hasattr(bpy.types, 'OBJECT_OT_threaded_container') or hasattr(bpy.types, 'OBJECT_OT_threaded_lid'):
            layout.separator()
        if hasattr(bpy.types, 'OBJECT_OT_threaded_container'):
            layout.operator("object.threaded_container",    text="Threaded Container")
        if hasattr(bpy.types, 'OBJECT_OT_threaded_lid'):
            layout.operator("object.threaded_lid",           text="Threaded Lid")


# ── Top-level Mechanisms menu ─────────────────────────────────────────────────

class VIEW3D_MT_mechanisms_add(bpy.types.Menu):
    bl_idname = "VIEW3D_MT_mechanisms_add"
    bl_label  = "Mechanisms"

    def draw(self, context):
        layout    = self.layout
        found_any = False

        if hasattr(bpy.types, 'OBJECT_OT_add_hairspring'):
            layout.operator("object.add_hairspring",       text="Hairspring")
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_serpentine_spring'):
            layout.operator("object.add_serpentine_spring", text="Serpentine Spring")
            found_any = True

        ratchet_any = (hasattr(bpy.types, 'OBJECT_OT_add_ratchet_mechanism') or
                       hasattr(bpy.types, 'OBJECT_OT_add_internal_ratchet'))
        if ratchet_any:
            layout.menu(VIEW3D_MT_mechanisms_ratchet.bl_idname)
            found_any = True

        gears_any = (hasattr(bpy.types, 'OBJECT_OT_add_spur_gear')                      or
                     hasattr(bpy.types, 'OBJECT_OT_add_rack')                            or
                     hasattr(bpy.types, 'OBJECT_OT_add_cluster_gear')                    or
                     hasattr(bpy.types, 'OBJECT_OT_add_compound_gear')                   or
                     hasattr(bpy.types, 'OBJECT_OT_helical_gear')                        or
                     hasattr(bpy.types, 'OBJECT_OT_herringbone_gear')                    or
                     hasattr(bpy.types, 'OBJECT_OT_annulus_gear')                        or
                     hasattr(bpy.types, 'OBJECT_OT_helical_annulus_gear')                or
                     hasattr(bpy.types, 'OBJECT_OT_herringbone_annulus_gear')            or
                     hasattr(bpy.types, 'OBJECT_OT_planetary_gear_set')                  or
                     hasattr(bpy.types, 'OBJECT_OT_helical_planetary_gear_set')          or
                     hasattr(bpy.types, 'OBJECT_OT_herringbone_planetary_gear_set')      or
                     hasattr(bpy.types, 'OBJECT_OT_bevel_gear')                          or
                     hasattr(bpy.types, 'OBJECT_OT_helical_rack')                        or
                     hasattr(bpy.types, 'OBJECT_OT_herringbone_rack')                    or
                     hasattr(bpy.types, 'OBJECT_OT_crown_gear')                          or
                     hasattr(bpy.types, 'OBJECT_OT_add_sprocket'))
        if gears_any:
            layout.menu(VIEW3D_MT_mechanisms_gears.bl_idname)
            found_any = True

        if hasattr(bpy.types, 'OBJECT_OT_add_ball_bearing'):
            layout.operator("object.add_ball_bearing",     text="Ball Bearing")
            found_any = True

        fasteners_any = (hasattr(bpy.types, 'OBJECT_OT_hex_bolt')                or
                         hasattr(bpy.types, 'OBJECT_OT_hex_nut')                 or
                         hasattr(bpy.types, 'OBJECT_OT_add_threaded_fastener'))
        if fasteners_any:
            layout.menu(VIEW3D_MT_mechanisms_fasteners.bl_idname)
            found_any = True

        if not found_any:
            layout.label(text="No generators enabled", icon='INFO')


# ── Hook into Shift-A ─────────────────────────────────────────────────────────

def menu_draw(self, context):
    self.layout.menu(VIEW3D_MT_mechanisms_add.bl_idname)


classes = (
    VIEW3D_MT_mechanisms_ratchet,
    VIEW3D_MT_mechanisms_gears_external,
    VIEW3D_MT_mechanisms_gears_ring,
    VIEW3D_MT_mechanisms_gears_planetary,
    VIEW3D_MT_mechanisms_gears_bevel,
    VIEW3D_MT_mechanisms_gears_rack,
    VIEW3D_MT_mechanisms_gears,
    VIEW3D_MT_mechanisms_fasteners,
    VIEW3D_MT_mechanisms_add,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_add.prepend(menu_draw)


def unregister():
    bpy.types.VIEW3D_MT_add.remove(menu_draw)
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
