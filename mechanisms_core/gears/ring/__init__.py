import importlib

from . import annulus_gear
from . import helical_annulus_gear
from . import herringbone_annulus_gear

if "bpy" in locals():
    importlib.reload(annulus_gear)
    importlib.reload(helical_annulus_gear)
    importlib.reload(herringbone_annulus_gear)

import bpy


def register():
    annulus_gear.register()
    helical_annulus_gear.register()
    herringbone_annulus_gear.register()


def unregister():
    herringbone_annulus_gear.unregister()
    helical_annulus_gear.unregister()
    annulus_gear.unregister()
