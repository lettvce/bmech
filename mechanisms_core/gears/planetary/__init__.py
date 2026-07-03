import importlib

from . import planetary_gear_set
from . import helical_planetary_gear_set
from . import herringbone_planetary_gear_set

if "bpy" in locals():
    importlib.reload(planetary_gear_set)
    importlib.reload(helical_planetary_gear_set)
    importlib.reload(herringbone_planetary_gear_set)

import bpy


def register():
    planetary_gear_set.register()
    helical_planetary_gear_set.register()
    herringbone_planetary_gear_set.register()


def unregister():
    herringbone_planetary_gear_set.unregister()
    helical_planetary_gear_set.unregister()
    planetary_gear_set.unregister()
