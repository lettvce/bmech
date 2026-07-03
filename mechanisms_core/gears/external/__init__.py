import importlib

from . import involute_gear_rack
from . import cluster_gear
from . import compound_gear
from . import helical_gear
from . import herringbone_gear

if "bpy" in locals():
    importlib.reload(involute_gear_rack)
    importlib.reload(cluster_gear)
    importlib.reload(compound_gear)
    importlib.reload(helical_gear)
    importlib.reload(herringbone_gear)

import bpy


def register():
    involute_gear_rack.register()
    cluster_gear.register()
    compound_gear.register()
    helical_gear.register()
    herringbone_gear.register()


def unregister():
    herringbone_gear.unregister()
    helical_gear.unregister()
    compound_gear.unregister()
    cluster_gear.unregister()
    involute_gear_rack.unregister()
