import importlib
from . import straight_rack
from . import helical_rack
from . import herringbone_rack

if "bpy" in locals():
    importlib.reload(straight_rack)
    importlib.reload(helical_rack)
    importlib.reload(herringbone_rack)

import bpy


def register():
    straight_rack.register()
    helical_rack.register()
    herringbone_rack.register()


def unregister():
    herringbone_rack.unregister()
    helical_rack.unregister()
    straight_rack.unregister()
