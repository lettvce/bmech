import importlib

from . import gear_matching
from . import external
from . import ring
from . import planetary
from . import bevel
from . import rack

if "bpy" in locals():
    importlib.reload(gear_matching)
    importlib.reload(external)
    importlib.reload(ring)
    importlib.reload(planetary)
    importlib.reload(bevel)
    importlib.reload(rack)

import bpy


def register():
    gear_matching.register()
    external.register()
    ring.register()
    planetary.register()
    bevel.register()
    rack.register()


def unregister():
    rack.unregister()
    bevel.unregister()
    planetary.unregister()
    ring.unregister()
    external.unregister()
    gear_matching.unregister()
