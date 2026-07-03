import importlib

from . import menu
from . import gears
from . import fasteners
from . import bearings
from . import springs
from . import ratchets

if "bpy" in locals():
    importlib.reload(gears)
    importlib.reload(fasteners)
    importlib.reload(bearings)
    importlib.reload(springs)
    importlib.reload(ratchets)
    importlib.reload(menu)

import bpy


def register():
    springs.register()
    ratchets.register()
    fasteners.register()
    gears.register()
    bearings.register()
    menu.register()


def unregister():
    menu.unregister()
    bearings.unregister()
    gears.unregister()
    fasteners.unregister()
    ratchets.unregister()
    springs.unregister()


if __name__ == "__main__":
    register()
