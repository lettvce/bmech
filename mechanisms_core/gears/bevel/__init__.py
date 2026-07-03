import importlib

from . import bevel_gear

if "bpy" in locals():
    importlib.reload(bevel_gear)

import bpy


def register():
    bevel_gear.register()


def unregister():
    bevel_gear.unregister()
