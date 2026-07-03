import importlib

from . import hairspring
from . import serpentine_spring

if "bpy" in locals():
    importlib.reload(hairspring)
    importlib.reload(serpentine_spring)

import bpy


def register():
    hairspring.register()
    serpentine_spring.register()


def unregister():
    serpentine_spring.unregister()
    hairspring.unregister()
