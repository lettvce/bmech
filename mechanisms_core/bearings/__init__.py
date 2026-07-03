import importlib

from . import ball_bearing

if "bpy" in locals():
    importlib.reload(ball_bearing)

import bpy


def register():
    ball_bearing.register()


def unregister():
    ball_bearing.unregister()
