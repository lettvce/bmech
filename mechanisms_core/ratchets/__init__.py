import importlib

from . import ratchet_pawl
from . import internal_ratchet

if "bpy" in locals():
    importlib.reload(ratchet_pawl)
    importlib.reload(internal_ratchet)

import bpy


def register():
    ratchet_pawl.register()
    internal_ratchet.register()


def unregister():
    internal_ratchet.unregister()
    ratchet_pawl.unregister()
