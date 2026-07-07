import importlib

from . import fastener_matching
from . import threaded_fastener
from . import hex_bolt
from . import hex_nut
from . import threaded_container
from . import threaded_lid
from . import press_fit_pin

if "bpy" in locals():
    importlib.reload(fastener_matching)
    importlib.reload(threaded_fastener)
    importlib.reload(hex_bolt)
    importlib.reload(hex_nut)
    importlib.reload(threaded_container)
    importlib.reload(threaded_lid)
    importlib.reload(press_fit_pin)

import bpy


def register():
    fastener_matching.register()
    threaded_fastener.register()
    hex_bolt.register()
    hex_nut.register()
    threaded_container.register()
    threaded_lid.register()
    press_fit_pin.register()


def unregister():
    press_fit_pin.unregister()
    threaded_lid.unregister()
    threaded_container.unregister()
    hex_nut.unregister()
    hex_bolt.unregister()
    threaded_fastener.unregister()
    fastener_matching.unregister()
