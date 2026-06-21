import importlib

from . import menu
from . import hairspring
from . import serpentine_spring
from . import ratchet_pawl
from . import internal_ratchet
from . import press_fit_pin
from . import involute_gear_rack
from . import ball_bearing

if "bpy" in locals():
    importlib.reload(hairspring)
    importlib.reload(serpentine_spring)
    importlib.reload(ratchet_pawl)
    importlib.reload(internal_ratchet)
    importlib.reload(press_fit_pin)
    importlib.reload(involute_gear_rack)
    importlib.reload(ball_bearing)
    importlib.reload(menu)

import bpy


def register():
    hairspring.register()
    serpentine_spring.register()
    ratchet_pawl.register()
    internal_ratchet.register()
    press_fit_pin.register()
    involute_gear_rack.register()
    ball_bearing.register()
    menu.register()


def unregister():
    menu.unregister()
    ball_bearing.unregister()
    involute_gear_rack.unregister()
    press_fit_pin.unregister()
    internal_ratchet.unregister()
    ratchet_pawl.unregister()
    serpentine_spring.unregister()
    hairspring.unregister()


if __name__ == "__main__":
    register()
