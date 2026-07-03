"""
Mechanisms Reloader — dev convenience.

Disables Mechanisms Core, purges its cached submodules from sys.modules, and
re-enables it. Blender's addon_enable() reuses already-imported modules by
default, so a plain disable/enable can silently keep running stale code —
purging sys.modules first forces a genuine re-import from disk, equivalent to
a full Blender restart for this one extension.

Lives as its own extension (not inside mechanisms_core) so reloading the
target doesn't also tear down the operator doing the reloading.
"""

import bpy
import sys

TARGET_MODULE = "bl_ext.user_default.mechanisms_core"


class WM_OT_reload_mechanisms_core(bpy.types.Operator):
    """Disable, purge cached modules, and re-enable Mechanisms Core"""
    bl_idname  = "wm.reload_mechanisms_core"
    bl_label   = "Reload Mechanisms Core"
    bl_options = {'REGISTER'}

    def execute(self, context):
        try:
            bpy.ops.preferences.addon_disable(module=TARGET_MODULE)
        except Exception as e:
            self.report({'WARNING'}, "Disable step failed: %s" % e)

        purged = [name for name in list(sys.modules)
                  if name == TARGET_MODULE or name.startswith(TARGET_MODULE + ".")]
        for name in purged:
            del sys.modules[name]

        def _reenable():
            try:
                bpy.ops.preferences.addon_enable(module=TARGET_MODULE)
            except Exception as e:
                print("Mechanisms Reloader: re-enable failed:", e)
            return None

        bpy.app.timers.register(_reenable, first_interval=0.05)
        self.report({'INFO'}, "Reloading Mechanisms Core (%d modules purged)..." % len(purged))
        return {'FINISHED'}


classes = (WM_OT_reload_mechanisms_core,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
