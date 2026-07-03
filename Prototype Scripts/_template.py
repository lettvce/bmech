"""
Prototype Script Template
─────────────────────────
Paste into Blender's Text Editor and hit Run Script (Alt+P).

Parameters are editable live in the Redo panel (bottom-left of viewport after
running, or press F9). Every property change triggers a rebuild automatically.

Graduating to mechanisms_core:
  1. Copy the operator class into mechanisms_core/<name>.py
  2. Delete the last two lines (register() + bpy.ops.*)
  3. Add the standard classes/register/unregister block
  4. Wire into __init__.py and menu.py
  The operator class itself does not need to change.
"""

import bpy
import bmesh
from bpy.props import FloatProperty, IntProperty, FloatVectorProperty
from math import pi, cos, sin


class OBJECT_OT_prototype_thing(bpy.types.Operator):
    """One-line tooltip shown on hover."""
    bl_idname  = "object.prototype_thing"   # must be unique — change per script
    bl_label   = "Prototype Thing"
    bl_options = {'REGISTER', 'UNDO'}

    # ── Properties — these become the redo-panel sliders ──────────────────────
    param_a: FloatProperty(name="Param A (mm)", default=10.0, min=0.1, soft_max=100.0)
    param_b: IntProperty(  name="Param B",      default=6,    min=3,   soft_max=100)

    # ── invoke() — shows popup dialog before building; delete when graduating ───
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=300)

    # ── draw() — inline validation goes here (copy into mechanisms_core as-is) ─
    def draw(self, context):
        layout = self.layout
        layout.prop(self, "param_a")
        layout.prop(self, "param_b")

        # Example inline error (uncomment and adapt):
        # if self.param_a < self.param_b:
        #     layout.label(text="Param A must exceed Param B", icon='ERROR')

    # ── execute() — build the mesh ────────────────────────────────────────────
    def execute(self, context):
        bm = bmesh.new()

        # build geometry using self.param_a, self.param_b ...

        me = bpy.data.meshes.new("PrototypeMesh")
        bm.to_mesh(me)
        bm.free()
        me.update()

        obj = bpy.data.objects.new("Prototype", me)
        context.collection.objects.link(obj)
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        context.view_layer.objects.active = obj

        return {'FINISHED'}


# ── These lines are deleted when graduating to mechanisms_core ────────────────
try:
    bpy.utils.unregister_class(OBJECT_OT_prototype_thing)
except Exception:
    pass
bpy.utils.register_class(OBJECT_OT_prototype_thing)
bpy.ops.object.prototype_thing('INVOKE_DEFAULT')
