"""
Mechanisms Reference Mesh Exporter — Spec v1.0
────────────────────────────────────────────────
Standalone Blender 5.1 add-on. Exports the active mesh object's vertices,
edges, and faces to a structured CSV reference file for an AI code-generation
workflow (a separate agent reads the CSV as context when writing parametric
bmesh generators).

This is a data-export utility, not a Mechanisms Library generator — it does
NOT belong in the Shift-A -> Add -> Mechanisms submenu and is not part of the
bmech core/registry pattern. It lives under File -> Export instead.

Run with Alt+P for a quick test (opens the export file browser immediately);
for real use, install as a standalone add-on so it persists under
File -> Export -> Mechanisms Reference Mesh (.csv).

Fixed behavior (not user-configurable in v1.0, see spec §4):
  - Active object only, no multi-object export
  - Local/object space coordinates (matrix_world NOT applied)
  - Evaluated mesh via depsgraph (modifiers applied)
  - Native face topology (no triangulation) — tris/quads/n-gons as-authored
  - Face normals from polygon.normal (geometry-derived, not custom split normals)

Flagged decision (spec §6.4, §8.1): FACE rows have variable trailing-column
counts (n-gon vertex lists), so this CSV is intentionally ragged rather than
fixed-width. The `type` field plus (for FACE) the explicit vert_count field
make every row unambiguously parseable positionally without a fixed header.
"""

bl_info = {
    "name": "Mechanisms Reference Exporter",
    "author": "",
    "version": (1, 0),
    "blender": (5, 1, 0),
    "location": "File > Export > Mechanisms Reference Mesh (.csv)",
    "description": "Export a mesh's vertices, edges, and faces as a structured CSV reference file",
    "category": "Import-Export",
}

import bpy
import csv
import datetime
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, IntProperty


# ── Operator ──────────────────────────────────────────────────────────────────

class EXPORT_MESH_OT_mechanisms_reference_csv(bpy.types.Operator, ExportHelper):
    """Export the active mesh object's vertices, edges, and faces as a structured CSV reference file"""
    bl_idname  = "export_mesh.mechanisms_reference_csv"
    bl_label   = "Export Mechanisms Reference Mesh (CSV)"
    bl_options = {'REGISTER'}

    filename_ext = ".csv"
    filter_glob: StringProperty(default="*.csv", options={'HIDDEN'})

    decimal_precision: IntProperty(
        name="Decimal Precision",
        description="Rounding precision for all float output (vertex coordinates, normal components)",
        default=4, min=1, max=10,
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "decimal_precision")

    def execute(self, context):
        # ── Validation (spec §7) — object selected -> is mesh -> not edit mode -> vertex count > 0
        obj = context.active_object
        if obj is None:
            self.report({'ERROR'}, "No object selected. Select a mesh object to export.")
            return {'CANCELLED'}
        if obj.type != 'MESH':
            self.report({'ERROR'},
                "Active object '%s' is not a mesh. Select a mesh object and try again." % obj.name)
            return {'CANCELLED'}
        if obj.mode == 'EDIT':
            self.report({'ERROR'}, "Cannot export while in Edit Mode. Switch to Object Mode and try again.")
            return {'CANCELLED'}
        if len(obj.data.vertices) == 0:
            self.report({'ERROR'}, "Mesh '%s' has no vertices. Nothing to export." % obj.name)
            return {'CANCELLED'}

        dp = self.decimal_precision

        # ── Evaluated mesh (spec §4) ──────────────────────────────────────────
        depsgraph = context.evaluated_depsgraph_get()
        eval_obj  = obj.evaluated_get(depsgraph)
        mesh      = None
        try:
            mesh = eval_obj.to_mesh()

            if len(mesh.vertices) == 0:
                self.report({'ERROR'}, "Mesh '%s' has no vertices. Nothing to export." % obj.name)
                return {'CANCELLED'}

            vert_rows = []
            for v in mesh.vertices:
                co = v.co
                vert_rows.append([
                    "VERT", v.index,
                    round(co.x, dp), round(co.y, dp), round(co.z, dp),
                ])

            edge_rows = []
            for e in mesh.edges:
                va, vb = e.vertices[0], e.vertices[1]
                edge_rows.append(["EDGE", e.index, va, vb])

            face_rows = []
            for p in mesh.polygons:
                n   = p.normal
                row = ["FACE", p.index, len(p.vertices)]
                row.extend(list(p.vertices))
                row.extend([round(n.x, dp), round(n.y, dp), round(n.z, dp)])
                face_rows.append(row)

            vert_count = len(vert_rows)
            edge_count = len(edge_rows)
            face_count = len(face_rows)
        finally:
            eval_obj.to_mesh_clear()

        # ── Write CSV (spec §5.e, §6) ──────────────────────────────────────────
        timestamp = datetime.datetime.now().isoformat(timespec='seconds')

        try:
            with open(self.filepath, 'w', newline='') as f:
                f.write("# Mechanisms Reference Mesh Export\n")
                f.write("# Object: %s\n" % obj.name)
                f.write("# Blender Version: %s\n" % bpy.app.version_string)
                f.write("# Export Timestamp: %s\n" % timestamp)
                f.write("# Units: millimeters (1 BU = 1 mm)\n")
                f.write("# Coordinate Space: Local (object space, modifiers applied, matrix_world NOT applied)\n")
                f.write("# Mesh Source: Evaluated (depsgraph, modifiers applied)\n")
                f.write("# Vertex Count: %d\n" % vert_count)
                f.write("# Edge Count: %d\n" % edge_count)
                f.write("# Face Count: %d\n" % face_count)
                f.write("# Decimal Precision: %d\n" % dp)
                f.write("#\n")
                f.write("# Row schema:\n")
                f.write("# VERT,<index>,<x>,<y>,<z>\n")
                f.write("# EDGE,<index>,<vert_index_a>,<vert_index_b>\n")
                f.write("# FACE,<index>,<vert_count>,<v0>,<v1>,...,<v(n-1)>,<normal_x>,<normal_y>,<normal_z>\n")

                writer = csv.writer(f)
                for row in vert_rows:
                    writer.writerow(row)
                for row in edge_rows:
                    writer.writerow(row)
                for row in face_rows:
                    writer.writerow(row)
        except OSError as exc:
            self.report({'ERROR'}, "Failed to write file: %s" % str(exc))
            return {'CANCELLED'}

        self.report({'INFO'},
            "Exported %d verts, %d edges, %d faces to %s"
            % (vert_count, edge_count, face_count, self.filepath))
        return {'FINISHED'}


# ── Menu ──────────────────────────────────────────────────────────────────────

def menu_func_export(self, context):
    self.layout.operator(EXPORT_MESH_OT_mechanisms_reference_csv.bl_idname,
                          text="Mechanisms Reference Mesh (.csv)")


# ── Registration ──────────────────────────────────────────────────────────────

def register():
    bpy.utils.register_class(EXPORT_MESH_OT_mechanisms_reference_csv)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)
    bpy.utils.unregister_class(EXPORT_MESH_OT_mechanisms_reference_csv)


# ── Register and run ──────────────────────────────────────────────────────────
try:
    unregister()
except Exception:
    pass
register()
bpy.ops.export_mesh.mechanisms_reference_csv('INVOKE_DEFAULT')
