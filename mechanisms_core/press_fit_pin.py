# press_fit_pin/press_fit_pin.py
# Blender 5.1 Add-on: Press-Fit Pin + Hole Cutter Generator
# Generates a tapered press-fit pin and a matching undersized hole-cutter mesh
# for manual Boolean Difference workflows. All values in mm.

import bpy
import bmesh
from math import radians, tan, cos, sin, pi
from mathutils import Vector


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

NOZZLE_WIDTH_MM = 0.4

PRESS_FIT_INTERFERENCE_PRESETS_MM = {
    'LIGHT':  0.10,
    'MEDIUM': 0.20,
    'TIGHT':  0.30,
}


# ---------------------------------------------------------------------------
# Derived values
# ---------------------------------------------------------------------------

def compute_press_fit_diameters(nominal_diameter_mm, interference_mm,
                                 pin_diameter_compensation_mm,
                                 hole_diameter_compensation_mm):
    """Returns (pin_diameter_mm, hole_diameter_mm).

    Interference is split symmetrically: half added to the pin, half
    subtracted from the hole, then printer-bias compensation on top.
    """
    half_i           = interference_mm / 2.0
    pin_diameter_mm  = nominal_diameter_mm + half_i - pin_diameter_compensation_mm
    hole_diameter_mm = nominal_diameter_mm - half_i + hole_diameter_compensation_mm
    return pin_diameter_mm, hole_diameter_mm


def compute_tip_diameter(pin_diameter_mm, taper_length_mm, taper_angle_deg):
    return pin_diameter_mm - 2.0 * taper_length_mm * tan(radians(taper_angle_deg))


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def validate_press_pin_parameters(nominal_diameter_mm, interference_mm,
                                   pin_diameter_compensation_mm,
                                   hole_diameter_compensation_mm,
                                   pin_length_mm, taper_length_mm,
                                   taper_angle_deg, radial_segments):
    """Returns a list of error strings. Empty = all clear."""
    errors = []

    pin_diameter_mm, hole_diameter_mm = compute_press_fit_diameters(
        nominal_diameter_mm, interference_mm,
        pin_diameter_compensation_mm, hole_diameter_compensation_mm,
    )
    tip_diameter_mm = compute_tip_diameter(pin_diameter_mm, taper_length_mm, taper_angle_deg)

    if not (interference_mm > 0):
        errors.append("Interference must be greater than zero for a press fit (got %.3f mm)." % interference_mm)
    if not (hole_diameter_mm > 0):
        errors.append(
            "Computed hole diameter (%.3f mm) is zero or negative — "
            "reduce interference or compensation, or increase nominal diameter." % hole_diameter_mm
        )
    if not (pin_diameter_mm > 0):
        errors.append(
            "Computed pin diameter (%.3f mm) is zero or negative — "
            "reduce pin compensation or increase nominal diameter." % pin_diameter_mm
        )
    if not (taper_length_mm < pin_length_mm):
        errors.append(
            "Taper length (%.3f mm) must be less than total pin length (%.3f mm)." % (taper_length_mm, pin_length_mm)
        )
    if not (tip_diameter_mm >= NOZZLE_WIDTH_MM):
        errors.append(
            "Computed tip diameter (%.3f mm) is below minimum printable size (%.1f mm). "
            "Reduce taper length/angle or increase nominal diameter." % (tip_diameter_mm, NOZZLE_WIDTH_MM)
        )
    if not (radial_segments >= 8):
        errors.append("Radial segments must be at least 8.")

    return errors


# ---------------------------------------------------------------------------
# Pin mesh construction
# ---------------------------------------------------------------------------

def _make_ring_verts(bm, radius, z, segments):
    verts = []
    for i in range(segments):
        ang = 2.0 * pi * i / segments
        verts.append(bm.verts.new((radius * cos(ang), radius * sin(ang), z)))
    return verts


def _bridge_rings(bm, ring_a, ring_b):
    n = len(ring_a)
    for i in range(n):
        i_next = (i + 1) % n
        bm.faces.new([ring_a[i], ring_a[i_next], ring_b[i_next], ring_b[i]])


def _cap_fan(bm, ring, z, reverse=False):
    center = bm.verts.new((0.0, 0.0, z))
    n = len(ring)
    for i in range(n):
        i_next = (i + 1) % n
        if reverse:
            bm.faces.new([center, ring[i_next], ring[i]])
        else:
            bm.faces.new([center, ring[i], ring[i_next]])


def build_pin_bmesh(pin_diameter_mm, pin_length_mm, taper_length_mm,
                     tip_diameter_mm, radial_segments):
    bm          = bmesh.new()
    pin_radius  = pin_diameter_mm / 2.0
    tip_radius  = tip_diameter_mm / 2.0
    shaft_top_z = pin_length_mm - taper_length_mm

    back_ring      = _make_ring_verts(bm, pin_radius, 0.0,          radial_segments)
    shaft_top_ring = _make_ring_verts(bm, pin_radius, shaft_top_z,  radial_segments)
    tip_ring       = _make_ring_verts(bm, tip_radius, pin_length_mm, radial_segments)

    _bridge_rings(bm, back_ring, shaft_top_ring)
    _bridge_rings(bm, shaft_top_ring, tip_ring)
    _cap_fan(bm, back_ring, 0.0, reverse=True)
    _cap_fan(bm, tip_ring,  pin_length_mm, reverse=False)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


# ---------------------------------------------------------------------------
# Hole cutter mesh construction
# ---------------------------------------------------------------------------

def build_hole_cutter_bmesh(hole_diameter_mm, hole_depth_mm,
                             hole_extend_margin_mm, radial_segments):
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        cap_tris=True,
        segments=radial_segments,
        radius1=hole_diameter_mm / 2.0,
        radius2=hole_diameter_mm / 2.0,
        depth=hole_depth_mm + hole_extend_margin_mm,
    )
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
    return bm


# ---------------------------------------------------------------------------
# Object creation
# ---------------------------------------------------------------------------

def create_mesh_object_from_bmesh(bm, name, location):
    mesh_data = bpy.data.meshes.new(name)
    bm.to_mesh(mesh_data)
    bm.free()
    obj = bpy.data.objects.new(name, mesh_data)
    bpy.context.collection.objects.link(obj)
    obj.location = location
    return obj


# ---------------------------------------------------------------------------
# Operator
# ---------------------------------------------------------------------------

def _update_interference_preset(self, context):
    """Overwrite interference_mm from the preset — CUSTOM leaves it untouched."""
    if self.interference_preset in PRESS_FIT_INTERFERENCE_PRESETS_MM:
        self.interference_mm = PRESS_FIT_INTERFERENCE_PRESETS_MM[self.interference_preset]


class OBJECT_OT_add_press_pin(bpy.types.Operator):
    """Add a tapered press-fit pin and matching hole cutter"""
    bl_idname  = "object.add_press_pin"
    bl_label   = "Add Press-Fit Pin"
    bl_options = {'REGISTER', 'UNDO'}

    nominal_diameter_mm: bpy.props.FloatProperty(
        name="Nominal Diameter (mm)",
        description="Centerline reference diameter the joint is built around",
        default=5.0, min=0.0001, soft_min=1.0, soft_max=50.0,
    )
    interference_preset: bpy.props.EnumProperty(
        name="Interference Preset",
        items=[
            ('LIGHT',  "Light",  "0.10 mm diametral interference"),
            ('MEDIUM', "Medium", "0.20 mm diametral interference"),
            ('TIGHT',  "Tight",  "0.30 mm diametral interference"),
            ('CUSTOM', "Custom", "Use the value below, no overwrite"),
        ],
        default='MEDIUM',
        update=_update_interference_preset,
    )
    interference_mm: bpy.props.FloatProperty(
        name="Interference (mm)",
        description="Total diametral interference (pin OD minus hole ID)",
        default=0.20, min=0.0001, soft_max=1.0,
    )
    pin_diameter_compensation_mm: bpy.props.FloatProperty(
        name="Pin Compensation (mm)",
        description="Subtracted from modeled pin diameter to counteract FDM overextrusion on external features",
        default=-0.1,
    )
    hole_diameter_compensation_mm: bpy.props.FloatProperty(
        name="Hole Compensation (mm)",
        description="Added to modeled hole diameter to counteract FDM undersize on internal features",
        default=0.2,
    )
    pin_length_mm: bpy.props.FloatProperty(
        name="Pin Length (mm)",
        description="Total pin length from back face to tip apex, including the taper",
        default=10.0, min=0.0001, soft_max=100.0,
    )
    taper_length_mm: bpy.props.FloatProperty(
        name="Taper Length (mm)",
        description="Axial length of the tapered lead-in at the tip",
        default=2.0, min=0.0001, soft_max=20.0,
    )
    taper_angle_deg: bpy.props.FloatProperty(
        name="Taper Half-Angle (deg)",
        description="Half-angle of the taper cone measured from the central axis",
        default=30.0, min=0.0001, max=89.9999,
    )
    hole_depth_mm: bpy.props.FloatProperty(
        name="Hole Depth (mm)",
        description="Depth of the hole cutter from its mouth face",
        default=10.0, min=0.0001, soft_max=100.0,
    )
    hole_extend_margin_mm: bpy.props.FloatProperty(
        name="Extend Margin (mm)",
        description="Extra cutter length beyond hole depth for clean through-cuts",
        default=0.0, min=0.0, soft_max=10.0,
    )
    radial_segments: bpy.props.IntProperty(
        name="Radial Segments",
        description="Circular resolution for both pin and cutter",
        default=32, min=8, soft_max=128,
    )

    def draw(self, context):
        layout = self.layout

        fit = layout.box()
        fit.label(text="Fit")
        fit.prop(self, "nominal_diameter_mm")
        fit.prop(self, "interference_preset")
        fit.prop(self, "interference_mm")

        pin_diameter_mm, hole_diameter_mm = compute_press_fit_diameters(
            self.nominal_diameter_mm, self.interference_mm,
            self.pin_diameter_compensation_mm, self.hole_diameter_compensation_mm,
        )
        tip_diameter_mm = compute_tip_diameter(pin_diameter_mm, self.taper_length_mm, self.taper_angle_deg)
        fit.label(text="Pin Ø: %.3f mm   Hole Ø: %.3f mm" % (pin_diameter_mm, hole_diameter_mm))

        comp = layout.box()
        comp.label(text="FDM Compensation")
        comp.prop(self, "pin_diameter_compensation_mm")
        comp.prop(self, "hole_diameter_compensation_mm")

        pin = layout.box()
        pin.label(text="Pin Geometry")
        pin.prop(self, "pin_length_mm")
        pin.prop(self, "taper_length_mm")
        pin.prop(self, "taper_angle_deg")
        pin.label(text="Tip Ø: %.3f mm" % tip_diameter_mm)

        hole = layout.box()
        hole.label(text="Hole Cutter")
        hole.prop(self, "hole_depth_mm")
        hole.prop(self, "hole_extend_margin_mm")

        layout.prop(self, "radial_segments")

    def execute(self, context):
        errors = validate_press_pin_parameters(
            self.nominal_diameter_mm, self.interference_mm,
            self.pin_diameter_compensation_mm, self.hole_diameter_compensation_mm,
            self.pin_length_mm, self.taper_length_mm, self.taper_angle_deg,
            self.radial_segments,
        )
        if errors:
            self.report({'ERROR'}, errors[0])
            return {'CANCELLED'}

        pin_diameter_mm, hole_diameter_mm = compute_press_fit_diameters(
            self.nominal_diameter_mm, self.interference_mm,
            self.pin_diameter_compensation_mm, self.hole_diameter_compensation_mm,
        )
        tip_diameter_mm = compute_tip_diameter(pin_diameter_mm, self.taper_length_mm, self.taper_angle_deg)
        location = context.scene.cursor.location.copy()

        try:
            pin_bm  = build_pin_bmesh(pin_diameter_mm, self.pin_length_mm,
                                       self.taper_length_mm, tip_diameter_mm, self.radial_segments)
            pin_obj = create_mesh_object_from_bmesh(pin_bm, "PressPin", location)

            cutter_bm  = build_hole_cutter_bmesh(hole_diameter_mm, self.hole_depth_mm,
                                                   self.hole_extend_margin_mm, self.radial_segments)
            cutter_obj = create_mesh_object_from_bmesh(cutter_bm, "PressPin_HoleCutter", location)
        except Exception as exc:
            self.report({'ERROR'}, "Failed to build geometry: %s" % exc)
            return {'CANCELLED'}

        bpy.ops.object.select_all(action='DESELECT')
        pin_obj.select_set(True)
        cutter_obj.select_set(True)
        context.view_layer.objects.active = pin_obj

        self.report({'INFO'},
            "Press pin Ø%.2f mm + hole cutter Ø%.2f mm — interference %.2f mm"
            % (pin_diameter_mm, hole_diameter_mm, self.interference_mm))
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Registration
# ---------------------------------------------------------------------------

def register():
    bpy.utils.register_class(OBJECT_OT_add_press_pin)


def unregister():
    bpy.utils.unregister_class(OBJECT_OT_add_press_pin)


if __name__ == "__main__":
    register()
