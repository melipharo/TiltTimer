#Author-m3liphar0
#Description-converts STGC pattern to slotted plane
# https://help.autodesk.com/view/fusion360/ENU/?guid=GUID-CB1A2357-C8CD-474D-921E-992CA3621D04

import adsk.core
import adsk.fusion
import adsk.cam
import traceback

import math


class UserParameters:
    def _get_value_from_design(self, name):
        try:
            return self.design.userParameters.itemByName(name).value
        except:
            # Internally, length units within Fusion core are _ALWAYS_ in centimeters.
            # see https://forums.autodesk.com/t5/fusion-360-api-and-scripts/fusion-360-api-units/td-p/9411489
            self.design.userParameters.add(
                name,
                adsk.core.ValueInput.createByString(self.defaults[name]),
                "mm",
                "STGC_Encoder_producer default variable"
            )
            return self.design.userParameters.itemByName(name).value

    def __init__(self, design):
        self.defaults = {
            "disk_diameter": "30 mm",
            "disk_thickness": "2 mm",
            "disk_center_to_slot": "10 mm",
            "disk_axis_diameter": "3 mm",
            "head_width": "3.4 mm",
            "head_height": "2.8 mm",
            "head_to_disk_edge": "1 mm"
        }

        self.design = design

        self.disk_diameter = self._get_value_from_design("disk_diameter")
        self.disk_thickness = self._get_value_from_design("disk_thickness")
        self.disk_center_to_slot = self._get_value_from_design("disk_center_to_slot")
        self.disk_axis_diameter = self._get_value_from_design("disk_axis_diameter")
        self.head_width = self._get_value_from_design("head_width")
        self.head_height = self._get_value_from_design("head_height")
        self.head_to_disk_edge = self._get_value_from_design("head_to_disk_edge")

        self.disk_radius = self.disk_diameter / 2

        # TODO these variables are not in Fusion document
        self.bits_pattern_string = '000000000000111000000110000001111111111111000111111001111110'
        self.heads_distance_bits = 5
        self.heads_count = 5

        self.bits_pattern = list(map(int, self.bits_pattern_string))
        assert sorted(set(self.bits_pattern)) == [0, 1], 'Pattern contains wrong characters'


def ra2xy(center, radius, angle):
    return adsk.core.Point3D.create(
                    center.x + radius * math.cos(angle),
                    center.y + radius * math.sin(angle),
                    0
                )


def produce_slots(root, plane, user_params):
    # slots = [(value, bit_length),]
    def get_slots(pattern):
        bits_count = len(pattern)
        slots = []

        current_length = 1
        for bit_pos in range(bits_count):
            current_bit = pattern[bit_pos]
            if bit_pos < bits_count - 1:
                next_bit = pattern[bit_pos + 1]
            else:
                next_bit = pattern[0]

            if current_bit != next_bit or bit_pos == bits_count - 1:
                slots.append((current_bit, current_length))
                current_length = 1
            else:
                current_length += 1
        # merge edge slots if are the same
        if slots[0][0] == slots[-1][0]:
            extra_length = slots[-1][1]
            del slots[-1]
            slots[0] = (slots[0][0], slots[0][1] + extra_length)
        return slots

    sketches = root.sketches
    extrudes = root.features.extrudeFeatures

    slots_sketch = sketches.add(plane)
    slots_sketch.name = "slots"
    sketch_center_point = slots_sketch.modelToSketchSpace(plane.geometry.origin)

    # slots
    slots = get_slots(user_params.bits_pattern)
    start_angle = 0
    for slot in slots:
        arc_length = (slot[1] * 2 * math.pi) / len(user_params.bits_pattern)
        if slot[0] == 1:
            ark_start_point = ra2xy(sketch_center_point, user_params.disk_radius, start_angle)
            ark_end_point = ra2xy(sketch_center_point, user_params.disk_radius, start_angle + arc_length)
            slots_sketch.sketchCurves.sketchLines.addByTwoPoints(sketch_center_point, ark_start_point)
            slots_sketch.sketchCurves.sketchLines.addByTwoPoints(sketch_center_point, ark_end_point)
            slots_sketch.sketchCurves.sketchArcs.addByCenterStartSweep(sketch_center_point, ark_start_point, arc_length)
        start_angle += arc_length

    # solid center
    slots_sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.disk_center_to_slot)

    sketch_profiles = adsk.core.ObjectCollection.create()
    for profile in slots_sketch.profiles:
        sketch_profiles.add(profile)
    distance = adsk.core.ValueInput.createByReal(user_params.disk_thickness)
    extrude1 = extrudes.addSimple(sketch_profiles, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body1 = extrude1.bodies.item(0)
    body1.name = "slots"

    #
    # # Get the state of the extrusion
    # health = extrude1.healthState
    # if health == adsk.fusion.FeatureHealthStates.WarningFeatureHealthState or health == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
    #     message = extrude1.errorOrWarningMessage
    #
    # # Get the state of timeline object
    # timeline = design.timeline
    # timelineObj = timeline.item(timeline.count - 1)
    # health = timelineObj.healthState
    # message = timelineObj.errorOrWarningMessage

    # axis hole
    # TODO
    # slots_sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.disk_axis_diameter / 2)


def produce_heads(root, plane, user_params):
    sketches = root.sketches
    extrudes = root.features.extrudeFeatures
    heads_sketch = sketches.add(plane)
    heads_sketch.name = "heads"
    sketch_center_point = heads_sketch.modelToSketchSpace(plane.geometry.origin)

    heads_distance_angle = (user_params.heads_distance_bits * 2 * math.pi) / len(user_params.bits_pattern)
    start_angle = 0
    for head in range(user_params.heads_count):
        heads_sketch.sketchCurves.sketchCircles.addByCenterRadius(
            ra2xy(sketch_center_point, user_params.disk_radius - user_params.head_to_disk_edge, start_angle),
            user_params.head_height
        )
        start_angle += heads_distance_angle


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface

        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        if not design:
            ui.messageBox('It is not supported in current workspace, please change to MODEL workspace and try again.')
            return

        user_params = UserParameters(design)
        root = design.rootComponent

        # TODO plane is need to be selectable by user
        # sel = ui.selectEntity('Select a path to create a pipe', 'Edges,SketchCurves')
        # selObj = sel.entity
        plane_xy = root.xYConstructionPlane

        produce_slots(root, plane_xy, user_params)
        produce_heads(root, plane_xy, user_params)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
