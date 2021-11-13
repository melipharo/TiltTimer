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
                "STGC_Encoder_producer default value"
            )
            return self.design.userParameters.itemByName(name).value

    def __init__(self, design):
        self.defaults = {
            "sensor_outer_diameter": "30 mm",
            "slots_disk_thickness": "2 mm",
            "ports_disk_thickness": "0.6 mm",
            "head_disk_thickness": "4 mm",
            "center_axis_diameter": "3 mm",
            "head_diameter": "3 mm",
            "head_to_disk_gap": "0.6 mm",
            "ports_disk_to_slots_gap": "0.6 mm",
        }

        self.design = design

        self.sensor_outer_diameter = self._get_value_from_design("sensor_outer_diameter")
        self.slots_disk_thickness = self._get_value_from_design("slots_disk_thickness")
        self.ports_disk_thickness = self._get_value_from_design("ports_disk_thickness")
        self.head_disk_thickness = self._get_value_from_design("head_disk_thickness")
        self.center_axis_diameter = self._get_value_from_design("center_axis_diameter")
        self.head_diameter = self._get_value_from_design("head_diameter")
        self.head_to_disk_gap = self._get_value_from_design("head_to_disk_gap")
        self.ports_disk_to_slots_gap = self._get_value_from_design("ports_disk_to_slots_gap")

        # TODO these variables are not in Fusion document for now
        self.bits_pattern_string = '000000000000111000000110000001111111111111000111111001111110'
        self.heads_distance_bits = 5
        self.heads_count = 6

        self.bits_pattern = list(map(int, self.bits_pattern_string))

        self.bit_length_angle = (2 * math.pi) / len(self.bits_pattern)
        assert sorted(set(self.bits_pattern)) == [0, 1], 'Pattern contains wrong characters'

        self.sensor_outer_radius = self.sensor_outer_diameter / 2
        self.head_radius = self.head_diameter / 2
        self.slots_disk_outer_radius = self.sensor_outer_radius - self.ports_disk_to_slots_gap
        self.slots_disk_inner_radius = self.slots_disk_outer_radius - self.head_diameter - self.head_to_disk_gap * 2
        self.heads_distance_angle = (self.heads_distance_bits * 2 * math.pi) / len(self.bits_pattern)
        self.head_center_distance = self.slots_disk_inner_radius + self.head_to_disk_gap + self.head_diameter / 2



def ra2xy(center, radius, angle):
    return adsk.core.Point3D.create(
                    center.x + radius * math.cos(angle),
                    center.y + radius * math.sin(angle),
                    0
                )


def produce_slots(root, plane, user_params: UserParameters):
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

    sketch = sketches.add(plane)
    sketch.name = "slots"
    sketch_center_point = sketch.modelToSketchSpace(plane.geometry.origin)

    # axis hole
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.center_axis_diameter / 2)

    # solid center
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.slots_disk_inner_radius)

    # slots
    slots = get_slots(user_params.bits_pattern)
    start_angle = 0
    for slot in slots:
        arc_length = slot[1] * user_params.bit_length_angle
        if slot[0] == 1:
            ark_start_point = ra2xy(sketch_center_point, user_params.slots_disk_outer_radius, start_angle)
            ark_end_point = ra2xy(sketch_center_point, user_params.slots_disk_outer_radius, start_angle + arc_length)
            # slot ark
            sketch.sketchCurves.sketchArcs.addByCenterStartSweep(sketch_center_point, ark_start_point, arc_length)
            # slot sides
            side_cw_start_point = ra2xy(sketch_center_point, user_params.slots_disk_inner_radius, start_angle)
            side_ccw_start_point = ra2xy(sketch_center_point, user_params.slots_disk_inner_radius, start_angle + arc_length)
            sketch.sketchCurves.sketchLines.addByTwoPoints(side_cw_start_point, ark_start_point)
            sketch.sketchCurves.sketchLines.addByTwoPoints(side_ccw_start_point, ark_end_point)
        start_angle += arc_length

    sketch_profiles = adsk.core.ObjectCollection.create()
    for profile in sketch.profiles:
        sketch_profiles.add(profile)
    sketch_profiles.removeByIndex(0)  # remove axis hole (the ugly way)
    distance = adsk.core.ValueInput.createByReal(user_params.slots_disk_thickness)
    extrude1 = extrudes.addSimple(sketch_profiles, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = extrude1.bodies.item(0)
    body.name = "slots"

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

    return body


def produce_ports(root, plane, user_params: UserParameters):
    sketches = root.sketches
    extrudes = root.features.extrudeFeatures
    sketch = sketches.add(plane)
    sketch.name = "ports"
    sketch_center_point = sketch.modelToSketchSpace(plane.geometry.origin)

    # main solid
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.sensor_outer_radius)

    # axis hole
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.center_axis_diameter / 2)

    # ports
    for i in range(user_params.heads_count):
        start_angle = user_params.heads_distance_angle * i
        point_cw_far = ra2xy(sketch_center_point, user_params.slots_disk_outer_radius - user_params.head_to_disk_gap, start_angle)
        point_cw_near = ra2xy(sketch_center_point, user_params.slots_disk_inner_radius + user_params.head_to_disk_gap, start_angle)
        point_ccw_far = ra2xy(sketch_center_point, user_params.slots_disk_outer_radius - user_params.head_to_disk_gap,
                              start_angle + user_params.bit_length_angle)
        point_ccw_near = ra2xy(sketch_center_point, user_params.slots_disk_inner_radius + user_params.head_to_disk_gap,
                               start_angle + user_params.bit_length_angle)
        sketch.sketchCurves.sketchLines.addByTwoPoints(point_cw_near, point_cw_far)
        sketch.sketchCurves.sketchLines.addByTwoPoints(point_cw_far, point_ccw_far)
        sketch.sketchCurves.sketchLines.addByTwoPoints(point_ccw_far, point_ccw_near)
        sketch.sketchCurves.sketchLines.addByTwoPoints(point_ccw_near, point_cw_near)

    profile = sketch.profiles[0]
    distance = adsk.core.ValueInput.createByReal(-user_params.ports_disk_thickness)
    extrude1 = extrudes.addSimple(profile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = extrude1.bodies.item(0)
    body.name = "ports"
    return body


def produce_heads(root, plane, user_params: UserParameters):
    sketches = root.sketches
    extrudes = root.features.extrudeFeatures
    sketch = sketches.add(plane)
    sketch.name = "heads"
    sketch_center_point = sketch.modelToSketchSpace(plane.geometry.origin)

    # main solid
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.sensor_outer_radius)

    # axis hole
    sketch.sketchCurves.sketchCircles.addByCenterRadius(sketch_center_point, user_params.center_axis_diameter / 2)

    # heads
    for i in range(user_params.heads_count):
        start_angle = user_params.heads_distance_angle * i
        point_head_center = ra2xy(sketch_center_point, user_params.head_center_distance, start_angle + user_params.bit_length_angle / 2)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(point_head_center, user_params.head_radius)

    profile = sketch.profiles[0]
    distance = adsk.core.ValueInput.createByReal(-user_params.head_disk_thickness)
    extrude1 = extrudes.addSimple(profile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = extrude1.bodies.item(0)
    body.name = "heads"
    return body


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
        # create new component from root
        sensor_component = design.rootComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
        sensor_component.name = "STGC Sensor RUN"
        planes = sensor_component.constructionPlanes
        features = sensor_component.features

        # TODO plane is need to be selectable by user
        # sel = ui.selectEntity('Select a path to create a pipe', 'Edges,SketchCurves')
        # selObj = sel.entity

        plane_xy = sensor_component.xYConstructionPlane

        # create bodies
        slots_body = produce_slots(sensor_component, plane_xy, user_params)
        ports_body = produce_ports(sensor_component, plane_xy, user_params)

        plane_for_heads_offset = adsk.core.ValueInput.createByReal(-user_params.ports_disk_thickness)
        plane_for_heads_input = planes.createInput()
        plane_for_heads_input.setByOffset(plane_xy, plane_for_heads_offset)
        plane_for_heads = planes.add(plane_for_heads_input)
        heads_body = produce_heads(sensor_component, plane_for_heads, user_params)

        # merge bodies
        tool_bodies_collection = adsk.core.ObjectCollection.create()
        tool_bodies_collection.add(heads_body)
        combine_feature_input = features.combineFeatures.createInput(ports_body, tool_bodies_collection)
        combine_feature_input.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        features.combineFeatures.add(combine_feature_input)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
