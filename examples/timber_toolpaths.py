import math

from compas.geometry import Frame, Vector, Point, Line, Polyline, Plane, Transformation, Brep, Curve

from compas_timber.elements import Beam
from compas_timber.fabrication import JackRafterCut, JackRafterCutProxy
from compas_timber.fabrication import Lap, LapProxy
from compas_timber.fabrication import BTLxProcessing
from compas_timber.fabrication import OrientationType

from compas_rhino.conversions import plane_to_rhino
from Rhino.Geometry import CurveOffsetCornerStyle


def get_toolpath_from_lap_processing(beam: Beam, 
                                     processing: BTLxProcessing,
                                     machining_transformation: Transformation = None,
                                     tool_radius: float = 0.2,
                                     stepdown: float = 0.05,
                                     min_step: float = None,
                                     divisions: float = None,
                                     approach_height: float = 0.5,
                                     machining_side: int = 0,
                                     tolerance : float = 1e-3):
    # tx = Transformation.from_frame(beam.frame).inverse()
    volume = processing.volume_from_params_and_beam(beam)
    volume_at_origin = volume.transformed(machining_transformation)

    e = Brep.from_mesh(volume_at_origin.to_mesh())
    
    slices = []
    slicing_frames = []

    levels = int(beam.height / stepdown) + 1

    machining_frame = beam.ref_sides[machining_side].transformed(machining_transformation)

    for i in range(levels):
        frame = machining_frame.copy()
        frame.point += -frame.zaxis * (stepdown * i)
        slicing_frames.append(frame)
        slices += e.slice(frame)

    direction = machining_frame.zaxis
    radius = tool_radius / 2

    spirals = []
    
    for current_slice in slices:
        point = current_slice.point_at(current_slice.domain[0])
        plane = Plane(point, direction)

        num_offsets = int((beam.width / 2) / radius) - 1
        current_offset = current_slice
        slice_offsets = [current_offset]

        for i in range(num_offsets):
            offset_step = radius * -1
            c = Curve.from_native(current_offset.native_curve.Offset(plane_to_rhino(plane), offset_step, tolerance, CurveOffsetCornerStyle.Chamfer)[0])
            current_offset = c
            slice_offsets.insert(0, c)
        
        spirals.append(slice_offsets)

    path = []

    for slice_offsets, slicing_frame in zip(spirals, slicing_frames):
        next_slice_start_point = None
        for slice_offset in slice_offsets:
            if min_step:
                _params, points = slice_offset.divide_by_length(min_step, return_points=True)
            elif divisions:
                _params, points = slice_offset.divide_by_count(divisions, return_points=True)
            if next_slice_start_point is None:
                next_slice_start_point = points[0]
            for point in points:
                frame = slicing_frame.copy()
                frame.point = point
                path.append(frame)

        # Move in the same frame to the start of the next slice
        frame = slicing_frame.copy()
        frame.point = next_slice_start_point
        path.append(frame)

    # Add safe approach and retract frames to the toolpath
    safe_approach = path[0].copy()
    # safe_approach.point.z += approach_height
    safe_approach.point += safe_approach.zaxis * approach_height
    path.insert(0, safe_approach)

    safe_retract = path[-1].copy()
    safe_retract.point.z = safe_approach.point.z
    path.append(safe_retract)

    return path, volume_at_origin, slices, slicing_frames

def get_toolpath_from_processing(beam: Beam, processing: BTLxProcessing, machining_transformation: Transformation, tool_radius: float, **kwargs):
    if isinstance(processing, (Lap, LapProxy)):
        return get_toolpath_from_lap_processing(beam, processing, machining_transformation, tool_radius, **kwargs)

