import math

from compas.geometry import Frame, Transformation, Brep, Curve

from compas_timber.elements import Beam
from compas_timber.fabrication import JackRafterCut, JackRafterCutProxy, StepJointNotch
from compas_timber.fabrication import Lap, LapProxy
from compas_timber.fabrication import BTLxProcessing

from compas_rhino.conversions import frame_to_rhino_plane
from compas_rhino.geometry import RhinoNurbsSurface

from Rhino.Geometry import CurveOffsetCornerStyle # type: ignore


def get_toolpath_from_lap_processing(beam: Beam, 
                                     processing: BTLxProcessing,
                                     machining_transformation: Transformation = None,
                                     machining_frame: Frame = None,
                                     tool_radius: float = 0.2,
                                     stepdown: float = 0.05,
                                     min_step: float = None,
                                     approach_height: float = 0.5,
                                     tolerance : float = 1e-3,
                                     **kwargs):

    volume = processing.volume_from_params_and_beam(beam)
    volume_at_origin = volume.transformed(machining_transformation)

    e = Brep.from_mesh(volume_at_origin.to_mesh())
    
    slices = []
    slicing_frames = []

    levels = int(beam.height / stepdown) + 1

    for i in range(levels):
        frame = machining_frame.copy()
        frame.point += -frame.zaxis * (stepdown * i)
        slicing_frames.append(frame)
        slices += e.slice(frame)

    radius = tool_radius / 2

    spirals = []
    
    for current_slice, slicing_frame in zip(slices, slicing_frames):
        num_offsets = int((beam.width / 2) / radius) - 1
        current_offset = current_slice
        slice_offsets = [current_offset]

        for i in range(num_offsets):
            offset_step = radius * -1
            curve_offset_style = CurveOffsetCornerStyle.NONE
            c = Curve.from_native(current_offset.native_curve.Offset(frame_to_rhino_plane(slicing_frame), offset_step, tolerance, curve_offset_style)[0])
            
            current_offset = c
            slice_offsets.insert(0, c)
        
        spirals.append(slice_offsets)

    path = []

    for slice_offsets, slicing_frame in zip(spirals, slicing_frames):
        next_slice_start_point = None
        for slice_offset in slice_offsets:
            max_divisions = int(slice_offset.length() / min_step)
            _params, points = slice_offset.divide_by_count(max_divisions, return_points=True)

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

    path = add_safe_points(path, approach_height)

    return "subtraction", path, volume_at_origin, slices, slicing_frames


def get_toolpath_from_jackraftercut_processing(beam: Beam, 
                                               processing: BTLxProcessing,
                                               machining_transformation: Transformation = None,
                                               machining_frame: Frame = None,
                                               tool_radius: float = 0.2,
                                               stepdown: float = 0.05,
                                               min_step: float = None,
                                               approach_height: float = 0.5,
                                               flip_direction: bool = False,
                                               tolerance : float = 1e-3,
                                               **kwargs):
    plane = processing.plane_from_params_and_beam(beam)
    plane_at_origin = plane.transformed(machining_transformation)
    frame = Frame.from_plane(plane_at_origin)

    blank_brep_at_origin = beam.blank.to_brep().transformed(machining_transformation)
    slices = blank_brep_at_origin.slice(frame)
    if len(slices) != 1:
        raise ValueError("Expected exactly one slice from the blank at the machining plane.")
    
    slice_surface = RhinoNurbsSurface.from_corners(slices[0].points[0:4])

    path = []
    radius = tool_radius / 2
    num_steps = int((beam.height / 2) / radius) - 1
    isocurves = []

    if flip_direction:
        params = slice_surface.space_u(num_steps)
    else:
        params = slice_surface.space_v(num_steps)

    for i, param in enumerate(params):
        if flip_direction:
            isocurve = slice_surface.isocurve_u(param)
        else:
            isocurve = slice_surface.isocurve_v(param)

        max_divisions = int(isocurve.length() / min_step)
        _params, points = isocurve.divide_by_count(max_divisions, return_points=True)
        if i % 2 == 0:
            points.reverse()

        for point in points:
            frame = machining_frame.copy()
            frame.point = point
            path.append(frame)

        isocurves.append(isocurve)

    path = add_safe_points(path, approach_height)

    return "cut", path, slice_surface, isocurves


def add_safe_points(path: list[Frame], approach_height: float) -> list[Frame]:
    # Add safe approach and retract frames to the toolpath
    safe_approach = path[0].copy()
    safe_approach.point += safe_approach.zaxis * approach_height
    path.insert(0, safe_approach)

    safe_retract = path[-1].copy()
    safe_retract.point.z = safe_approach.point.z
    path.append(safe_retract)

    return path


def get_toolpath_from_processing(beam: Beam, processing: BTLxProcessing, machining_transformation: Transformation, machining_side: int, **kwargs):
    machining_frame = beam.ref_sides[machining_side].transformed(machining_transformation)

    if isinstance(processing, (Lap, LapProxy)):
        return get_toolpath_from_lap_processing(beam, processing, machining_transformation, machining_frame=machining_frame, **kwargs)
    elif isinstance(processing, (JackRafterCut, JackRafterCutProxy)):
        return get_toolpath_from_jackraftercut_processing(beam, processing, machining_transformation, machining_frame=machining_frame, **kwargs)


