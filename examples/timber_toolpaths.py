import math

from compas.geometry import Frame, Transformation, Brep, Curve, Point, Line

from compas_timber.elements import Beam
from compas_timber.fabrication import JackRafterCut, JackRafterCutProxy, StepJointNotch
from compas_timber.fabrication import Lap, LapProxy
from compas_timber.fabrication import FrenchRidgeLap
from compas_timber.fabrication import BTLxProcessing

from compas_rhino.conversions import frame_to_rhino_plane
from compas_rhino.geometry import RhinoNurbsSurface

from Rhino.Geometry import CurveOffsetCornerStyle  # type: ignore


def get_toolpath_from_lap_processing(
    beam: Beam,
    processing: BTLxProcessing,
    machining_transformation: Transformation = None,
    machining_frame: Frame = None,
    tool_radius: float = 0.2,
    stepdown: float = 0.05,
    min_step: float = None,
    approach_height: float = 0.5,
    tolerance: float = 1e-3,
    **kwargs
):

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
            c = Curve.from_native(
                current_offset.native_curve.Offset(
                    frame_to_rhino_plane(slicing_frame),
                    offset_step,
                    tolerance,
                    curve_offset_style,
                )[0]
            )

            current_offset = c
            slice_offsets.insert(0, c)

        spirals.append(slice_offsets)

    path = []

    for slice_offsets, slicing_frame in zip(spirals, slicing_frames):
        next_slice_start_point = None
        for slice_offset in slice_offsets:
            max_divisions = int(slice_offset.length() / min_step)
            _params, points = slice_offset.divide_by_count(
                max_divisions, return_points=True
            )

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


def get_toolpath_from_jackraftercut_processing(
    beam: Beam,
    processing: BTLxProcessing,
    machining_transformation: Transformation = None,
    machining_frame: Frame = None,
    tool_radius: float = 0.2,
    stepdown: float = 0.05,
    min_step: float = None,
    approach_height: float = 0.5,
    flip_direction: bool = False,
    tolerance: float = 1e-3,
    **kwargs
):
    plane = processing.plane_from_params_and_beam(beam)
    plane_at_origin = plane.transformed(machining_transformation)
    frame = Frame.from_plane(plane_at_origin)

    blank_brep_at_origin = beam.blank.to_brep().transformed(machining_transformation)
    slices = blank_brep_at_origin.slice(frame)
    if len(slices) != 1:
        raise ValueError(
            "Expected exactly one slice from the blank at the machining plane."
        )

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


def get_toolpath_from_frenchRidgeLap_processing(
    beam: Beam,
    processing: BTLxProcessing,
    machining_transformation: Transformation = None,
    tool_radius: float = 0.2,
    stepdown: float = 0.05,
    min_step: float = None,
    divisions: float = None,
    approach_height: float = 0.5,
    machining_side: int = 0,
    tolerance: float = 1e-3,
):
    # tx = Transformation.from_frame(beam.frame).inverse()
    volume = processing.lap_volume_from_params_and_beam(beam)
    volume_at_origin = volume.transformed(machining_transformation)
    # print(type(volume_at_origin))

    e = volume_at_origin

    slices = []
    slicing_frames = []

    levels = int(beam.height / stepdown) + 1

    machining_frame = beam.ref_sides[machining_side].transformed(
        machining_transformation
    )

    for i in range(levels):
        frame = machining_frame.copy()
        frame.point += -frame.zaxis * (stepdown * i)
        slicing_frames.append(frame)
        slices += e.slice(frame)

    radius = tool_radius / 2

    spirals = []

    for current_slice, slicing_frame in zip(slices, slicing_frames):
        dis = distance_center_to_edge(current_slice.points)
        num_offsets = int(dis / radius)
        current_offset = current_slice
        slice_offsets = [current_offset]

        for i in range(num_offsets):
            offset_step = radius * -1
            curve_offset_style = CurveOffsetCornerStyle.NONE
            c = Curve.from_native(
                current_offset.native_curve.Offset(
                    frame_to_rhino_plane(slicing_frame),
                    offset_step,
                    tolerance,
                    curve_offset_style,
                )[0]
            )

            current_offset = c
            slice_offsets.insert(0, c)

        spirals.append(slice_offsets)

    path = []

    for slice_offsets, slicing_frame in zip(spirals, slicing_frames):
        next_slice_start_point = None
        for slice_offset in slice_offsets:
            if min_step:
                _params, points = slice_offset.divide_by_length(
                    min_step, return_points=True
                )
            elif divisions:
                _params, points = slice_offset.divide_by_count(
                    divisions, return_points=True
                )
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

    return path, volume_at_origin, slices, slicing_frames, spirals


def add_safe_points(path: list[Frame], approach_height: float) -> list[Frame]:
    # Add safe approach and retract frames to the toolpath
    safe_approach = path[0].copy()
    safe_approach.point += safe_approach.zaxis * approach_height
    path.insert(0, safe_approach)

    safe_retract = path[-1].copy()
    safe_retract.point.z = safe_approach.point.z
    path.append(safe_retract)

    return path


def distance_center_to_edge(points):
    """
    points: list of compas.geometry.Point
    return: min distance (float)

    get min distance from average center point to curve edge
    """
    # calcu average points center
    n = len(points)
    sum_x = sum(p.x for p in points)
    sum_y = sum(p.y for p in points)
    sum_z = sum(p.z for p in points)
    avg_center = Point(sum_x / n, sum_y / n, sum_z / n)

    # calculate distance to every line
    min_dist = float("inf")
    for i in range(n):
        start = points[i]
        end = points[(i + 1) % n]
        # skip zero-length line
        if start.distance_to_point(end) <= 0.001:
            continue
        line = Line(start, end)
        dist = avg_center.distance_to_line(line)
        if dist < min_dist:
            min_dist = dist

    return min_dist


def get_toolpath_from_processing(
    beam: Beam,
    processing: BTLxProcessing,
    machining_transformation: Transformation,
    machining_side: int,
    **kwargs
):
    machining_frame = beam.ref_sides[machining_side].transformed(
        machining_transformation
    )

    if isinstance(processing, (Lap, LapProxy)):
        return get_toolpath_from_lap_processing(
            beam,
            processing,
            machining_transformation,
            machining_frame=machining_frame,
            **kwargs
        )
    elif isinstance(processing, (JackRafterCut, JackRafterCutProxy)):
        return get_toolpath_from_jackraftercut_processing(
            beam,
            processing,
            machining_transformation,
            machining_frame=machining_frame,
            **kwargs
        )
    elif isinstance(processing, (FrenchRidgeLap)):
        return get_toolpath_from_frenchRidgeLap_processing(
            beam,
            processing,
            machining_transformation,
            machining_frame=machining_frame,
            **kwargs
        )
