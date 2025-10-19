
from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Polygon
from compas.geometry import Vector
from compas.geometry import intersection_line_line

class ConnectionPoint:
    def __init__(self, point: Point = None, parameter: float = None, vector: Vector = None):
        self.point = point
        self.parameter = parameter
        self.vector = vector


class RodSegment:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end
        self.rod = None

    @property
    def line(self) -> Line:
        return Line(self.start, self.end)


class RFUnit:
    def __init__(self, shape: Polygon, start_eccentricity=0.0, end_eccentricity=0.0, overlap=0.0):
        self.shape = shape                              # The polygonal shape inscribing the unit
        self.start_eccentricity = start_eccentricity    # Offset distance for the line start points
        self.end_eccentricity = end_eccentricity        # Offset distance for the line end points
        self.overlap = overlap                          # Overlap of the rods beyond the intersection point
        self.segments = []  # List of segment objects representing the unit's rods
        self.connection_points = []  # List of RFConnectionPoint

    @property
    def centroid(self) -> Point:
        return self.shape.centroid

    @property
    def edges(self) -> list[Line]:
        return self.shape.lines

    def generate_segments(self):
        self.segments = []
        self.connection_points = []  # Reset connection points

        for edge in self.edges:
            midpoint = edge.midpoint  # Calculate midpoint of the edge

            # Calculate the normal vector to the edge
            edge_vector = Vector.from_start_end(edge.start, edge.end)

            # Calculate eccentricities
            start = self.centroid + edge_vector * self.start_eccentricity
            end = midpoint + edge_vector * self.end_eccentricity

            # Extend the line inwards
            start = start - Vector.from_start_end(start, end)

            rod_segment = RodSegment(start, end)

            self.segments.append(rod_segment)

            # Store the connection point
            _point, param = edge.closest_point(end, return_parameter=True)
            vector = Vector.from_start_end(start, end).unitized()
            connection = ConnectionPoint(end, param, vector)
            self.connection_points.append(connection)

    def adjust_segments(self, overlap=None):
        overlap = overlap if overlap is not None else self.overlap  # Ensure overlap is not None

        adjusted_segments = []

        num_segments = len(self.segments)
        for i in range(num_segments):
            # Get a pair of segments (current and next, wrapping around)
            segment = self.segments[i]
            next_segment = self.segments[(i + 1) % num_segments]

            # Find the intersection between the current rod and the next rod
            intersection = intersection_line_line((segment.start, segment.end), (next_segment.start, next_segment.end))

            if not intersection or intersection[0] is None:
                print(f"Warning: No intersection found between Rod {i} and Rod {(i + 1) % num_segments}.")
                adjusted_segments.append(segment)  # Keep original rod if no intersection
                continue

            # Use the first intersection point
            intersection_point = Point(*intersection[0])

            # Extend the rod from the intersection point with the specified overlap
            segment_vector = Vector.from_start_end(segment.start, intersection_point).unitized()
            extended_start = intersection_point + segment_vector * -overlap

            # Update the segment's start point
            segment.start = extended_start

            # Store the adjusted rod
            adjusted_segments.append(segment)

        # Replace the old rods with the adjusted ones
        self.segments = adjusted_segments

