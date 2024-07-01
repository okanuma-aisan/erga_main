from __future__ import annotations
import math

class Point:
    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)
    
    def __add__(self, p: Point) -> Point:
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p: Point) -> Point:
        return Point(self.x - p.x, self.y - p.y)

    def yaw_to(self, p: Point) -> float:
        return math.atan2(p.y - self.y, p.x - self.x)
    
    def __eq__(self, p: Point) -> bool:
        if self.x != p.x:
            return False
        if self.y != p.y:
            return False
        return True

    def absolute_distance_to_point(self, p: Point) -> float:
        a = (self.x-p.x)**2
        b = (self.y-p.y)**2
        added = a+b
        distance = math.sqrt(added)
        return distance

    def fast_distance_to_point(self, p: Point) -> float:
        a = (self.x-p.x)**2
        b = (self.y-p.y)**2
        added = a+b
        #distance = math.sqrt(added)
        return added
    
    def absolute_distance_to_xy(self, x: float, y: float) -> float:
        origin = Point(x, y)
        distance = self.absolute_distance_to_point(origin)
        return distance

    def as_tuple(self) -> tuple:
        return (self.x, self.y)

    def __str__(self) -> str:
        return f"Point({self.x}, {self.y})"

    def __repr__(self) -> str:
        return self.__str__()
    
def calc_midpoint(p1: Point, p2: Point) -> Point:
    delta = p1 + p2
    return Point(delta.x/2, delta.y/2)

# https://algorithmtutor.com/Computational-Geometry/Check-if-two-line-segment-intersect/
def calc_cross_product(p1: Point, p2: Point) -> float:
	return p1.x * p2.y - p2.x * p1.y

def calc_direction(p1: Point, p2: Point, p3: Point) -> float:
	return calc_cross_product(p3 - p1, p2 - p1)

def point_on_segment(segment: tuple, p: Point) -> bool:
    p1, p2 = segment
    return min(p1.x, p2.x) <= p.x <= max(p1.x, p2.x) and min(p1.y, p2.y) <= p.y <= max(p1.y, p2.y)

def lines_intersect(line_segment_a: tuple, line_segment_b: tuple) -> bool:
    p1, p2 = line_segment_a
    p3, p4 = line_segment_b

    d1 = calc_direction(p3, p4, p1)
    d2 = calc_direction(p3, p4, p2)
    d3 = calc_direction(p1, p2, p3)
    d4 = calc_direction(p1, p2, p4)

    # lines intersect
    if ((d1 > 0.0 and d2 < 0.0) or (d1 < 0.0 and d2 > 0.0)) and ((d3 > 0.0 and d4 < 0.0) or (d3 < 0.0 and d4 > 0.0)):
        return True

    # on top of segment
    elif d1 == 0.0 and point_on_segment((p3, p4), p1):
        return True
    elif d2 == 0.0 and point_on_segment((p3, p4), p2):
        return True
    elif d3 == 0.0 and point_on_segment((p1, p2), p3):
        return True
    elif d4 == 0.0 and point_on_segment((p1, p2), p4):
        return True
    
    return False

# https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
# returns none of line is not intersecting
def calc_intersection_point(line_segment: tuple, ray_segment: tuple) -> Point:
    if not lines_intersect(line_segment, ray_segment):
        return

    line_point_a, line_point_b = line_segment
    ray_point_a, ray_point_b = ray_segment

    denominator = ( (ray_point_b.x - ray_point_a.x) * (line_point_a.y - line_point_b.y) - (ray_point_b.y - ray_point_a.y) * (line_point_a.x - line_point_b.x) )
    if denominator == 0: # parallel
        return

    out_x = ( (ray_point_b.x * ray_point_a.y - ray_point_b.y * ray_point_a.x) * (line_point_a.x - line_point_b.x) - (ray_point_b.x - ray_point_a.x) * (line_point_a.x * line_point_b.y - line_point_a.y * line_point_b.x) ) / denominator
    out_y = ( (ray_point_b.x * ray_point_a.y - ray_point_b.y * ray_point_a.x) * (line_point_a.y - line_point_b.y) - (ray_point_b.y - ray_point_a.y) * (line_point_a.x * line_point_b.y - line_point_a.y * line_point_b.x) ) / denominator

    return Point(out_x, out_y)

def calc_rotated_point(origin_point: Point, angle_theta: float) -> Point:
    # (autoware is weird sometimes)
    # angle_theta = 1.570796327 - angle_theta
    new_x = (origin_point.x * math.cos(angle_theta)) - (origin_point.y * math.sin(angle_theta))
    new_y = (origin_point.x * math.sin(angle_theta)) + (origin_point.y * math.cos(angle_theta))
    return Point(new_x, new_y)

def calc_translated_rotated_point(origin_point: Point, relative_point: Point, angle: float) -> Point:
    # rotate
    rotated_point = calc_rotated_point(relative_point, angle)

    # translate
    new_global_point = origin_point + rotated_point
    return new_global_point

# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
def quaternion_to_rpy(quat: tuple) -> tuple:
    q_x, q_y, q_z, q_w = quat
    roll  = math.atan2(2.0*(q_y*q_z + q_w*q_x), q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)
    pitch = math.asin(-2.0*(q_x*q_z - q_w*q_y))
    yaw   = math.atan2(2.0*(q_x*q_y + q_w*q_z), q_w*q_w + q_x*q_x - q_y*q_y - q_z*q_z)
    return (roll, pitch, yaw)

def rpy_to_quaternion(roll, pitch, yaw) -> tuple:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    y = cy * cp * sr - sy * sp * cr
    x = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return (x, y, z, w)

# based upon https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
def point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    num_vertices = len(polygon)
    x, y = point.x, point.y
    inside = False
 
    # Store the first point in the polygon and initialize the second point
    p1 = polygon[0]
 
    # Loop through each edge in the polygon
    for i in range(1, num_vertices + 1):
        # Get the next point in the polygon
        p2 = polygon[i % num_vertices]
 
        # Check if the point is above the minimum y coordinate of the edge
        if y > min(p1.y, p2.y):
            # Check if the point is below the maximum y coordinate of the edge
            if y <= max(p1.y, p2.y):
                # Check if the point is to the left of the maximum x coordinate of the edge
                if x <= max(p1.x, p2.x):
                    # Calculate the x-intersection of the line connecting the point to the edge
                    x_intersection = (y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x
 
                    # Check if the point is on the same line as the edge or to the left of the x-intersection
                    if p1.x == p2.x or x <= x_intersection:
                        # Flip the inside flag
                        inside = not inside
 
        # Store the current point as the first point for the next iteration
        p1 = p2
 
    # Return the value of the inside flag
    return inside