import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point, Polygon

def create_line(point, length=20):
    """
    Create a line extending from a point in the direction of orientation.
    :param point: [x, y, theta] where theta is in radians
    :param length: Length of the line segment
    :return: LineString object
    """
    x, y, theta = point
    dx = np.cos(theta) * length  # Delta x
    dy = np.sin(theta) * length  # Delta y
    start = (x, y)  # Line starts at the input point
    end = (x + dx, y + dy)  # Line ends in the direction of orientation
    return LineString([start, end])

def find_intersections(lines):
    """
    Find intersections between all pairs of lines.
    :param lines: List of LineString objects
    :return: List of intersection points
    """
    intersections = []
    for i, line1 in enumerate(lines):
        for j, line2 in enumerate(lines):
            if i < j:  # Avoid duplicate pairs
                intersection = line1.intersection(line2)
                if isinstance(intersection, Point):  # Only consider point intersections
                    intersections.append(intersection)
    return intersections

def plot_lines_and_region(points, length=20):
    """
    Create lines from points with orientations and plot the region formed by their intersection.
    :param points: List of [x, y, theta] points
    :param length: Length of each line
    """
    # Create lines
    lines = [create_line(p, length) for p in points]
    
    # Find intersections
    intersection_points = find_intersections(lines)
    
    # Create a polygon from intersection points
    polygon = None
    if len(intersection_points) > 2:
        # Sort points to form a convex hull
        coords = [(pt.x, pt.y) for pt in intersection_points]
        polygon = Polygon(coords).convex_hull
    
    # Calculate area if the polygon is valid
    area = polygon.area if polygon and polygon.is_valid else 0
    
    # Plot lines and points
    plt.figure(figsize=(8, 8))
    for idx, (line, point) in enumerate(zip(lines, points)):
        x, y = line.xy
        plt.plot(x, y, label=f"Line {idx + 1}")
        plt.plot(point[0], point[1], 'go', label=f"Point {idx + 1} ({point[0]}, {point[1]})")
    
    # Plot intersection points
    for pt in intersection_points:
        plt.plot(pt.x, pt.y, 'ro', label=f"Intersection ({pt.x:.2f}, {pt.y:.2f})")
    
    # Plot the region
    if polygon and polygon.is_valid:
        x, y = polygon.exterior.xy
        plt.fill(x, y, alpha=0.3, color='orange', label=f'Intersection Region\nArea: {area:.2f}')

        
    # Calculate the centroid (center) of the polygon
    centroid = polygon.centroid if polygon and polygon.is_valid else None
    
    # Calculate the distance from each point to the centroid
    distances = []
    if centroid:
        for pt in points:
            point = Point(pt[0], pt[1])
            distance = centroid.distance(point)
            distances.append(distance)
            print(f"Distance from point ({pt[0]}, {pt[1]}) to the center ({centroid.x:.2f}, {centroid.y:.2f}): {distance:.2f}")
    
    # Print the area
    print(f"Area of the intersection region: {area:.2f}")
    
    plt.legend()
    plt.grid(True)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Lines, Points, and Intersection Region')
    plt.show()

# Example points
points = [
    [0, 0, 1.57],  # Vertical line starting at (0, 0)
    [1, 1, 2.5],   # Diagonal line starting at (1, 1)
    [-1, 1, 0.78]  # Another diagonal line starting at (-1, 1)
]

plot_lines_and_region(points, length=2)
