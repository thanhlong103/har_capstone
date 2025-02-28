import numpy as np
import matplotlib.pyplot as plt
import cv2
import yaml
import csv

# Load map metadata from YAML
def load_yaml(yaml_filename):
    with open(yaml_filename, 'r') as yaml_file:
        return yaml.safe_load(yaml_file)

# Convert world coordinates (x, y) to image pixel coordinates
def world_to_pixel(x, y, map_metadata):
    resolution = map_metadata['resolution']
    origin_x, origin_y = -3.0, -2.5974 
    # origin_x, origin_y = 0.0, 0.0

    
    # Convert world coordinates to pixel coordinates
    pixel_x = int((x - origin_x) / resolution)
    pixel_y = int((y - origin_y) / resolution)
    
    # Flip y-axis because images use a different origin (top-left)
    pixel_y = map_height - pixel_y
    
    return pixel_x, pixel_y

# Load the map image
map_filename = "../../../socin_robot_ws/src/turtlebot3_navigation2/map/sim_map.pgm"  # Change to your map file path
yaml_filename = "../../../socin_robot_ws/src/turtlebot3_navigation2/map/sim_map.yaml"  # Change to your YAML file path
# odom_csv_filename = "../../odom_log.csv"
odom_csv_filename = "../../data/sim_run/metric_ESP.csv"

map_image = cv2.imread(map_filename, cv2.IMREAD_GRAYSCALE)  # Load as grayscale
map_metadata = load_yaml(yaml_filename)

# Get map dimensions
map_height, map_width = map_image.shape

# Load odometry data
x_vals, y_vals, theta_vals = [], [], []

with open(odom_csv_filename, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # Skip header
    for row in csv_reader:
        x_vals.append(float(row[5]))  # x
        y_vals.append(float(row[6]))  # y
        theta_vals.append(float(row[7]))  # theta

# Convert trajectory points to pixel coordinates
trajectory_pixels = [world_to_pixel(x, y, map_metadata) for x, y in zip(x_vals, y_vals)]

# Convert to NumPy array
trajectory_pixels = np.array(trajectory_pixels)

# Convert grayscale map to BGR for color overlay
map_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

# Draw trajectory on the map
for i in range(1, len(trajectory_pixels)):
    cv2.line(map_bgr, tuple(trajectory_pixels[i-1]), tuple(trajectory_pixels[i]), (0, 0, 255), 2)

# # Draw orientation arrows
# arrow_scale = 15  # Scale for arrows
# for i in range(0, len(trajectory_pixels), max(1, len(trajectory_pixels) // 100)):  # Reduce arrows for clarity
#     px, py = trajectory_pixels[i]
#     dx = int(arrow_scale * np.cos(theta_vals[i]))
#     dy = int(arrow_scale * np.sin(theta_vals[i]))
#     cv2.arrowedLine(map_bgr, (px, py), (px + dx, py + dy), (255, 0, 0), 2, tipLength=0.3)

# Display the final image
plt.figure(figsize=(8, 6))
plt.imshow(map_bgr)
plt.title("Robot Path on Map")
plt.axis("off")
plt.show()

# Save the overlay image
cv2.imwrite("map_with_path.png", map_bgr)
