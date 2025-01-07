import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.decomposition import PCA

# Points (nose, eyes, ears, shoulders, etc.)
points = np.array([
    [0.5, 0.37, 1.5],  # nose
    [0.55, 0.4, 1.6],  # left eye
    [0.45, 0.4, 1.6],  # right eye
    [0.4, 0.47, 1.5],  # left ear
    [0.6, 0.47, 1.5],  # right ear
    [0.2, 0.2, 1.3],  # left shoulder
    [0.8, 0.6, 1.3],  # right shoulder
    [0.1, 0.38, 1.0],  # left elbow
    [0.9, 0.38, 1.0],  # right elbow
    [0.0, 0.4, 0.55],  # left wrist
    [1.0, 0.4, 0.55],  # right wrist
    [0.2, 0.3, 0.7],  # left hip
    [0.8, 0.6, 0.7],  # right hip
    [0.25, 0.25, 0.4],  # left knee
    [0.75, 0.67, 0.4],  # right knee
    [0.25, 0.15, 0.0],  # left ankle
    [0.75, 0.77, 0.0],  # right ankle
])

# Define the connections between the keypoints (skeleton structure)
connections = [
    (0, 1), (0, 2),  # Nose to Left and Right Eyes
    (1, 3), (2, 4),  # Nose to Left and Right Ears
    (0, 5), (0, 6),  # Shoulders (Left and Right)
    (5, 6),  # Shoulders
    (5, 7), (6, 8),  # Shoulders to Elbows
    (7, 9), (8, 10),  # Elbows to Wrists
    (5, 11), (6, 12),  # Shoulders to Hips
    (11, 12),  # Hips
    (11, 13), (12, 14),  # Hips to Knees
    (13, 15), (14, 16),  # Knees to Ankles
]

# Perform PCA to find the normal of the plane
pca = PCA(n_components=2)
pca.fit(points)
normal = np.cross(pca.components_[0], pca.components_[1])
print(pca.components_)

# Find a point on the plane (mean of the points)
point_on_plane = np.mean(points, axis=0)

# Plane equation: Ax + By + Cz = D
A, B, C = normal
D = np.dot(normal, point_on_plane)

# Create a grid of points to plot the plane
x = np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 10)
y = np.linspace(np.min(points[:, 1]), np.max(points[:, 1]), 10)
X, Y = np.meshgrid(x, y)
Z = (-A * X - B * Y + D) / C

# Plot the points and the plane
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot points
ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r', label='Points')

# Plot plane
ax.plot_surface(X, Y, Z, color='b', alpha=0.3, label='Estimated Plane')

# Set the same scale for all axes
max_range = np.ptp(points)  # peak-to-peak (max-min) range of the points
center = np.mean(points, axis=0)  # center of the points

# Set the limits for each axis
ax.set_xlim(center[0] - max_range / 2, center[0] + max_range / 2)
ax.set_ylim(center[1] - max_range / 2, center[1] + max_range / 2)
ax.set_zlim(center[2] - max_range / 2, center[2] + max_range / 2)

# # Draw 2 lines crossing the center of the plane
# # Line 1: vary y, fix x and z
# y_vals = np.linspace(center[1] - max_range / 2, center[1] + max_range / 2, 10)
# x_vals = np.full_like(y_vals, center[0])
# z_vals = (-A * x_vals - B * y_vals + D) / C
# ax.plot(x_vals, y_vals, z_vals, color='g', label='Line 1')

# # Line 2: vary x, fix y and z
# x_vals = np.linspace(center[0] - max_range / 2, center[0] + max_range / 2, 10)
# y_vals = np.full_like(x_vals, center[1])
# z_vals = (-A * x_vals - B * y_vals + D) / C
# ax.plot(x_vals, y_vals, z_vals, color='y', label='Line 2')

# Create the plane perpendicular to the Ox axis
# This plane has the normal vector along the X-axis, i.e., (1, 0, 0)
# The plane equation is of the form: y * B + z * C = D_2
# To cross the previous vertical line, we solve for D_2 based on the center's y and z values

D_2 = center[1] * B + center[2] * C  # Calculate D_2 using the center


# Add plane perpendicular to Oy at the center of the estimated plane
x_vals = np.linspace(center[0] - max_range / 2, center[0] + max_range / 2, 10)
z_vals = np.linspace(center[2] - max_range / 2, center[2] + max_range / 2, 10)
X, Z = np.meshgrid(x_vals, z_vals)
Y = np.full_like(X, center[1])
ax.plot_surface(X, Y, Z, color='y', alpha=0.3, label='Perpendicular Plane to Ox')

# Create a grid for the perpendicular plane
y_vals = np.linspace(center[1] - max_range / 2, center[1] + max_range / 2, 10)
z_vals = np.linspace(center[2] - max_range / 2, center[2] + max_range / 2, 10)
Y, Z = np.meshgrid(y_vals, z_vals)

# Calculate corresponding X values based on the plane equation
X_vals = np.full_like(Y, center[0])

# Plot the perpendicular plane
ax.plot_surface(X_vals, Y, Z, color='r', alpha=0.3, label='Perpendicular Plane to Oy')

# Compute PC1 and PC2 directions
pc1_direction = pca.components_[0]
pc2_direction = pca.components_[1]

# Plot the principal components as vectors
ax.quiver(point_on_plane[0], point_on_plane[1], point_on_plane[2], 
          pc1_direction[0], pc1_direction[1], pc1_direction[2], 
          length=0.5, color='g', label='PC1')

ax.quiver(point_on_plane[0], point_on_plane[1], point_on_plane[2], 
          pc2_direction[0], pc2_direction[1], pc2_direction[2], 
          length=0.5, color='b', label='PC2')

# Plot the translated skeleton connections (lines between keypoints)
for start, end in connections:
    ax.plot([points[start, 0], points[end, 0]],
            [points[start, 1], points[end, 1]],
            [points[start, 2], points[end, 2]], color='r')
    
# Normal vector of the estimated plane
normal_estimated = np.array([A, B, C])
# Normal vector of the plane perpendicular to Oy
normal_perpendicular = np.array([0, 1, 0])
# Calculate the dot product
dot_product = np.dot(normal_estimated, normal_perpendicular)

# Calculate the magnitudes of the normal vectors
magnitude_estimated = np.linalg.norm(normal_estimated)
magnitude_perpendicular = np.linalg.norm(normal_perpendicular)

ax.quiver(point_on_plane[0], point_on_plane[1], point_on_plane[2], 
          normal[0], normal[1], normal[2], 
          length=0.5, color='r', label='Facing Direction')

# Calculate the cosine of the angle
cos_angle = dot_product / (magnitude_estimated * magnitude_perpendicular)

# Calculate the angle in radians
angle_radians = np.arccos(cos_angle)

# Convert the angle to degrees
angle_degrees = np.degrees(angle_radians)

# Add the angle text annotation on the plot
angle_text = f"{angle_degrees:.2f}°"
ax.text(center[0] + 0.1, center[1] + 0.1, center[2] + 0.1, angle_text, color='black', fontsize=12)
    
# Labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

print(f"The angle between the estimated plane and the plane perpendicular to Oy is {angle_degrees:.2f} degrees.")
plt.show()
