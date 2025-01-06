import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

# Example: 10 points with x, y, z coordinates
points = np.array([
    [1, 2, 3],
    [2, 3, 5],
    [3, 5, 6],
    [5, 7, 8],
    [6, 4, 10],
    [7, 9, 11],
    [8, 10, 12],
    [9, 12, 10],
    [10, 14, 18],
    [11, 15, 20]
])

def estimate_plane(points):
    # Extract x, y, z coordinates
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    
    # Design matrix for least squares
    A = np.c_[x, y, np.ones(points.shape[0])]
    
    # Solve for plane coefficients (a, b, d) using least squares
    # z = ax + by + d => a, b, d are plane parameters
    C, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
    
    # Plane coefficients
    a, b, d = C
    c = -1  # Assuming normalizing factor for z
    
    return a, b, c, d

# Estimate plane
a, b, c, d = estimate_plane(points)

# Plane equation
print(f"Plane equation: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")

print(math.degrees(math.acos((0*a+1*b+0*c)/(math.sqrt(1^2)*math.sqrt(a**2+b**2+c**2)))))

# Create grid for the plane
x_range = np.linspace(points[:, 0].min(), points[:, 0].max(), 10)
y_range = np.linspace(points[:, 1].min(), points[:, 1].max(), 10)
x_grid, y_grid = np.meshgrid(x_range, y_range)
z_grid = a * x_grid + b * y_grid + d  # Compute z values for the plane

# Plotting
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot points
ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r', label='Points')

# Plot plane
plane_surface = ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, color='blue')

# Add proxy artist for the plane in the legend
plane_proxy = plt.Line2D([0], [0], linestyle="none", marker="s", color="blue", alpha=0.5, label="Estimated Plane")
ax.legend(handles=[plane_proxy], loc='upper left')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Visualization of Points and Estimated Plane')

plt.show()
