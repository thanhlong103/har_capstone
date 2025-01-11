import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example set of 3D points (x, y, z)
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

# Least Squares Fit to estimate the plane
# Plane equation: Ax + By + Cz + D = 0
# We need to solve for A, B, C, and D

# Create the matrix A for the linear system (x, y, z, 1)
A = np.c_[points, np.ones(points.shape[0])]

# Solve the linear system (A.T * A) * plane_params = A.T * b
# Where b is the vector of zeros since all points lie on the plane
b = np.zeros(points.shape[0])

# Solve for plane parameters using least squares
plane_params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

# Plane parameters (A, B, C, D)
A_param, B_param, C_param, D_param = plane_params

# Plane equation is: Ax + By + Cz + D = 0
print(f"Plane equation: {A_param:.2f}x + {B_param:.2f}y + {C_param:.2f}z + {D_param:.2f} = 0")

# Create a grid to plot the plane
xx, yy = np.meshgrid(np.linspace(min(points[:, 0]), max(points[:, 0]), 10),
                     np.linspace(min(points[:, 1]), max(points[:, 1]), 10))
zz = (-A_param * xx - B_param * yy - D_param) / C_param

# Plot the 3D points and the fitted plane
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r', label='Data points')
ax.plot_surface(xx, yy, zz, alpha=0.3, color='b', label='Fitted Plane')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()