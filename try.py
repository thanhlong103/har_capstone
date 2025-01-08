import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def estimate_plane_pca(points):
    """
    Estimates a plane from a given set of 3D points using PCA.
    
    Parameters:
        points (numpy.ndarray): A (N, 3) array of 3D points.
        
    Returns:
        plane_normal (numpy.ndarray): A (3,) vector representing the plane's normal.
        plane_point (numpy.ndarray): A (3,) vector representing a point on the plane (centroid).
    """
    # Step 1: Compute the centroid (mean) of the points
    centroid = np.mean(points, axis=0)
    
    # Step 2: Center the points by subtracting the centroid
    centered_points = points - centroid
    
    # Step 3: Compute the covariance matrix
    covariance_matrix = np.cov(centered_points, rowvar=False)
    
    # Step 4: Perform eigen decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
    
    # Step 5: The normal vector to the plane is the eigenvector associated with the smallest eigenvalue
    normal_vector = eigenvectors[:, np.argmin(eigenvalues)]
    
    return normal_vector, centroid

def plot_plane_and_points(points, normal_vector, plane_point):
    """
    Plots the 3D points and the estimated plane.
    
    Parameters:
        points (numpy.ndarray): A (N, 3) array of 3D points.
        normal_vector (numpy.ndarray): The normal vector of the plane.
        plane_point (numpy.ndarray): A point on the plane.
    """
    # Create a grid of points on the plane
    d = -np.dot(normal_vector, plane_point)  # Plane equation: ax + by + cz + d = 0
    xx, yy = np.meshgrid(np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 10),
                         np.linspace(np.min(points[:, 1]), np.max(points[:, 1]), 10))
    zz = (-normal_vector[0] * xx - normal_vector[1] * yy - d) / normal_vector[2]

    # Plot the points and the plane
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='blue', label='3D Points')
    plane_surface = ax.plot_surface(xx, yy, zz, color='orange', alpha=0.5)

    # Highlight the centroid
    ax.scatter(plane_point[0], plane_point[1], plane_point[2], color='red', s=100, label='Centroid')

    # Create a proxy artist for the plane to include in the legend
    from matplotlib.patches import Patch
    plane_patch = Patch(color='orange', label='Estimated Plane')
    ax.legend(handles=[plane_patch], loc='upper right')

    # Set labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    plt.show()

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

# Estimate the plane using PCA
normal_vector, plane_point = estimate_plane_pca(points)

# Print the results
print("Plane Normal Vector:", normal_vector)
print("Point on Plane (Centroid):", plane_point)

# Plot the points and the estimated plane
plot_plane_and_points(points, normal_vector, plane_point)
