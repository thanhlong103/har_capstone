import numpy as np
from sklearn.cluster import DBSCAN

# Example list of points (x, y, theta)
points = [
    [1, 2, 0.5],
    [2, 3, 0.6],
    [3, 4, 0.7],
    [10, 10, 1.0],  # outlier point
    [5, 6, 0.8],
    [6, 7, 1.1]
]

# Convert the points to a NumPy array for easier manipulation
points = np.array(points)

# Apply DBSCAN clustering
# eps is the maximum distance between two points to be considered as neighbors
# min_samples is the minimum number of points to form a cluster
dbscan = DBSCAN(eps=3, min_samples=2)
labels = dbscan.fit_predict(points)

# Print the labels assigned by DBSCAN
print("DBSCAN Labels:", labels)

# Filter out the points labeled as outliers (label = -1)
filtered_points = points[labels != -1]

# Output the filtered points
print("Filtered points (without outliers):")
print(filtered_points)
