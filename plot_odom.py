import numpy as np
import matplotlib.pyplot as plt
import csv

# Load odometry data from CSV file
csv_filename = "odom_log.csv"
timestamps, x_vals, y_vals, theta_vals = [], [], [], []

with open(csv_filename, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # Skip header
    for row in csv_reader:
        timestamps.append(float(row[0]))
        x_vals.append(float(row[1]))
        y_vals.append(float(row[2]))
        theta_vals.append(float(row[3]))

# Convert to NumPy arrays
x_vals = np.array(x_vals)
y_vals = np.array(y_vals)
theta_vals = np.array(theta_vals)

# Plot the robot's trajectory
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, 'b-', label="Path")  # Plot path

# Add arrows to show orientation
arrow_scale = 1.2  # Length of arrows
for i in range(0, len(x_vals), max(1, len(x_vals)//150)):  # Plot arrows at intervals
    dx = arrow_scale * np.cos(theta_vals[i])
    dy = arrow_scale * np.sin(theta_vals[i])
    plt.arrow(x_vals[i], y_vals[i], dx, dy, head_width=0.15, head_length=0.15, fc='r', ec='r')

# Labels and display
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Robot Path from Odometry")
plt.legend()
plt.grid()
plt.axis("equal")  # Keep aspect ratio square
plt.show()
