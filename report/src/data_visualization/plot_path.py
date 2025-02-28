import numpy as np
import matplotlib.pyplot as plt
import csv


odom_csv_filename = "../../data/sim_run/metric_HAR.csv"

# Load odometry data
x_vals, y_vals, theta_vals = [], [], []

with open(odom_csv_filename, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    next(csv_reader)  # Skip header
    for row in csv_reader:
        x_vals.append(float(row[5]))  # x
        y_vals.append(float(row[6]))  # y
        theta_vals.append(float(row[7]))  # theta

plt.figure(figsize=(20,20))
plt.plot(x_vals, y_vals, linewidth=5, color="red")
plt.show()