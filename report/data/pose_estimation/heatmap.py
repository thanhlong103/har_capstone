import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Load ground truth and detected keypoints
gt_file = "ground_truth.csv"
pred_file = "detected.csv"

gt_df = pd.read_csv(gt_file)
pred_df = pd.read_csv(pred_file)

# Ensure both files have the same frames
if not (gt_df["Frame Index"].equals(pred_df["Frame Index"])):
    print("Error: Frame indices do not match between ground truth and predictions.")
    exit()

# Number of keypoints
num_keypoints = 17  # Change this if needed

# Compute error metrics
mae_list = []
rmse_list = []
pe_list = []
keypoint_labels = [f"Keypoint {i}" for i in range(num_keypoints)]

for i in range(num_keypoints):
    # Extract X and Y coordinates for each keypoint
    gt_x = gt_df[f"Point {i+1} X"] / 640
    gt_y = (480 - gt_df[f"Point {i+1} Y"]) / 480

    pred_x = pred_df[f"Point {i+1} X"] / 640
    pred_y = pred_df[f"Point {i+1} Y"] / 480

    # Compute errors
    error_x = pred_x - gt_x
    error_y = pred_y - gt_y
    error = np.sqrt(error_x**2 + error_y**2)  # Euclidean distance error per keypoint
    gt_euclidean = np.sqrt(gt_x**2 + gt_y**2)

    # Compute MAE, RMSE, and PE
    mae = np.mean(np.abs(error))  # Mean Absolute Error
    rmse = np.sqrt(np.mean(error**2))  # Root Mean Squared Error
    percentage_error = np.mean(error / gt_euclidean)

    mae_list.append(mae)
    rmse_list.append(rmse)
    pe_list.append(percentage_error)

# Compute overall errors
overall_mae = np.mean(mae_list)
overall_rmse = np.mean(rmse_list)
overall_percentage = np.mean(pe_list)

print(f"\nOverall MAE: {overall_mae:.4f}")
print(f"Overall RMSE: {overall_rmse:.4f}")
print(f"Overall PE: {overall_percentage:.4f}")

# Convert data into DataFrame for plotting
error_df = pd.DataFrame(
    {"Keypoint": keypoint_labels, "MAE": mae_list, "RMSE": rmse_list, "PE": pe_list}
)

# Set seaborn style
sns.set(style="whitegrid")

# ---- HEATMAP ----
plt.figure(figsize=(18, 6))
sns.heatmap(
    error_df.set_index("Keypoint").T,
    cmap="coolwarm",
    annot=True,
    fmt=".3f",
    linewidths=0.5,
)
plt.title("Heatmap of Error Metrics per Keypoint")
plt.ylabel("Error Type")
plt.xlabel("Keypoints")
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
