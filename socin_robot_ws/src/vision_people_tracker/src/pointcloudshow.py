import open3d as o3d

# Load the saved point cloud
point_cloud = np.loadtxt("point_cloud.xyz")

# Extract XYZ and RGB
xyz = point_cloud[:, :3]
rgb = point_cloud[:, 3:] / 255.0  # Normalize colors to [0,1]

# Create Open3D point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(rgb)

# Visualize
o3d.visualization.draw_geometries([pcd], window_name="Point Cloud", width=800, height=600)
