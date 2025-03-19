import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

# Configure the RealSense pipeline for depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming and get the depth sensor scale
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

# Create a pointcloud object
pc = rs.pointcloud()

# Define depth range (in meters) for segmentation (updated)
min_distance = 0.15
max_distance = 1.0

# Define field-of-view boundaries (ROI) in pixel coordinates
left_bound = 100
right_bound = 540
top_bound = 50
bottom_bound = 430

# Set up Open3D Visualizer for displaying the object's point cloud
vis = o3d.visualization.Visualizer()
vis.create_window("Object Point Cloud")
pcd_o3d = o3d.geometry.PointCloud()
geometry_added = False  # flag to track if pcd_o3d has been added to the visualizer

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_image_m = depth_image * depth_scale

        # Create a binary depth mask for the defined range
        depth_mask = cv2.inRange(depth_image_m, min_distance, max_distance)
        roi_depth_mask = depth_mask[top_bound:bottom_bound, left_bound:right_bound]
        roi_color_image = color_image[top_bound:bottom_bound, left_bound:right_bound]

        # Perform color segmentation (detecting red) in the ROI
        hsv_roi = cv2.cvtColor(roi_color_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
        color_mask = cv2.add(mask1, mask2)

        # Combine the depth and color masks
        combined_mask = cv2.bitwise_and(roi_depth_mask, color_mask)
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Select the largest contour as the candidate object
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            # Convert ROI coordinates to full image coordinates
            x_full = x + left_bound
            y_full = y + top_bound

            cv2.rectangle(color_image, (x_full, y_full), (x_full + w, y_full + h), (0, 255, 0), 2)
            cx = x_full + w // 2
            cy = y_full + h // 2
            cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)

            # Generate the full point cloud for the current frame
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(480, 640, 3)
            # Extract the object's point cloud from the bounding box area
            obj_points = vertices[y_full:y_full + h, x_full:x_full + w, :].reshape(-1, 3)
            # Filter out invalid points (with zero depth)
            valid_idx = np.where(obj_points[:, 2] > 0)[0]
            obj_points = obj_points[valid_idx]

            if obj_points.shape[0] > 0:
                pcd_o3d.points = o3d.utility.Vector3dVector(obj_points)
                if not geometry_added:
                    vis.add_geometry(pcd_o3d)
                    geometry_added = True
                else:
                    vis.update_geometry(pcd_o3d)
            else:
                if geometry_added:
                    vis.remove_geometry(pcd_o3d, reset_bounding_box=True)
                    geometry_added = False

            vis.poll_events()
            vis.update_renderer()

        # Draw the ROI boundary on the color image for reference
        cv2.rectangle(color_image, (left_bound, top_bound), (right_bound, bottom_bound), (0, 0, 255), 2)
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Combined Mask", combined_mask)

        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    pipeline.stop()
    vis.destroy_window()
    cv2.destroyAllWindows()
