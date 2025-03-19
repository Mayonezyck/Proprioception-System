import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
# Enable the depth stream (640x480 at 30 fps)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# Enable the color stream (640x480 at 30 fps)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image to HSV for easier color segmentation
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define HSV range for the target color (red in this example)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        
        # Create masks for red color (combining two ranges to cover red hue)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Choose the largest contour as the target object
            c = max(contours, key=cv2.contourArea)
            # Get bounding box for the detected contour
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Compute the center of the bounding box
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)
            
            # Retrieve the depth at the center pixel (in meters)
            depth = depth_frame.get_distance(cx, cy)
            cv2.putText(color_image, f"Depth: {depth:.2f}m", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Get the camera intrinsics to convert pixel coordinates to real world coordinates
            profile = pipeline.get_active_profile()
            depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            # Deproject from pixel to 3D point (result is in meters: [x, y, z])
            object_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth)
            cv2.putText(color_image,
                        f"World: ({object_3d[0]:.2f}, {object_3d[1]:.2f}, {object_3d[2]:.2f})",
                        (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the color image and the mask
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1)
        if key == 27:  # Exit on ESC key
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

