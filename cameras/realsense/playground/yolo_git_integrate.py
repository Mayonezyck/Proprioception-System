import cv2
import numpy as np
import pyrealsense2 as rs
#import torch
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("yolo11x-seg.pt")

# Set up the RealSense D455 camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

#write your yolov5 depth scale here

depth_scale = 0.0010000000474974513

# Main loop
while True:
    
    # Get the latest frame from the camera
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    # Convert the frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Convert the color image to grayscale
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Convert the depth image to meters
    depth_image = depth_image * depth_scale

    # Detect objects using YOLOv5
    results = model(color_image)

    # # Process the results
    # for result in results:
    #     boxes = result.boxes.xyxy  # Get the bounding boxes
    #     for box in boxes:
    #         x1, y1, x2, y2 = map(int, box)  # Convert to integer coordinates
    #         cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw the bounding box
    #         cv2.putText(color_image, "Object", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Label the object
    # # Show the image
    if results is not None:
        for result in results:
            if result.masks is not None:
                xy = result.masks.xy  # mask in polygon format
                xyn = result.masks.xyn  # normalized
                masks = result.masks.data  # mask in matrix format (num_objects x H x W)
    # cv2.imshow("Color Image", color_image)
    # Apply masks to the color image
    if masks is not None:
        for mask in masks:
            mask = mask.cpu().numpy()
            color_image[mask == 1] = [0, 255, 0]  # Apply green color to the masked area

    # Show the masked image
    cv2.imshow("Masked Image", color_image)
    cv2.waitKey(1)

# Release the VideoWriter object
out.release()