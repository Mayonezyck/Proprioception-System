import pyrealsense2 as rs
import cv2
import numpy as np

class RealSenseCamera:
    def __init__(self):
        # Create a pipeline
        self.pipeline = rs.pipeline()
        
        # Create a config and configure the pipeline to stream
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(self.config)
        print("Camera started")

    def get_aligned_frames(self):
        # Create an align object
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            raise RuntimeError("Could not acquire aligned frames")

        return aligned_depth_frame, color_frame

    def display_frames(self):
        while True:
            aligned_depth_frame, color_frame = self.get_aligned_frames()

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))

            # Show images
            cv2.imshow('Aligned Frames', images)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
    def stop(self):
        # Stop streaming
        self.pipeline.stop()
        print("Camera stopped")

# Example usage
if __name__ == "__main__":
    camera = RealSenseCamera()
    # Add your code here to use the camera
    camera.display_frames()
    camera.stop()