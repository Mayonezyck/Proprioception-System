import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

class RealSenseCamera:
    def __init__(self, config_data):
        self.config_data = config_data
        # Create a pipeline
        self.pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        self.config = rs.config()
        if self.config_data['depth']['enabled']:
            self.config.enable_stream(rs.stream.depth, 
                  self.config_data['resolution']['width'], 
                  self.config_data['resolution']['height'], 
                  rs.format.z16, 
                  self.config_data['frame_rate'])
        if self.config_data['color']['enabled']:
            self.config.enable_stream(rs.stream.color, 
                  self.config_data['resolution']['width'], 
                  self.config_data['resolution']['height'], 
                  rs.format.bgr8, 
                  self.config_data['frame_rate'])
        self.align = rs.align(rs.stream.color)
        
    
    def start(self):
        print("Starting camera stream...")
        # Start streaming
        self.pipeline.start(self.config)
        profile = self.pipeline.get_active_profile()
        depth_stream = profile.get_stream(rs.stream.depth)
        self.intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
        print("Camera started")

    def get_color_image(self):
        frames = self.get_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Could not acquire color frame.")
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_depth_image(self):
        frames = self.get_frames()
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            raise RuntimeError("Could not acquire depth frame.")
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    def get_frames(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames(1000)
        return frames

    def stop(self): 
        print("Stopping camera stream...")
        self.pipeline.stop()
        print("Camera stopped")