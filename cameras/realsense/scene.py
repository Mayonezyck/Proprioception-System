import open3d as o3d
import numpy as np
from ultralytics import YOLO
import threading
import time

# Cite https://github.com/isl-org/Open3D/discussions/5953

class Scene:
    GEOM_NAME = "Global Map"
    def __init__(self,camera, update_delay = -1):
        self.global_map = o3d.geometry.PointCloud()
        self.camera = camera
        self.model = YOLO("yolo11x-seg.pt")

        self.update_delay = update_delay
        self.is_done = False
        self.lock = threading.Lock()

        self.app = o3d.visualization.gui.Application.instance
        self.window = self.app.create_window("Open3D Python App", width=800, height=600, x=0, y=30)
        self.window.set_on_close(self.on_main_window_closing)
        if self.update_delay < 0:
            self.window.set_on_tick_event(self.on_main_window_tick_event)

        self.widget = o3d.visualization.gui.SceneWidget()
        self.widget.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.widget.scene.set_background([1.0, 1.0, 1.0, 1.0])
        self.window.add_child(self.widget)

        self.add_depth_image()
        self.geom_mat = o3d.visualization.rendering.MaterialRecord()
        self.geom_mat.shader = 'defaultUnlit'
        self.geom_mat.point_size = 2.0
        self.widget.scene.add_geometry(self.GEOM_NAME, self.global_map, self.geom_mat)
        self.widget.setup_camera(60, self.widget.scene.bounding_box, [0, 0, 0])
        #self.setup_camera()

    def setup_camera(self):
        # Define the camera parameters
        field_of_view = 60.0  # Field of view in degrees
        look_at = [0, 0, -1]  # Look-at point
        up = [0, -1, 0]  # Up vector
        eye = [0, 0, 1]  # Camera position

        # Set up the camera
        self.widget.setup_camera(field_of_view, o3d.geometry.AxisAlignedBoundingBox(), eye)
        self.widget.scene.camera.look_at(look_at, eye, up)


    def startThread(self):
            if self.update_delay >= 0:
                threading.Thread(target=self.update_thread).start()

    def update_thread(self):
            def do_update():
                return self.update_point_cloud()

            while not self.is_done:
                time.sleep(self.update_delay)
                print("update_thread")
                with self.lock:
                    if self.is_done:  # might have changed while sleeping.
                        break
                    o3d.visualization.gui.Application.instance.post_to_main_thread(self.window, self.update_point_cloud)
    def on_main_window_closing(self):
        with self.lock:
            self.is_done = True
        return True  # False would cancel the close
    
    def on_main_window_tick_event(self):
        print("tick")
        return self.decaying()
    
    def update_point_cloud(self):
        self.add_depth_image()
        temp = self.global_map
        self.widget.enable_scene_caching(False)
        self.widget.scene.remove_geometry(self.GEOM_NAME)
        self.widget.scene.add_geometry(self.GEOM_NAME, temp, self.geom_mat)
        self.decaying()

    def decaying(self):
        print('Decaying colors...')
        colors = np.asarray(self.global_map.colors)
        print(colors)
        colors = np.clip(colors - 5 / 255.0, 0, 1)
        print(colors)
        self.global_map.colors = o3d.utility.Vector3dVector(colors)
        self.delete_points(0.2)
    
    def delete_points(self,death_threshold):
        colors = np.asarray(self.global_map.colors)
        points = np.asarray(self.global_map.points)
        mask = np.all(colors >= death_threshold / 255.0, axis=1)
        self.global_map.points = o3d.utility.Vector3dVector(points[mask])
        self.global_map.colors = o3d.utility.Vector3dVector(colors[mask])

    def trim_points(self, input_points, input_colors, input_percentage):
        num_points = len(input_points)
        num_points_to_keep = int(num_points * input_percentage)
        indices = np.random.choice(num_points, num_points_to_keep, replace=False)
        trimmed_points = input_points[indices]
        trimmed_colors = input_colors[indices]
        return trimmed_points, trimmed_colors

    def add_depth_image(self):
        intrinsics = self.camera.intrinsics
        depth_image = self.camera.get_depth_image()
        #color_image = self.camera.get_color_image()
        masks = self.getMask()
        # Convert depth image to point cloud
        height, width = depth_image.shape
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        points = []
        separate_points = []
        counter = 0
        for mask in masks:
            for v in range(height):
                for u in range(width):
                    if mask[v, u]:
                        z = depth_image[v, u] / 1000.0  # Convert from mm to meters
                        if z > 0:
                            x = (u - cx) * z / fx
                            y = (v - cy) * z / fy
                            points.append([x, y, z])
                            counter += 1
            separate_points.append(counter)
        self.color_assigned = self.assign_color(separate_points)
        print(self.color_assigned)
        # Create point cloud from points
        starting_point = 0  
        if points:
            for i in range(len(self.color_assigned)):
                points_parts = np.array(points[starting_point:separate_points[i]])
                starting_point = separate_points[i]
                #colors = np.full(points.shape, 127 / 255.0)  # Assign color with 127
                colors_parts = np.full(points_parts.shape, self.color_assigned[i])  # Assign color with RGBA
                # colors = color_image[mask]
                # colors = colors / 255.0  # Normalize colors to [0, 1]
                points_trimed, colors_trimed = self.trim_points(points_parts, colors_parts, 0.1)

                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(points_trimed)
                point_cloud.colors = o3d.utility.Vector3dVector(colors_trimed)
                
                # Merge with global map
                self.global_map += point_cloud

    def assign_color(self, separate_points):
        colors = []
        seed = [1,2,3,4,5,6,7,8,9,10]
        seed_count = 0 
        for _ in separate_points:
            np.random.seed(seed[seed_count])  # Set a random seed for reproducibility
            seed_count += 1
            color = np.random.uniform(3)  # Generate a random color in the specified range
            colors.append(color)
        return np.array(colors)

    def getMask(self):
        frames = self.camera.get_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image)
        result_mask = [] # = None
        if results is not None:
            for result in results:
                if result.masks is not None:
                    masks = result.masks.data 
        if masks is not None:
            for mask in masks:
                if result_mask is None:
                    result_mask.append(mask.cpu().numpy) ##mask.cpu().numpy()
                else:
                    #result_mask = np.logical_or(result_mask, mask.cpu().numpy())
                    result_mask.append(mask.cpu().numpy())
        return result_mask
    
    
    
