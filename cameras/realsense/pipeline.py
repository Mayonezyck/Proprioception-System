from cam_realsense import RealSenseCamera
from scene import Scene
import numpy as np
import yaml 
import open3d as o3d

def main():
    # Initialize the camera
    
    with open('/home/yicheng/Github/deformable_explore/cameras/realsense/CONFIG.yaml', 'r') as file:
        camera_config = yaml.safe_load(file)['camera']
    # Start the camera stream
    camera = RealSenseCamera(camera_config)
    camera.start()
    o3d.visualization.gui.Application.instance.initialize()
    thread_delay = 0.1
    use_tick = -1
    scene = Scene(camera, thread_delay)
    scene.startThread()
    o3d.visualization.gui.Application.instance.run()


    #scene = Scene(camera)
    #scene.test_display_static_pointcloud()
    #scene.display()
    camera.stop()
    result = scene.return_map()
    object_List = scene.getObjetList()
    print(object_List)
    print(result)
 
    #o3d.visualization.draw_geometries([captured_point_cloud])


if __name__ == "__main__":
    main()