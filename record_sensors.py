import carla
import numpy as np
import time
import cv2
import os
import open3d as o3d
from matplotlib import cm

from carla import ColorConverter as cc

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

def lidar_callback(point_cloud, point_list):
    """Prepares a point cloud with intensity
    colors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))

    # Isolate the intensity and compute a color for it
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]

    # Isolate the 3D data
    points = data[:, :-1]

    # We're negating the y to correclty visualize a world that matches
    # what we see in Unreal since Open3D uses a right-handed coordinate system
    points[:, :1] = -points[:, :1]

    # # An example of converting points from sensor to vehicle space if we had
    # # a carla.Transform variable named "tran":
    # points = np.append(points, np.ones((points.shape[0], 1)), axis=1)
    # points = np.dot(tran.get_matrix(), points.T).T
    # points = points[:, :-1]

    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)

def addImageToStream(i, images_array, index, out, color_converter):

    i.convert(color_converter)

    array = np.frombuffer(i.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (i.height, i.width, 4))
    array = array[:, :, :3]
    #images_array[index].append(array)
    out.write(array)

    #print(len(np.frombuffer(i.raw_data, dtype=np.dtype("uint8"))))
    
    #images_array[index].append(np.copy(np.frombuffer(i.raw_data, dtype=np.dtype("uint8"))))

def main():
    try:
        out = None
        images_array = []
        width = 0
        height = 0
        fps = 0
        cameras = []
        lidars = []
        out_streams = []
        vis = None
        point_list = None

        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        map = world.get_map()
        ego = None
        agent = None
        dir = None
        frame = 0
        # print(world.get_actors().filter('sensor.*'))
        # print(world.get_actors().filter('vehicle.*'))

        while True:
            time.sleep(1)

            sensors = world.get_actors().filter('sensor.*')
            current_time = time.strftime("%H:%M:%S", time.localtime())

            if len(sensors) == 0:
                print(f'[{current_time}] Waiting for sensor...')
                if len(images_array) > 0:
                    ego = None
                    agent = None
                    for o in out_streams:
                        o.release()
                    out_streams.clear()

                    images_array_copy = []
                    for images in images_array:
                        images_array_copy.append(images.copy())
                        images.clear()
                    images_array.clear()

                # stop listening for data, since sensors already got destroyed
                # otherwise CARLA will throw an error
                for c in cameras: 
                    c.stop()
                cameras.clear()
                for l in lidars:
                    l.stop()
                if vis != None:
                    vis.destroy_window()
                    vis = None
                lidars.clear()
                frame = 0

                dir = None
                continue

            if len(lidars) == 0:
                if dir == None:
                    daytime = time.strftime("%H_%M", time.localtime())
                    dir = f'./videos/{daytime}'
                    if not os.path.exists(dir):
                        os.mkdir(dir)
                for l in sensors:
                    if not l.type_id.startswith('sensor.lidar'):
                        continue
                    print('Lidar sensor found')
                    lidars.append(l)

                    point_list = o3d.geometry.PointCloud()
                    lidars[0].listen(lambda data: lidar_callback(data, point_list))

                    vis = o3d.visualization.Visualizer()
                    vis.create_window(
                        window_name='Carla Lidar',
                        width=960,
                        height=540,
                        left=480,
                        top=270)
                    vis.get_render_option().background_color = [0.05, 0.05, 0.05]
                    vis.get_render_option().point_size = 1
                    vis.get_render_option().show_coordinate_frame = True

            if vis != None and len(lidars) > 0:
                if frame == 2:
                    vis.add_geometry(point_list)
                vis.update_geometry(point_list)

                vis.poll_events()
                vis.update_renderer()
                #if point_list != None:
                print("Write point cloud")
                o3d.io.write_point_cloud(dir + f"/point_cloud_{frame}.pcd", point_list)
                    #point_list = None

            # # This can fix Open3D jittering issues:
            #time.sleep(0.005)
            frame += 1

            if len(cameras) == 0: # init cameras
                index = 0
                if dir == None:
                    daytime = time.strftime("%H_%M", time.localtime())
                    dir = f'./videos/{daytime}'
                    if not os.path.exists(dir):
                        os.mkdir(dir)

                for c in sensors:
                    if not c.type_id.startswith('sensor.camera'):
                        continue          
                    print('Camera sensor found')
                    cameras.append(c)
                    
                    width = int(c.attributes['image_size_x'])
                    height = int(c.attributes['image_size_y'])
                    tick = c.attributes['sensor_tick']
                    if float(tick) > 0:
                        fps = int(1. / float(tick))
                    else:
                        fps = 100

                    print("FPS: ", fps)
                    print("Image size:", width, "x", height)

                    images = []
                    images_array.append(images)

                    out_dir = dir + f'/video_{index+1}.mkv'
                    out = cv2.VideoWriter(out_dir, cv2.VideoWriter_fourcc('M','J','P','G'), fps, (width, height))
                    out_streams.append(out)

                    if (c.type_id.startswith('sensor.camera.semantic_segmentation')):
                        color_converter = cc.CityScapesPalette
                    else:
                        color_converter = cc.Raw

                    if index == 0:
                        cameras[0].listen(lambda image: addImageToStream(image, images_array, 0, out_streams[0], color_converter))
                    elif index == 1:
                        cameras[1].listen(lambda image: addImageToStream(image, images_array, 1, out_streams[1], color_converter))
                    elif index == 2:
                        cameras[2].listen(lambda image: addImageToStream(image, images_array, 2, out_streams[2], color_converter))
                    elif index == 3:
                        cameras[3].listen(lambda image: addImageToStream(image, images_array, 3, out_streams[3], color_converter))
                    elif index == 4:
                        cameras[4].listen(lambda image: addImageToStream(image, images_array, 4, out_streams[4], color_converter))
                    elif index == 5:
                        cameras[5].listen(lambda image: addImageToStream(image, images_array, 5, out_streams[5], color_converter))
                    elif index == 6:
                        cameras[6].listen(lambda image: addImageToStream(image, images_array, 6, out_streams[6], color_converter))
                    elif index == 7:
                        cameras[7].listen(lambda image: addImageToStream(image, images_array, 7, out_streams[7], color_converter))
                    
                    index = index + 1               
    finally:
        pass

if __name__ == '__main__':
    main()
