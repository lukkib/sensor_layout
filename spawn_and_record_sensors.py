import carla
import numpy as np
import time
import cv2
import os
from threading import Thread

from carla import ColorConverter as cc

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

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
        out_streams = []

        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)
        world = client.get_world()
        map = world.get_map()

        # settings = world.get_settings()
        # mode = True
        # print(f'Synchronous mode: {mode}')
        # settings.synchronous_mode = mode
        # settings.fixed_delta_seconds = 0.05
        # world.apply_settings(settings)

        ego = None
        agent = None

        kml = None
        dir = None
        # print(world.get_actors().filter('sensor.*'))
        # print(world.get_actors().filter('vehicle.*'))

        instance_image_queue = None

        while True:
            time.sleep(1)

            vehicles = world.get_actors().filter('vehicle.*')
            current_time = time.strftime("%H:%M:%S", time.localtime())

            if len(vehicles) == 0:
                print(f'[{current_time}] Waiting for vehicles ...')
                if len(images_array) > 0:
                    ego = None
                    dir = None
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
                continue

            if len(cameras) == 0:
                index = 0

                for v in vehicles:
                    if v.attributes["role_name"] == "ego":
                        print('ego found')
                        ego = v

                        # attach camera
                        camera_bp = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
                        camera_bp.set_attribute('image_size_x', str(1920))
                        camera_bp.set_attribute('image_size_y', str(1080))
                        camera_bp.set_attribute('sensor_tick', str(0.01))
                        camera_position3 = carla.Transform(carla.Location(x=-6.0, y=0.0, z=2.5), carla.Rotation(pitch=0, yaw=0.0)) # Rearview
                        camera = world.spawn_actor(camera_bp, camera_position3, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
                        cameras.append(camera)

                        c = camera
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

                        daytime = time.strftime("%H_%M", time.localtime())
                        dir = f'./carla_recordings/{daytime}'
                        if not os.path.exists(f'./carla_recordings'):
                            os.mkdir(f'./carla_recordings')
                        if not os.path.exists(dir):
                            os.mkdir(dir)
                        out_dir = dir + f'/video_{index+1}.mkv'
                        out = cv2.VideoWriter(out_dir, cv2.VideoWriter_fourcc('M','J','P','G'), fps, (width, height))
                        out_streams.append(out)

                        if c.type_id.startswith('sensor.camera.semantic_segmentation'):
                            color_converter = cc.CityScapesPalette
                            print('Using semantic segmentation')
                        elif c.type_id.startswith('sensor.camera.instance_segmentation'):
                            print('Using instance segmentation')
                            color_converter = cc.Raw
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
