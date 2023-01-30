import carla
import numpy as np
import time
import cv2
import os
from threading import Thread
import simplekml
import utm

from carla import ColorConverter as cc

from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

def writeVideos(images_array, fps, width, height):
    count = 1
    daytime = time.strftime("%H_%M", time.localtime())

    for imgs in images_array:
        print(f'Saving video #{count} to disk ...')
        dir = f'./videos/{daytime}'
        if not os.path.exists(dir):
            os.mkdir(dir)
        out_dir = dir + f'/moskito_{count}.mkv'
        out = cv2.VideoWriter(out_dir, cv2.VideoWriter_fourcc('M','J','P','G'), fps, (width, height))
        for i in imgs:
            #i.convert(cc.Raw)
            # array = np.frombuffer(i.raw_data, dtype=np.dtype("uint8"))
            # array = np.reshape(array, (i.height, i.width, 4))
            # array = array[:, :, :3]
            # out.write(array)
            #print(len(i))
            #array = np.frombuffer(i, dtype=np.dtype("uint8"))
            array = np.reshape(i, (height, width, 4))
            array = array[:, :, :3]
            out.write(array)

            #out.write(i)
        imgs.clear()
        out.release()
        print(f'... video #{count} done!')
        count = count + 1
    images_array.clear()

def addImageToStream(i, images_array, index, out):
    i.convert(cc.Raw)
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
        ego = None
        agent = None

        kml = None
        dir = None
        # print(world.get_actors().filter('sensor.*'))
        # print(world.get_actors().filter('vehicle.*'))

        while True:
            time.sleep(1)

            sensors = world.get_actors().filter('sensor.*')
            current_time = time.strftime("%H:%M:%S", time.localtime())

            if len(sensors) == 0:
                # stop listening for data, since sensors already got destroyed
                # otherwise CARLA will throw an error
                for c in cameras: 
                    c.stop()
                cameras.clear()
                
                print(f'[{current_time}] Waiting for camera sensor...')
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
                continue

            if len(cameras) == 0:
                index = 0
                for c in sensors:              
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

                    daytime = time.strftime("%H_%M", time.localtime())
                    dir = f'./videos/{daytime}'
                    if not os.path.exists(dir):
                        os.mkdir(dir)
                    out_dir = dir + f'/moskito_{index+1}.mkv'
                    out = cv2.VideoWriter(out_dir, cv2.VideoWriter_fourcc('M','J','P','G'), fps, (width, height))
                    out_streams.append(out)

                    if index == 0:
                        cameras[0].listen(lambda image: addImageToStream(image, images_array, 0, out_streams[0]))
                    elif index == 1:
                        cameras[1].listen(lambda image: addImageToStream(image, images_array, 1, out_streams[1]))
                    elif index == 2:
                        cameras[2].listen(lambda image: addImageToStream(image, images_array, 2, out_streams[2]))
                    elif index == 3:
                        cameras[3].listen(lambda image: addImageToStream(image, images_array, 3, out_streams[3]))
                    elif index == 4:
                        cameras[4].listen(lambda image: addImageToStream(image, images_array, 4, out_streams[4]))
                    elif index == 5:
                        cameras[5].listen(lambda image: addImageToStream(image, images_array, 5, out_streams[5]))
                    elif index == 6:
                        cameras[6].listen(lambda image: addImageToStream(image, images_array, 6, out_streams[6]))
                    elif index == 7:
                        cameras[7].listen(lambda image: addImageToStream(image, images_array, 7, out_streams[7]))
                    
                    index = index + 1               
    finally:
        pass

if __name__ == '__main__':
    main()