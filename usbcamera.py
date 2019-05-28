import os
import time
import numpy as np
from PIL import Image
import glob

class BaseCamera:

    def run_threaded(self):
        return self.frame

class USBCamera(BaseCamera):
    def __init__(self, resolution = (160, 120), framerate = 20):

        import pygame
        import pygame.camera
        import subprocess
        subprocess.call(["v4l2-ctl --set-ctrl power_line_frequency=1"],shell=True)

        super().__init__()

        pygame.init()
        pygame.camera.init()
        l = pygame.camera.list_cameras()
        self.cam = pygame.camera.Camera(l[0], resolution, "RGB")
        self.resolution = resolution
        self.cam.start()
        self.framerate = framerate

        # initialize variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True

        # When using PIL
        # image_data = camFront.read_and_queue()
        # # print(image_data)
        # camFront.close()
        # self.frame = Image.frombytes("RGB", (size_x, size_y), image_data)
        # return np.asarray(self.frame)

        print('USB Camera loaded..')

        # time.sleep(2)

    def update(self):
        from datetime import datetime, timedelta
        import pygame.image
        while self.on:
            start = datetime.now()

            if self.cam.query_image():
                # snapshot = self.cam.get_image()
                # self.frame = list(pygame.image.tostring(snapshot, "RGB", False))
                snapshot = self.cam.get_image()
                snapshot1 = pygame.transform.scale(snapshot, self.resolution)
                self.frame = pygame.surfarray.pixels3d(pygame.transform.rotate(pygame.transform.flip(snapshot1, True, False), 90))

            stop = datetime.now()
            s = 1 / self.framerate - (stop - start).total_seconds()
            if s > 0:
                time.sleep(s)

        self.cam.stop()

    def run_threaded(self):
        return self.frame

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping Webcam')
        time.sleep(.5)

