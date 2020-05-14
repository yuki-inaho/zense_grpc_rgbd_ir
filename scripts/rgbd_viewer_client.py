#! /usr/bin/python
#  capture and visualize RGB-D images using zense

import grpc
import image_pb2
import image_pb2_grpc
import numpy as np
import cv2
import cvui

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
           ]  # Message size is up to 10MB


class RGBDImageManager:
    def __init__(self, options):
        self.channel = grpc.insecure_channel(
            'localhost:50051', options=options)
        self.rgb_img = None
        self.depth_img = None

    def update_rgb(self, response):
        w = response.image_rgb.width
        h = response.image_rgb.height
        c = response.image_rgb.channel
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_rgb.data, np.uint8)
        self.img_rgb = img_np.reshape(h, w, c)
        return True

    def update_depth(self, response):
        w = response.image_depth.width
        h = response.image_depth.height
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_depth.data, np.uint16)
        self.img_depth = img_np.reshape(h, w)
        return True

    def update(self):
        stub = image_pb2_grpc.ImageServiceStub(self.channel)
        response = stub.SendRGBDImage(image_pb2.ImageRequest())
        status = self.update_rgb(response)
        status &= self.update_depth(response)
        return status

    @property
    def rgb_image(self):
        return self.img_rgb

    @property
    def depth_image(self):
        return self.img_depth

    @property
    def depth_image_colorized(self):
        depth_img_colorized = np.zeros(
            [self.img_depth.shape[0], self.img_depth.shape[1],
             3]).astype(np.uint8)
        depth_img_colorized[:, :, 1] = 255
        depth_img_colorized[:, :, 2] = 255

        _depth_img_zense_hue = self.img_depth.copy().astype(np.float32)
        _depth_img_zense_hue[np.where(_depth_img_zense_hue > 2000)] = 0
        zero_idx = np.where((_depth_img_zense_hue > 2000)
                            | (_depth_img_zense_hue == 0))
        _depth_img_zense_hue *= 255.0 / 2000.0

        depth_img_colorized[:, :, 0] = _depth_img_zense_hue.astype(np.uint8)
        depth_img_colorized = cv2.cvtColor(depth_img_colorized,
                                           cv2.COLOR_HSV2RGB)
        depth_img_colorized[zero_idx[0], zero_idx[1], :] = 0

        return depth_img_colorized


#
# main rootine
#


zense_mng = RGBDImageManager(options)

cvui.init(WINDOW_NAME)
key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        rgb_img = zense_mng.rgb_image
        depth_img_colorized = zense_mng.depth_image_colorized

        rgb_img_resized = cv2.resize(rgb_img, (IMAGE_WIDTH, IMAGE_HEIGHT))
        depth_img_resized = cv2.resize(depth_img_colorized,
                                       (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH * 2, 3), np.uint8)
        frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = rgb_img_resized
        frame[0:IMAGE_HEIGHT, IMAGE_WIDTH:IMAGE_WIDTH * 2, :] = depth_img_colorized
        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
