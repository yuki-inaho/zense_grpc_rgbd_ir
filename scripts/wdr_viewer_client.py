#! /usr/bin/python
#  capture and visualize WDR-Depth images using zense

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
           ] # Message size is up to 10MB


class WDRImageManager:
    def __init__(self, options):
        self.channel = grpc.insecure_channel('localhost:50051',
                                             options=options)
        self.rgb_img = None
        self.depth_img = None

    def update_depth_range1(self, response):
        w = response.image_depth_range1.width
        h = response.image_depth_range1.height
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_depth_range1.data, np.uint16)
        self.img_depth_range1 = img_np.reshape(h, w)
        return True

    def update_depth_range2(self, response):
        w = response.image_depth_range2.width
        h = response.image_depth_range2.height
        if w == 0:
            return False
        img_np = np.frombuffer(response.image_depth_range2.data, np.uint16)
        self.img_depth_range2 = img_np.reshape(h, w)
        return True

    def update(self):
        stub = image_pb2_grpc.ImageServiceStub(self.channel)
        response = stub.SendWDRImage(image_pb2.ImageRequest())
        status = self.update_depth_range1(response)
        status &= self.update_depth_range2(response)
        return status

    def depth_image_colorized(self, depth_img):
        depth_img_colorized = np.zeros(
            [depth_img.shape[0], depth_img.shape[1], 3]).astype(np.uint8)
        depth_img_colorized[:, :, 1] = 255
        depth_img_colorized[:, :, 2] = 255

        _depth_img_zense_hue = depth_img.copy().astype(np.float32)
        _depth_img_zense_hue[np.where(_depth_img_zense_hue > 2000)] = 0
        zero_idx = np.where((_depth_img_zense_hue > 2000)
                            | (_depth_img_zense_hue == 0))
        _depth_img_zense_hue *= 255.0 / 2000.0

        depth_img_colorized[:, :, 0] = _depth_img_zense_hue.astype(np.uint8)
        depth_img_colorized = cv2.cvtColor(depth_img_colorized,
                                           cv2.COLOR_HSV2RGB)
        depth_img_colorized[zero_idx[0], zero_idx[1], :] = 0
        return depth_img_colorized

    @property
    def depth_image_range1(self):
        return self.img_depth_range1

    @property
    def depth_image_range2(self):
        return self.img_depth_range2

    @property
    def depth_image_range1_colorized(self):
        return self.depth_image_colorized(self.img_depth_range1)

    @property
    def depth_image_range2_colorized(self):
        return self.depth_image_colorized(self.img_depth_range2)


#
# main rootine
#


zense_mng = WDRImageManager(options)
cvui.init(WINDOW_NAME)

key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    status = zense_mng.update()
    if status:
        depth_img_r1_colorized = zense_mng.depth_image_range1_colorized
        depth_img_r2_colorized = zense_mng.depth_image_range2_colorized

        depth_img_r1_resized = cv2.resize(depth_img_r1_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        depth_img_r2_resized = cv2.resize(depth_img_r2_colorized,
                                          (IMAGE_WIDTH, IMAGE_HEIGHT))
        frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH * 2, 3), np.uint8)
        frame[0:IMAGE_HEIGHT, 0:IMAGE_WIDTH, :] = depth_img_r1_resized
        frame[0:IMAGE_HEIGHT,
              IMAGE_WIDTH:IMAGE_WIDTH * 2, :] = depth_img_r2_resized

        cvui.update()
        cv2.imshow(WINDOW_NAME, frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

cv2.destroyAllWindows()
