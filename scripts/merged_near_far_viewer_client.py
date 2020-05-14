#! /usr/bin/python

import numpy as np
import cv2
import cvui

from merged_wdr_manager import MergedWDRImageManager

WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
           ]


def depth_image_colorized(depth_img):
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

#
# main rootine
#


zense_mng = MergedWDRImageManager(options)
cvui.init(WINDOW_NAME)

key = cv2.waitKey(10)
while ((key & 0xFF != ord('q')) or (key & 0xFF != 27)):
    fused_depth_image = zense_mng.filtered_fused_depth
    fused_depth_image_colorized = depth_image_colorized(fused_depth_image)
    cv2.imshow(WINDOW_NAME, fused_depth_image_colorized)
    key = cv2.waitKey(20)
    if key == 27:
        break

cv2.destroyAllWindows()
