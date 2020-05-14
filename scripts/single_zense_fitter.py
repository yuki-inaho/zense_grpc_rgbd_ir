#! /usr/bin/python

import numpy as np
import cv2
import toml
import click
from fitter.pcl_result_2d_image_writer import PCLResult2DImageWriter
from merged_wdr_manager import MergedWDRImageManager
#from fitter.pcl_visualizer import PCLVisualizer
from fitter.pcl_fitter import PCLFitter
from fitter.cameraparam import CameraParam


WINDOW_NAME = "gRPC Test"
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

options = [('grpc.max_send_message_length', 10 * 1024 * 1024),
           ('grpc.max_receive_message_length', 10 * 1024 * 1024)
           ]


NAMESPACE = 'single_zense_fitter'
UPDATE_FREQ = 10.  # Hz
SCALE = 1e-3


class SingleZenseFitter(object):
    def __init__(self, toml_path, eps_default=0.016, min_point_default=10):
        self.toml_path = toml_path
        self._load_toml(toml_path)
        self.pub_setting_string()
        self.zense_mng = MergedWDRImageManager(options)

        self.eps_default = eps_default
        self.min_point_default = min_point_default
        self.result_img_writer = PCLResult2DImageWriter()

    def _load_toml(self, toml_file_path):
        sensor_toml = toml.load(open(toml_file_path))
        self.sensor_toml_path = sensor_toml
        selection_keys = ['pcl_cutoff_dist',
                          'target_max_dist',
                          'target_min_dist',
                          'target_max_len',
                          'target_min_len',
                          'target_max_tilt']

        if not all([k in sensor_toml['Selection'] for k in selection_keys]):
            print('Missing parameter in "[Selection]" section of TOML')
            print('Expected: {}, Specified: {}'.format(
                selection_keys, sensor_toml['Selection'].keys())
            )

        self.camera_param = self.setup_camera_param(sensor_toml)
        self.pcl_fitter = self.setup_pcl_fitter(sensor_toml)

    def setup_camera_param(self, dict_toml):
        camera_param = {}
        key = 'Camera0'
        height = int(dict_toml[key]['height'])
        width = int(dict_toml[key]['width'])
        fx = float(dict_toml[key]['fx'])
        fy = float(dict_toml[key]['fy'])
        cx = float(dict_toml[key]['cx'])
        cy = float(dict_toml[key]['cy'])
        roll = float(dict_toml[key]['rot_angle_roll'])
        pitch = float(dict_toml[key]['rot_angle_pitch'])
        yaw = float(dict_toml[key]['rot_angle_yaw'])
        tx = float(dict_toml[key]['translation_x'])
        ty = float(dict_toml[key]['translation_y'])
        tz = float(dict_toml[key]['translation_z'])

        K = (fx, 0., cx, 0., fy, cy, 0., 0., 1.)
        R = (1., 0., 0., 0., 1., 0., 0., 0., 1.)
        P = (fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.)
        size = (height, width)

        camera_param = CameraParam()
        camera_param.set_camera_param(K, R, P, size)
        camera_param.set_tf_rot_and_trans(
            [roll, pitch, yaw], [tx, ty, tz])

        return camera_param

    def setup_pcl_fitter(self, dict_toml):
        set_roll = float(dict_toml['General']['set_roll'])
        set_pitch = float(dict_toml['General']['set_pitch'])
        set_yaw = float(dict_toml['General']['set_yaw'])
        camera_set_param = CameraParam()
        camera_set_param.set_tf_rot_and_trans(
            [set_roll, set_pitch, set_yaw], [0., 0., 0.])
        pcl_fitter = PCLFitter(camera_set_param)

        pcl_fitter.pcl_cutoff_dist = float(
            dict_toml['Selection']['pcl_cutoff_dist'])
        pcl_fitter.target_max_dist = float(
            dict_toml['Selection']['target_max_dist'])
        pcl_fitter.target_min_dist = float(
            dict_toml['Selection']['target_min_dist'])
        pcl_fitter.target_max_len = float(
            dict_toml['Selection']['target_max_len'])
        pcl_fitter.target_min_len = float(
            dict_toml['Selection']['target_min_len'])
        pcl_fitter.target_max_tilt = float(
            dict_toml['Selection']['target_max_tilt'])

        return pcl_fitter

    def pub_setting_string(self):
        cutoff_dist = self.pcl_fitter.pcl_cutoff_dist * 100.0  # [cm]
        min_len = self.pcl_fitter.target_min_len * 100.0  # [cm]
        max_len = self.pcl_fitter.target_max_len * 100.0  # [cm]
        min_dist = self.pcl_fitter.target_min_dist * 100.0  # [cm]
        max_dist = self.pcl_fitter.target_max_dist * 100.0  # [cm]
        max_tilt = self.pcl_fitter.target_max_tilt  # [deg]
        setting_str = "cutoff: {:.0f} | len: {:.0f} - {:.0f} | dist: {:.0f} - {:.0f} | tilt: {:.0f}".format(
            cutoff_dist, min_len, max_len, min_dist, max_dist, max_tilt
        )
        print(setting_str)

    def terminate(self):
        pass

    def run(self):
        self.fused_depth_image = self.zense_mng.filtered_fused_depth * SCALE
        pcd = self.pcl_fitter.get_pcd_from_depth_img(
            self.fused_depth_image, self.camera_param)
        eps = self.eps_default
        min_point = self.min_point_default

        line_list, pcd_list, fitgeom_list = [], [], []
        line_list, pcd_list, fitgeom_list, all_points_ary, ground_points_ary = self.pcl_fitter.fit_pcd(
            pcd, eps, min_point)
        res_img = self.result_img_writer.draw_img_with_fit_result(
            self.fused_depth_image, line_list)
        return res_img


@click.command()
@click.option("--toml-path", "-t", default="../cfg/camera.toml")
def main(toml_path):
    fitter = SingleZenseFitter(toml_path)
    image_scale = 0.5
    key = cv2.waitKey(10)
    while key & 0xFF != 27:
        res_img = fitter.run()
        res_img = cv2.resize(res_img, (int(image_scale * res_img.shape[1]),
                                       int(image_scale * res_img.shape[0])))
        cv2.imshow("result", res_img)
        key = cv2.waitKey(10)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
