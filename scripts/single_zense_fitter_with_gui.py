#! /usr/bin/python

import os
import shutil

import cv2
import numpy as np
import PySimpleGUI as sg
import click
import toml
from datetime import datetime
import json
from itertools import filterfalse #if python2 import ifilterfalse
import pdb

from fitter.cameraparam import CameraParam
from fitter.pcl_fitter import PCLFitter
from merged_wdr_manager import MergedWDRImageManager
from fitter.pcl_result_2d_image_writer import PCLResult2DImageWriter
from collections import deque

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# from fitter.pcl_visualizer import PCLVisualizer

sg.theme('Black')

WINDOW_NAME = "gRPC Test"

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
        return res_img, line_list


class DetectionGUIManager():
    def __init__(self, toml_path, output_data_path, default_iter_num):
        self.iter_num = default_iter_num

        # define the window layout
        layout = [

            [
                sg.Image(filename='', key='-RESULT-',
                         tooltip='Right click for exit menu')
            ],
            [
                sg.Text('Number of Iteration of Detection : ',
                        font='Default 14'),
                sg.Input(key='-ITER-', size=(10, 1)),
                sg.Text('Current Number : ',
                        text_color='lightgreen', font='Default 12'),
                sg.Text('', text_color='lightgreen',
                        font='Default 12', key="iter_num", size=(5, 1)),
            ],
            [
                sg.Button('Detection', size=(10, 1)),
                sg.Button('Clear Result', size=(10, 1)),
                sg.Button('Change Iter Num', size=(15, 1)),
                sg.Button('Output', size=(10, 1)),
            ]
        ]

        # Ready for save result
        self.output_data_path = output_data_path
        self.mkdir4save()

        self.fitter = SingleZenseFitter(toml_path)
        self.image_scale = 0.5
        self.update_detection_result()

        # create the window and show it without the plot
        self.window = sg.Window('SingleZenseDetector', layout, location=(800, 200),
                                no_titlebar=True, grab_anywhere=True,
                                right_click_menu=['&Right', ['E&xit']], )  # if trying Qt, you will need to remove this right click menu

        # Initial drawing
        _, _ = self.window.read(timeout=20)
        self.draw_result()
        self.window['iter_num'].update(self.iter_num)
        self.window.FindElement('Output').Update(disabled=True)

    def make_detected_lines_queue(self):
        return deque(maxlen=self.iter_num)

    def mkdir4save(self):
        if not os.path.exists(self.output_data_path):
            os.mkdir(self.output_data_path)

    def clear_saved_data(self):
        assert os.path.exists(self.output_data_path)
        shutil.rmtree(self.output_data_path)
        os.mkdir(self.output_data_path)

    def draw_result(self):
        imgbytes = cv2.imencode('.png', self.res_img)[1].tobytes()
        self.window['-RESULT-'].update(data=imgbytes)

    def update_detection_result(self, result_deque=None):
        res_img, detected_line_list = self.fitter.run()
        res_img = cv2.resize(res_img, (int(self.image_scale * res_img.shape[1]),
                                       int(self.image_scale * res_img.shape[0])))
        self.res_img = res_img
        if result_deque is not None:
            result_deque.append(detected_line_list)

    def get_timestamp(self):
        return datetime.now().strftime('%Y:%m:%d_%H:%M:%S')

    def extract_valid_lines(self, line_list):
        n_timeframe = len(line_list)
        for t in range(n_timeframe):
            # Extract lines enabled is_final flag
            validity_list = [line.is_final for line in line_list[t]]
            valid_elem = [i for i, val in enumerate(validity_list) if val] 
            line_list[t] = [line_list[t][_elem] for _elem in valid_elem] 

    def convert_line_to_json(self, result_line_list_deque):
        json_line_obj = {}
        for k, line_list_t in enumerate(result_line_list_deque):
            _line_obj_t = {}
            for i, line in enumerate(line_list_t):
                _line_obj_t[i] = line.position_ends[0].tolist() #root position
            json_line_obj[k] = _line_obj_t
        return json_line_obj

    def update(self):
        event, values = self.window.read(timeout=20)
        if event in ('Exit', sg.WIN_CLOSED):
            return False
        elif event == 'Detection':
            self.result_line_list_deque = self.make_detected_lines_queue()
            for i in range(self.iter_num):
                self.update_detection_result(self.result_line_list_deque)
                self.draw_result()
            self.window.FindElement('Output').Update(disabled=False)
            return True
        elif event == 'Clear Result':
            self.clear_saved_data()
            return True
        elif event == 'Change Iter Num':
            iter_input = values['-ITER-']
            if iter_input == None:
                print("No string is inputtted in text box.")
                return True
            if not str.isnumeric(iter_input):
                print("Inputted string is not numeric.")
                return True
            iter_num = int(iter_input)
            if iter_num <= 0:
                print("Inputted number have to be > 0")
                return True
            self.iter_num = iter_num
            self.window['iter_num'].update(self.iter_num)
            return True
        elif event == 'Output':
            timestamp = self.get_timestamp()
            self.extract_valid_lines(self.result_line_list_deque)
            json_line_obj = self.convert_line_to_json(self.result_line_list_deque)
            with open(os.path.join(self.output_data_path, "{}.json").format(timestamp), 'w') as f:
                json.dump(json_line_obj, f)
            return True
        else:
            return True

    def run(self):
        while self.update():
            pass
        self.window.close()


@ click.command()
@ click.option("--toml-path", "-t",
               default="{}/../cfg/camera.toml".format(SCRIPT_DIR))
@ click.option("--output-data-path", "-o",
               default="{}/../data".format(SCRIPT_DIR))
@ click.option("--default-iter-num", "-i",
               default=1)
def main(toml_path, output_data_path, default_iter_num):
    gui_mng = DetectionGUIManager(
        toml_path, output_data_path, default_iter_num)
    gui_mng.run()


if __name__ == "__main__":
    main()
