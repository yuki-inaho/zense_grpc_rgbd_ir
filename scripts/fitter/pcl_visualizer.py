#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import pi
import os
import sys
from time import sleep

import numpy as np
import matplotlib.pyplot as plt

import cv2
import cvui
import open3d as o3d
import rospy

from std_msgs.msg import Int16

from pcl_fitter import DBSCAN_EPS, DBSCAN_MINPOINTS
DEG_TO_RAD = pi / 180.

# stop buffering
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

CVUI_WINDOW_NAME = 'aspara_detector control'

class PCLVisualizer:
    def __init__(self, auto_mode):
        self.pub_send_next_img = rospy.Publisher('/send_next_img', Int16, queue_size=1)

        self.terminate = 0
        self.pcd_list = []
        self.fitgeom_list = []
        self.vis = None
        self.plots = {}
        self.cvui_frame = None
        self.cvui_dbscan_eps = [DBSCAN_EPS]
        self.cvui_dbscan_minpoints = [DBSCAN_MINPOINTS]

        self.apply_fit = True
        self.show_fit_geometry = True
        self.show_goodbad_color = True

        if auto_mode:
            self.auto_mode = True
            self.show_2d = True
            self.show_3d = False
        else:
            self.auto_mode = False
            self.show_2d = True
            self.show_3d = True
            self.create_visualizer()
            self.cvui_frame = np.zeros((200, 500, 3), np.uint8)
            cvui.init(CVUI_WINDOW_NAME)


    def create_visualizer(self, window_name='orig'):
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(window_name=window_name, width=1280, height=720)
        self.vis.register_key_callback(ord("N"), self.call_next)
        self.vis.register_key_callback(ord("B"), self.call_prev)
        self.vis.register_key_callback(ord("R"), self.call_refit)
        self.vis.register_key_callback(ord("Q"), self.call_exit)
        self.vis.register_key_callback(ord("V"), self.call_default_view)
        self.vis.register_key_callback(ord("F"), self.call_toggle_fitgeom)
        self.vis.register_key_callback(ord("2"), self.call_toggle_2d)
        self.vis.register_key_callback(ord("3"), self.call_toggle_3d)
        self.vis.register_key_callback(ord("C"), self.call_toggle_targetcolor)
        self.vis.register_key_callback(ord("D"), self.call_toggle_apply_fit)
        self.vis.register_key_callback(ord("A"), self.call_toggle_auto_mode)

        render_option = self.vis.get_render_option()  # open3d.visualization.RenderOption
        render_option.point_show_normal = False
        render_option.point_size = 2.

        self.call_default_view()


    def update_visualizer(self):
        if not self.show_3d:
            return
        if self.vis is None:
            return
            
        [self.vis.update_geometry(_pcd) for _pcd in self.pcd_list] 
        [self.vis.update_geometry(_fitgeom) for _fitgeom in self.fitgeom_list] 
        self.vis.poll_events()
        self.vis.update_renderer()


    def save_visualizer(self):
        if not self.show_3d:
            return
        if self.vis is None:
            return
        if hasattr(self, 'output_path'):
            save_file = '{}/{}_3d.png'.format(self.output_path[0], self.output_path[1])
            print 'Saving 3D image : {}'.format(save_file)
            self.vis.capture_screen_image(save_file, do_render=False)


    def save_view(self, json_path=None):
        if self.vis is None:
            return
        param = self.vis.get_view_control().convert_to_pinhole_camera_parameters()
        if json_path is None:
            self.view_param = param
        else:
            print('Saving view to : {}'.format(json_path))
            o3d.io.write_pinhole_camera_parameters(json_path, param)

    def load_view(self, json_path=None):
        if self.vis is None:
            return
#        print 'Loading view from : {}'.format(json_path)
        if json_path is None:
            param = self.view_param
        else:
            param = o3d.io.read_pinhole_camera_parameters(json_path)
        self.vis.get_view_control().convert_from_pinhole_camera_parameters(param)


    def call_default_view(self, dummy_vis=None):
        print 'Setting to default view.'
        selfdir = os.path.dirname(os.path.abspath(__file__))
        json_path = '{}/default_camera_params.json'.format(selfdir)
        self.load_view(json_path)

    def call_exit(self, dummy_vis=None):
        print 'Exit'
        self.terminate = 2

    def call_next(self, dummy_vis=None):
        print 'Next'
        msg = Int16()
        msg.data = +1
        self.pub_send_next_img.publish(msg)
        self.terminate = 1

    def call_prev(self, dummy_vis=None):
        print 'Prev'
        msg = Int16()
        msg.data = -1
        self.pub_send_next_img.publish(msg)
        self.terminate = 1

    def call_refit(self, dummy_vis=None):
        print 'Refit'
        self.terminate = 1

    def call_toggle_auto_mode(self, dummy_vis=None):
        self.auto_mode = not self.auto_mode
        print 'Turning {} auto-mode'.format('on' if self.auto_mode else 'off')

    def call_toggle_apply_fit(self, dummy_vis=None):
        self.apply_fit = not self.apply_fit
        print 'Turning fit {}'.format('on' if self.apply_fit else 'off')
        self.terminate = 1

    def call_toggle_fitgeom(self, dummy_vis=None):
        self.show_fit_geometry = not self.show_fit_geometry
        print 'Turning {} fit geometry'.format('on' if self.show_fit_geometry else 'off')
        if self.show_fit_geometry:
            self.add_geometry(self.fitgeom_list)
        else:
            self.remove_geometry(self.fitgeom_list)

    def call_toggle_2d(self, dummy_vis=None):
        self.show_2d = not self.show_2d
        print 'Turn {} 2D'.format('on' if self.show_2d else 'off')

    def call_toggle_3d(self, dummy_vis=None):
        self.show_3d = not self.show_3d
        print 'Turn {} 3D'.format('on' if self.show_3d else 'off')

    def call_toggle_targetcolor(self, dummy_vis=None):
        self.show_goodbad_color = not self.show_goodbad_color
        print 'Line color mode : {}'.format('good-bad' if self.show_goodbad_color else 'cluster')
        self.terminate = 1

    def add_geometry(self, geom_list):
        if self.vis is None:
            return
        self.save_view()
        for geom in geom_list:
            self.vis.add_geometry(geom)
        self.load_view()

    def remove_geometry(self, geom_list):
        if self.vis is None:
            return
        self.save_view()
        for geom in geom_list:
#            print(geom)
            success = self.vis.remove_geometry(geom)
#            print(success)
        self.load_view()

    def reset_geometry(self):
        if hasattr(self, 'pcd_list'):
            self.remove_geometry(self.pcd_list + self.fitgeom_list)
        self.pcd_list = []
        self.fitgeom_list = []


    def update_cvui(self):
        if self.cvui_frame is None:
            return
        self.cvui_frame[:] = (49, 52, 49)
        if True:
            cvui.text(self.cvui_frame, 20, 30, 'DBSCAN')
            cvui.text(self.cvui_frame, 40, 60, 'eps')
            cvui.counter(self.cvui_frame, 110, 55, self.cvui_dbscan_eps, 0.001, '%.3f')
            cvui.text(self.cvui_frame, 40, 90, 'minpoints')
            cvui.counter(self.cvui_frame, 110, 85, self.cvui_dbscan_minpoints, 1, '%d')
        cvui.imshow(CVUI_WINDOW_NAME, self.cvui_frame)
        cv_key = cv2.waitKey(1)
        if cv_key == 27:
            self.call_exit()
        elif cv_key == 110:
            self.call_next()
        elif cv_key == 98:
            self.call_prev()
        elif cv_key == 114:
            self.call_refit()
        elif cv_key == 113:
            self.call_exit()
        elif cv_key == 118:
            self.call_default_view()
        elif cv_key == 102:
            self.call_toggle_fitgeom()
        elif cv_key == 50:
            self.call_toggle_2d()
        elif cv_key == 51:
            self.call_toggle_3d()
        elif cv_key == 99:
            self.call_toggle_targetcolor()
        elif cv_key == 100:
            self.call_toggle_apply_fit()
        elif cv_key == 97:
            self.call_toggle_auto_mode()
        elif cv_key != -1:
            print(cv_key)


    def draw_depth_img(self, depth_img, title='default', cmap='jet', max_val=1.2):
        camera_name_list = depth_img.keys()
        n_camera = len(camera_name_list)
        if not title in self.plots:
            fig, ax = plt.subplots(nrows=1, ncols=n_camera, figsize=(5.5*n_camera+1., 8.), dpi=100)
            if n_camera == 1:
                ax = [ax]
            else:
                ax = ax.reshape(-1)
            fig.canvas.set_window_title(title)
            plt.subplots_adjust(left=0., right=1., bottom=0.1, top=0.9, wspace=-0.1)
            self.plots[title] = [fig, ax]
        else:
            plot = self.plots[title]
            fig = plot[0]
            ax = plot[1]
        for i_cam in range(n_camera):
            ax[i_cam].clear()
            ax[i_cam].set_xticks([])
            ax[i_cam].set_yticks([])
            depth_tmp = np.array(depth_img[camera_name_list[i_cam]].transpose()[:,::-1])
            sm = ax[i_cam].imshow(depth_tmp, cmap=cmap, vmin=0., vmax=max_val, aspect='equal')
        if len(self.plots[title]) == 2:
            colorbar = fig.colorbar(sm)
            self.plots[title].append(colorbar)
        if hasattr(self, 'output_path'):
            save_file = '{}/{}_{}.jpg'.format(self.output_path[0], self.output_path[1], title)
            print 'Saving 2D image : {}'.format(save_file)
            fig.savefig(save_file)
        plt.pause(0.001)


    def draw_img_with_fit_result(self, depth_img, line_list):
        LINE_WIDTH = 3
        MARKER_RADIUS = 10
        MAX_DEPTH = 1.
        MAX_COLOR = MAX_DEPTH * 2.

        image_dict = {}
        camera_name_list = depth_img.keys()
        for camera_name in camera_name_list:
            image = np.clip(depth_img[camera_name], 0., MAX_DEPTH)

            tip_depth_list = [line.xyd_ends[camera_name][1, 2] for line in line_list]
            tip_depth_sortidx = np.argsort(np.array(tip_depth_list))[::-1]
            for i_line in range(len(line_list)):
                line = line_list[tip_depth_sortidx[i_line]]
                if line.is_ignored:
                    continue
#                if not (line.is_solitary and not line.is_multiline_cluster and line.is_grounded):
#                    continue
                if line.is_good:
                    line_color = 2.
#                elif line.is_contained:
#                    line_color = 1.75
                else:
                    line_color = 1.5

                xyd_ends = line.xyd_ends[camera_name].astype(np.int32)
                cv2.line(image, (xyd_ends[0,0], xyd_ends[0,1]), (xyd_ends[1,0], xyd_ends[1,1]), line_color, LINE_WIDTH)
                if not hasattr(line, 'sector_mask'):
                    continue
                if not camera_name in line.sector_mask:
                    continue
                image[line.sector_mask[camera_name]] = line_color
                image[line.occlusion_mask[camera_name]] = 1.
                if not line.is_good:
                    continue
                cv2.circle(image, (xyd_ends[0,0], xyd_ends[0,1]), MARKER_RADIUS, line_color, -1)
            image_dict[camera_name] = image

        self.draw_depth_img(image_dict, "result", cmap='brg', max_val=MAX_COLOR)


    def set_line_goodbad_color(self, line_list):
        for line in line_list:
            if line.is_ignored:
                continue
            line_color = [0., 1., 0.] if line.is_good else [1., 0., 0.]
            for mesh in line.meshes:
                mesh.paint_uniform_color(line_color)


    def update(self, depth_img, line_list, pcd_list, fitgeom_list):

        if self.show_2d:
            # self.draw_depth_img(depth_img, "depth") # show depth_img
            if self.apply_fit:
                self.draw_img_with_fit_result(depth_img, line_list)

        if self.show_3d:
            self.reset_geometry()
            if self.show_goodbad_color:
                self.set_line_goodbad_color(line_list)
            self.pcd_list = pcd_list
            self.fitgeom_list = fitgeom_list
            if self.show_fit_geometry:
                self.add_geometry(self.pcd_list + self.fitgeom_list)
            else:
                self.add_geometry(self.pcd_list)

        while not rospy.is_shutdown():
            if self.auto_mode:
                self.call_next()
            self.update_cvui()
            self.update_visualizer()
            sleep(0.05)
            if self.terminate != 0:
                break
        self.save_visualizer()

        do_exit = (self.terminate == 2)
        self.terminate = 0
        return do_exit

    def update_wo_ros(self, pcl, line_list=[], fitgeom_list=[]):
        self.reset_geometry()
        self.pcd_list = [pcl]
        print(len(self.pcd_list))
        self.add_geometry(self.pcd_list)
        # if line_list:
        #     self.set_line_goodbad_color(line_list)
        self.fitgeom_list = fitgeom_list if fitgeom_list else []
        self.add_geometry(self.pcd_list + self.fitgeom_list)
        self.update_cvui()
        self.update_visualizer()
        do_exit = (self.terminate == 2)
        self.terminate = 0
        return do_exit

    def view(self):
        while True:
            self.update_cvui()
            self.update_visualizer()
            sleep(0.05)
            if self.terminate != 0:
                break

    @property
    def get_param(self):
        return self.cvui_dbscan_eps[0], self.cvui_dbscan_minpoints[0]
