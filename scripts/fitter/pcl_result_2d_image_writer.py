#!/usr/bin/env python
# -*- coding: utf-8 -*-
import io
from math import pi
import os
import sys

import numpy as np
import matplotlib.pyplot as plt

import cv2

from .pcl_fitter import DBSCAN_EPS, DBSCAN_MINPOINTS
DEG_TO_RAD = pi / 180.


CVUI_WINDOW_NAME = 'aspara_detector control'


class PCLResult2DImageWriter:
    def __init__(self):
        self.plots = {}

    def draw_depth_img(self, depth_img, title='default', cmap='jet', max_val=1.2):
        if not title in self.plots:
            fig, ax = plt.subplots(
                nrows=1, ncols=1, figsize=(5.5+1., 8.), dpi=100)
            ax = [ax]
            fig.canvas.set_window_title(title)
            plt.subplots_adjust(left=0., right=1.,
                                bottom=0.1, top=0.9, wspace=-0.1)
            self.plots[title] = [fig, ax]
        else:
            plot = self.plots[title]
            fig = plot[0]
            ax = plot[1]

        i_cam = 0
        ax[i_cam].clear()
        ax[i_cam].set_xticks([])
        ax[i_cam].set_yticks([])
        depth_tmp = np.array(depth_img.transpose()[:, ::-1])
        sm = ax[i_cam].imshow(depth_tmp, cmap=cmap, vmin=0.,
                              vmax=max_val, aspect='equal')
        if len(self.plots[title]) == 2:
            colorbar = fig.colorbar(sm)
            self.plots[title].append(colorbar)

        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=180)
        buf.seek(0)
        img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        img = cv2.imdecode(img_arr, 1)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def draw_img_with_fit_result(self, depth_img, line_list):
        LINE_WIDTH = 3
        MARKER_RADIUS = 10
        MAX_DEPTH = 1.
        MAX_COLOR = MAX_DEPTH * 2.

        image = np.clip(depth_img, 0., MAX_DEPTH)

        tip_depth_list = [line.xyd_ends[1, 2] for line in line_list]
        tip_depth_sortidx = np.argsort(np.array(tip_depth_list))[::-1]
        for i_line in range(len(line_list)):
            line = line_list[tip_depth_sortidx[i_line]]
            if line.is_ignored:
                continue
            if line.is_good:
                line_color = 2.
            else:
                line_color = 1.5

            xyd_ends = line.xyd_ends.astype(np.int32)
            cv2.line(image, (xyd_ends[0, 0], xyd_ends[0, 1]),
                     (xyd_ends[1, 0], xyd_ends[1, 1]), line_color, LINE_WIDTH)
            if not hasattr(line, 'sector_mask') or len(line.sector_mask) == 0:
                continue
            image[line.sector_mask] = line_color
            image[line.occlusion_mask] = 1.
            if not line.is_good:
                continue
            cv2.circle(
                image, (xyd_ends[0, 0], xyd_ends[0, 1]), MARKER_RADIUS, line_color, -1)

        return self.draw_depth_img(image, "result", cmap='brg', max_val=MAX_COLOR)
