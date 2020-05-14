#!/usr/bin/env python

from collections import deque
import numpy as np

NAMESPACE = 'merge_near_far'
MERGE_RATE = 3
NEAR = 'depth_near'
FAR = 'depth_far'
DEPTH_SHAPE = (480, 640)
DEPTH_SIZE = DEPTH_SHAPE[0] * DEPTH_SHAPE[1]

class MergeFarNearImage(object):
    def __init__(self, merge_queue_size=5):
        assert merge_queue_size > 0
        self.merge_queue_size = merge_queue_size
        self.merged_depth_img_near_far = None
        self.near_deque = deque(maxlen=self.merge_queue_size)
        self.far_deque = deque(maxlen=self.merge_queue_size)

    def clear_queue(self):
        self.near_deque.clear()
        self.far_deque.clear()

    def get_median(self, _deque):
        depth_stk = np.array(list(_deque))
        sort_idx = np.argsort(depth_stk, axis=0)
        n_filled = (depth_stk > 0).sum(axis=0)
        median_idx_in_sorted = -(n_filled / 2) + (self.merge_queue_size - 1)
        #median_idx = sort_idx.reshape(self.merge_queue_size, -1)[median_idx_in_sorted.reshape(-1),np.arange(DEPTH_SIZE)] # On python3.6, not works
        median_idx = sort_idx.reshape(self.merge_queue_size, -1)[median_idx_in_sorted.reshape(-1).astype(np.uint64), np.arange(DEPTH_SIZE)]
        depth_median = (depth_stk.reshape(self.merge_queue_size, -1)[
            median_idx, np.arange(DEPTH_SIZE)
        ]).reshape(DEPTH_SHAPE)
        return depth_median

    def get_merged(self, near, far):
        bflg_near_empty = (near == 0)
        depth_merged_median = near.copy()
        depth_merged_median[bflg_near_empty] = far[bflg_near_empty]
        return depth_merged_median

    def update(self, depth_image_near, depth_image_far):
        self.near_deque.append(depth_image_near)
        self.far_deque.append(depth_image_far)
        assert len(self.near_deque) == len(self.far_deque)
        n_contain_elem = len(self.near_deque)
        if n_contain_elem == self.merge_queue_size:
            near_depth_img_median = self.get_median(self.near_deque)
            far_depth_img_median = self.get_median(self.far_deque)
            self.merged_depth_img_near_far = self.get_merged(
                near_depth_img_median, far_depth_img_median)
            self.clear_queue()
            return True
        else:
            return False

    @property
    def merged_depth_image(self):
        assert self.merged_depth_img_near_far is not None
        return self.merged_depth_img_near_far
