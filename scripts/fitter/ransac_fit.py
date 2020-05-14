#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np

from time import time

# TODO: Use np.dot!!!

def print_type(item_list):
    for item in item_list:
        print("{}".format(item.dtype))


def ransac_line_fit(pcl, n_iter=500, dthres_inlier=0.010, cut_percentile=0.8, max_root_z=0.20):

    n_point = pcl.shape[0]

    if max_root_z > 0.:
        pcl_z_min = pcl[:,2].min()
        pcl_z_max = pcl[:,2].max()

        a_z_max = (pcl_z_max - pcl_z_min) * max_root_z + pcl_z_min
        b_z_min = (pcl_z_max - pcl_z_min) * max(0.40, max_root_z) + pcl_z_min

        idx_array = np.arange(n_point)
        flg_a_pool = (pcl[:,2] < a_z_max)
        idx_a_pool = idx_array[flg_a_pool]
        flg_b_pool = (pcl[:,2] > b_z_min)
        idx_b_pool = idx_array[flg_b_pool]

        idx_a = idx_a_pool[np.random.randint(idx_a_pool.shape[0], size=n_iter)]
        idx_b = idx_b_pool[np.random.randint(idx_b_pool.shape[0], size=n_iter)]
    else:
        idx_a = np.random.randint(n_point, size=n_iter)
        idx_b = np.random.randint(n_point-1, size=n_iter) + idx_a + 1
        idx_b[idx_b > n_point-1] -= n_point

    point_a = pcl[idx_a]
    point_b = pcl[idx_b]

    is_flipped = (point_b[:, 2] < point_a[:, 2])

    origin = point_a.copy()
    direction = point_b - origin

    origin[is_flipped] = point_b[is_flipped]
    direction[is_flipped] *= -1

    direction /= np.linalg.norm(direction, axis=-1).reshape(-1, 1)
    theta = np.arccos(direction[:,2], dtype=np.float32)
    phi = np.arctan2(direction[:,1], direction[:,0], dtype=np.float32)

    cos_phi = np.cos(phi, dtype=np.float32)
    sin_phi = np.sin(phi, dtype=np.float32)
    phi_mtx = np.zeros([n_iter,3,3], dtype=np.float32)
    phi_mtx[:, 0, 0] = cos_phi
    phi_mtx[:, 0, 1] = +sin_phi
    phi_mtx[:, 1, 0] = -sin_phi
    phi_mtx[:, 1, 1] = cos_phi
    phi_mtx[:, 2, 2] = 1.

    cos_theta = np.cos(theta, dtype=np.float32)
    sin_theta = np.sin(theta, dtype=np.float32)
    theta_mtx = np.zeros([n_iter,3,3], dtype=np.float32)
    theta_mtx[:, 2, 2] = cos_theta
    theta_mtx[:, 2, 0] = +sin_theta
    theta_mtx[:, 0, 2] = -sin_theta
    theta_mtx[:, 0, 0] = cos_theta
    theta_mtx[:, 1, 1] = 1.

    rot_mtx = (theta_mtx.reshape(-1,3,1,3) * phi_mtx.transpose([0,2,1]).reshape(-1,1,3,3)).sum(axis=-1)

    pcl_tfm = np.matmul(rot_mtx.reshape(-1, 1, 3, 3),  (pcl.reshape(1, -1, 3, 1) - origin.reshape(-1, 1, 3, 1))).reshape(n_iter, n_point, 3)

    pcl_tfm_xy = pcl_tfm[:,:,0:2]
    pcl_tfm_z = pcl_tfm[:,:,2]

    dsq2line = (pcl_tfm_xy ** 2).sum(axis=-1)
    is_inlier = (dsq2line < dthres_inlier ** 2)
    n_inlier = is_inlier.sum(axis=-1) - 2
    frac_inlier = n_inlier.astype(np.float32) / float(n_point-2)

    is_outlier = (is_inlier == False)
    pcl_tfm_z_inlier = pcl_tfm_z * is_inlier
    line_z_min = (pcl_tfm_z_inlier + is_outlier * +1e5).min(axis=-1)
    line_z_max = (pcl_tfm_z_inlier + is_outlier * -1e5).max(axis=-1)
    line_len = line_z_max - line_z_min
    mean_density = n_inlier / line_len

    n_inlier_for_denom = n_inlier.astype(np.float32)
    n_inlier_for_denom[n_inlier == 0] = 1.
    dsq_mean = (dsq2line * is_inlier).sum(axis=-1) / n_inlier_for_denom
    cand_err = dsq_mean

#    cut_variable = frac_inlier
    cut_variable = frac_inlier * mean_density

    n_cand = cand_err.shape[0]
    cut_thres = np.sort(cut_variable)[int(n_cand * cut_percentile)]
    flg_pass = (cut_variable >= cut_thres)
    idx_sorted = np.argsort(cand_err)
    i_best = idx_sorted[np.argmax(flg_pass[idx_sorted])]

    tfm_mtx = np.zeros([4,4], dtype=np.float32)
    tfm_mtx[:3,:3] = np.linalg.inv(rot_mtx[i_best])
    tfm_mtx[:3,3] = origin[i_best] + direction[i_best] * line_z_min[i_best]
    tfm_mtx[3,3] = 1.
#    print tfm_mtx

#    print_type([pcl, rot_mtx,pcl_tfm,dsq2line,is_inlier,n_inlier,cut_variable])

    if flg_pass.sum() > 0:
        return line_len[i_best], tfm_mtx, is_outlier[i_best]
    else:
        return None, None, None



def ransac_ground_fit(pcl, xy_binidx, grid_xyzw, n_iter=100, dthres_inlier=0.010, cut_percentile=0.8):

    min_n_inbin = 20
    max_ground_tilt = 60. / 180. * math.pi
    min_normal_z = math.cos(max_ground_tilt)

    x_nbin = grid_xyzw.shape[0]
    y_nbin = grid_xyzw.shape[1]

    n_success = 0
    for i_x in range(x_nbin):
        for i_y in range(y_nbin):

            i_xy = np.array([i_x, i_y])
            bflg_inbin = (xy_binidx == i_xy).all(axis=-1)
            n_point = bflg_inbin.sum()
            if n_point < min_n_inbin:
#                print ' - Failed! n={} , xy={} , Too few points'.format(n_point, grid_xyzw[i_x,i_y,0:2])
                continue
            pcl_inbin = pcl[bflg_inbin]

            rand_0 = np.random.randint(n_point, size=n_iter)
            rand_1 = np.random.randint(n_point-1, size=n_iter)
            rand_2 = np.random.randint(n_point-2, size=n_iter)

            idx_0 = rand_0
            idx_1 = idx_0 + 1 + rand_1
            idx_2 = idx_0 + 1 + rand_2
            idx_2[idx_2 == idx_1] += 1
            idx_1[idx_1 > n_point-1] -= n_point
            idx_2[idx_2 > n_point-1] -= n_point

            point_0 = pcl_inbin[idx_0]
            point_1 = pcl_inbin[idx_1]
            point_2 = pcl_inbin[idx_2]

            origin = point_0
            v_1 = point_1 - origin
            v_2 = point_2 - origin

            normal = np.cross(v_2, v_1)
            norm_normal = np.linalg.norm(normal, axis=-1)
            bflg_zeronorm = (norm_normal == 0.)
            norm_normal[bflg_zeronorm] = 1.
            normal /= norm_normal.reshape(-1, 1)
            normal[bflg_zeronorm] = [0., 0., 1.]

            dist2plane = np.absolute(((pcl_inbin.reshape(1,-1,3) - origin.reshape(-1,1,3)) * normal.reshape(-1,1,3)).sum(-1))
            is_inlier = (dist2plane < dthres_inlier)
            n_inlier = is_inlier.sum(-1) - 3
            frac_inlier = n_inlier.astype(np.float32) / float(n_point-3)

            n_inlier_for_denom = n_inlier.astype(np.float32)
            n_inlier_for_denom[n_inlier == 0] = 1.
            dist_mean = (dist2plane * is_inlier).sum(axis=-1) / n_inlier_for_denom
            cand_err = dist_mean

            cut_variable = frac_inlier

            n_cand = cand_err.shape[0]
            cut_thres = np.sort(cut_variable)[int(n_cand * cut_percentile)]
            flg_pass = (cut_variable >= cut_thres)
            idx_sorted = np.argsort(cand_err)
            i_best = idx_sorted[np.argmax(flg_pass[idx_sorted])]

            if not flg_pass.sum() > 0:
#                print ' - Failed! n={} , xy={} , No valid candidate'.format(n_point, grid_xyzw[i_x,i_y,0:2])
                continue

            best_origin = origin[i_best]
            best_normal = normal[i_best]
            if best_normal[2] < 0.:
                best_normal *= -1.

            if best_normal[2] < min_normal_z:
#                print ' - Failed! n={} , xy={} , Exceeds maximum tilt : cosz={}'.format(n_point, grid_xyzw[i_x,i_y,0:2], best_normal[2])
                continue

            grid_ctr_xyz = grid_xyzw[i_x, i_y, :3]

            d_ctr = grid_ctr_xyz - best_origin
            d_ctr[2] = -(best_normal[0] * d_ctr[0] + best_normal[1] * d_ctr[1]) / best_normal[2]
            grid_xyzw[i_x, i_y, :3] = best_origin + d_ctr
            grid_xyzw[i_x, i_y, 3] = 5.

            n_success += 1

    print('Local ground fit success : {} / {}'.format(n_success, x_nbin * y_nbin))

    return grid_xyzw
