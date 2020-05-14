

import math
from math import pi
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import cv2
import toml

from .cameraparam import CameraParam
from .fitted_line import FittedLine
from .ransac_fit import ransac_line_fit, ransac_ground_fit
from .util import check_all_false

# TODO: output random seed used in ransac and open3d

# PCL pre-processing (the unit of these numerics is [m])
DOWNSAMPLE_VOXEL_SIZE = 0.003
DOWNSAMPLE_VOXEL_SIZE_GROUND = 0.005

# Ground fit
X_MIN = 0.
X_MAX = +1.2
Y_MIN = -0.8
Y_MAX = +0.8
GRID_SIZE = 0.080
GROUND_SEED_Z_MAX = 0.
GROUND_SEED_MARGIN = 0.080
GROUND_MARGIN = 0.030
SMOOTHING_KERNEL = GRID_SIZE * 0.5

# Clustering
# DBSCAN_EPS : Density parameter that is used to find neighbouring points
# DBSCAN_MINPOINTS : Minimum number of points to form a cluster
DBSCAN_EPS = 0.016
DBSCAN_MINPOINTS = 10

CLUSTER_MINPOINTS = 50

CMAP_CLUSTER = plt.get_cmap("tab20")

def set_pcl_fitter(toml_path):
    dict_toml = toml.load(open(toml_path))
    set_roll = float(dict_toml['General']['set_roll'])
    set_pitch = float(dict_toml['General']['set_pitch'])
    set_yaw = float(dict_toml['General']['set_yaw'])
    camera_set_param = CameraParam()
    camera_set_param.set_tf_rot_and_trans([set_roll, set_pitch, set_yaw], [0., 0., 0.])
    return PCLFitter(camera_set_param, dict_toml)


class PCLFitter(object):

    def __init__(self, camera_set_param=None, target_attribute=None):
        self.depth_img = None
        self.camera_param = None
        self.grid_xyzw = None
        if camera_set_param is None:
            self.camera_set_param = CameraParam()
        else:
            self.camera_set_param = camera_set_param
        if target_attribute is None:
            self.set_parameters()
        else:
            self.set_target_attribute(target_attribute)
    
    def set_target_attribute(self, dict_toml):
        self.pcl_cutoff_dist = float(dict_toml['Selection']['pcl_cutoff_dist'])
        self.target_max_dist = float(dict_toml['Selection']['target_max_dist'])
        self.target_min_dist = float(dict_toml['Selection']['target_min_dist'])
        self.target_max_len = float(dict_toml['Selection']['target_max_len'])
        self.target_min_len = float(dict_toml['Selection']['target_min_len'])
        self.target_max_tilt = float(dict_toml['Selection']['target_max_tilt'])

    def set_parameters(self):
        self.pcl_cutoff_dist = 1.1
        self.target_max_dist = 0.85
        self.target_min_dist = 0.3
        self.target_min_len = 0.25
        self.target_max_len = 0.40
        self.target_max_tilt = 30.

    def get_pcd_from_depth_img(self, depth_img, camera_param):
        self.depth_img = depth_img
        self.camera_param = camera_param
        pcl_raw = self.tfm_pcl_cam2global(self.cvt_depth2pcl(self.depth_img, self.camera_param), camera_param)
        pcd = self.downsample(pcl_raw, voxel_size=DOWNSAMPLE_VOXEL_SIZE)
        return pcd

    def fit_pcd(self, pcd, cluster_eps=DBSCAN_EPS, cluster_min_points=DBSCAN_MINPOINTS, verbose=True):
        pcd_list = []
        fitgeom_list = []

        pcd_array = np.array(pcd.points, dtype=np.float32)
        bflg_above_ground, xy_binidx, grid_xyzw, pcd_grounds_list = self.ground_fit(pcd_array)
        pcd_grounds_ary_pre_downsample = np.asarray(pcd_grounds_list[2].points) # pcd_grounds = [pcd_out_of_bin, pcd_groundseed, pcd_ground]
        pcd_grounds = self.downsample(pcd_grounds_ary_pre_downsample, voxel_size=DOWNSAMPLE_VOXEL_SIZE_GROUND)
        ground_points_ary = np.asarray(pcd_grounds.points)
        pcd_list += [ground_points_ary]
        fitgeom_list.append(self.get_mesh_ground())

        # TODO debug.error() send to cloud if above ground is all false
        if check_all_false(bflg_above_ground):
            return [], pcd_list, fitgeom_list, pcd_array, ground_points_ary
        labels, cluster_pcd = self.clustering(pcd_array[bflg_above_ground],
            eps=cluster_eps, min_points=cluster_min_points)
        pcd_list.append(cluster_pcd)

        line_list = self.line_fit(pcd_array[bflg_above_ground], labels)
        self.merge_lines(line_list)
        self.mark_multiline_clusters(line_list)
        self.extend_lines_to_ground(line_list, grid_xyzw)
        self.check_line_truncation(line_list)
        self.final_selection(line_list)
        if verbose:
            self.print_line_info(line_list)
        self.bkg_postprocess(line_list)
        self.remove_noise_lines(line_list, grid_xyzw)

        mesh_cylinders = self.get_line_fit_geometry(line_list)
        fitgeom_list += mesh_cylinders

        return line_list, pcd_list, fitgeom_list, pcd_array, ground_points_ary

    def cvt_depth2pcl(self, depth_img, camera_param):
        cx, cy = camera_param.center_xy
        fx, fy = camera_param.focal_xy
        DEPTH_MIN = 1e-3

        arr_y = np.arange(depth_img.shape[0], dtype=np.float32)
        arr_x = np.arange(depth_img.shape[1], dtype=np.float32)
        val_x, val_y = np.meshgrid(arr_x, arr_y)

        # TODO: rewrite axis convertion explicitly (i.e. zense clockwise rotation)
        tmp_x = +depth_img
        tmp_y = +depth_img * (val_y - cy) * (1. / fy)
        tmp_z = -depth_img * (val_x - cx) * (1. / fx)

        filled = (depth_img > DEPTH_MIN) * (depth_img < self.pcl_cutoff_dist + 0.2)

        filled_x = tmp_x[filled]
        filled_y = tmp_y[filled]
        filled_z = tmp_z[filled]

        pcl = np.stack([filled_x, filled_y, filled_z], axis=-1)
        return pcl

    def tfm_pcl_cam2global(self, pcl_camframe, camera_param):
        pcl_tmp = np.dot(pcl_camframe, camera_param.rot_mtx.transpose()) + camera_param.translation
        pcl_global = np.dot(pcl_tmp, self.camera_set_param.rot_mtx.transpose())
        return pcl_global


    def cvt_to_2d_image_xyd(self, input_points, camera_param):
        points = input_points.reshape(-1, 3)

        points_tmp = np.dot(points, self.camera_set_param.inv_rot_mtx.transpose())
        points_camframe = np.dot(points_tmp - camera_param.translation, camera_param.inv_rot_mtx.transpose())

        cx, cy = camera_param.center_xy
        fx, fy = camera_param.focal_xy
        depth = +points_camframe[:, 0]
        val_y = +points_camframe[:, 1] / depth * fy + cy
        val_x = -points_camframe[:, 2] / depth * fx + cx
        xyd = np.stack([val_x, val_y, depth], axis=-1)

        return xyd.reshape(input_points.shape)


    def downsample(self, pcl_raw, voxel_size):
        pcd_raw = self.cvt_numpy2open3d(pcl_raw, color=[0., 0., 1.])
        pcd = pcd_raw.voxel_down_sample(voxel_size=voxel_size)
        return pcd

    def cvt_numpy2open3d(self, pcl, color=None):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl.astype(np.float64))
        if not color is None:
            pcd.paint_uniform_color(color)
        return pcd


    def ground_fit(self, pcl):
        x_nbin = int( (X_MAX - X_MIN) / float(GRID_SIZE) + 1e-3 )
        y_nbin = int( (Y_MAX - Y_MIN) / float(GRID_SIZE) + 1e-3 )
        x_edge = np.linspace(X_MIN, X_MIN + GRID_SIZE * x_nbin, x_nbin + 1).reshape(1, -1)
        y_edge = np.linspace(Y_MIN, Y_MIN + GRID_SIZE * y_nbin, y_nbin + 1).reshape(1, -1)

        x_ctr = (x_edge[0, 1:] + x_edge[0, :-1]) * 0.5
        y_ctr = (y_edge[0, 1:] + y_edge[0, :-1]) * 0.5

        pcl_tmp = pcl.reshape(-1, 1, 3)
        x_binflg = (pcl_tmp[:, :, 0] >= x_edge[:, :-1]) * (pcl_tmp[:, :, 0] < x_edge[:, 1:])
        y_binflg = (pcl_tmp[:, :, 1] >= y_edge[:, :-1]) * (pcl_tmp[:, :, 1] < y_edge[:, 1:])
        x_binidx = np.argmax(x_binflg, axis=-1)
        y_binidx = np.argmax(y_binflg, axis=-1)
        x_binidx[(x_binflg.sum(axis=-1) == 0)] = -1
        y_binidx[(y_binflg.sum(axis=-1) == 0)] = -1
        xy_binidx = np.concatenate([x_binidx.reshape(-1,1), y_binidx.reshape(-1,1)], axis=-1)
        bflg_out_of_bin = (xy_binidx == -1).sum(-1).astype(np.bool)
        bflg_in_bin = (bflg_out_of_bin == False)

        grid_xyzw = np.zeros([x_nbin, y_nbin, 4], dtype=np.float64)
        for i_x in range(x_nbin):
            for i_y in range(y_nbin):
                in_bin = (x_binidx == i_x) * (y_binidx == i_y)
                pcl_in_bin = pcl[in_bin]
                valid = (pcl_in_bin[:, 2] < GROUND_SEED_Z_MAX)
                pcl_valid = pcl_in_bin[valid]
                if pcl_valid.shape[0] == 0:
                    z_val = 0.
                    wgt = 0.1
                else:
                    z_val = pcl_valid[:, 2].min()
                    wgt = 1.
                grid_xyzw[i_x, i_y] = [x_ctr[i_x], y_ctr[i_y], z_val, wgt]

        grid_xyzw = self.fill_empy_gridz(grid_xyzw, w_thres=0.1)

        pcd_groundseed = self.cvt_numpy2open3d(grid_xyzw.reshape(-1, 4)[:, :3], color=[1., 0., 1.])

        pcl_ground_seed_z = grid_xyzw[x_binidx, y_binidx, 2]
        bflg_ground_seed = (pcl[:, 2] < (pcl_ground_seed_z + GROUND_SEED_MARGIN)) * bflg_in_bin

        grid_xyzw = ransac_ground_fit(pcl[bflg_ground_seed], xy_binidx[bflg_ground_seed], grid_xyzw)
        grid_xyzw = self.fill_empy_gridz(grid_xyzw, w_thres=1.)
        grid_xyzw = self.smooth_ground(grid_xyzw, kernel_size=SMOOTHING_KERNEL)
        self.grid_xyzw = grid_xyzw

        bflg_in_range = (np.linalg.norm(pcl[:,:2], axis=-1) < self.pcl_cutoff_dist)
        bflg_valid_points = bflg_in_range * bflg_in_bin

        pcl_ground_z = grid_xyzw[x_binidx, y_binidx, 2]
        bflg_ground = (pcl[:, 2] < (pcl_ground_z + GROUND_MARGIN)) * bflg_valid_points

        bflg_above_ground = (bflg_ground == False) * bflg_valid_points

        pcd_out_of_bin = self.cvt_numpy2open3d(pcl[bflg_valid_points == False], color=[0.3, 0., 0.5])
        pcd_ground = self.cvt_numpy2open3d(pcl[bflg_ground], color=[0., 0., 0.5])

        pcd_all = [pcd_out_of_bin, pcd_groundseed, pcd_ground]
        return bflg_above_ground, xy_binidx, grid_xyzw, pcd_all

    def fill_empy_gridz(self, grid_xyzw, w_thres=0.1):
        filled = (grid_xyzw[:,:,3] > w_thres)
        empty = (filled == False)
#        print 'filled ', filled.shape, filled.sum()
#        print 'empty ', empty.shape, empty.sum()
        filled_xyzw = grid_xyzw[filled].reshape(-1, 1, 4)
        empty_xyzw = grid_xyzw[empty].reshape(1, -1, 4)
#        print 'filled_xyzw ', filled_xyzw.shape
#        print 'empty_xyzw ', empty_xyzw.shape
        dist_array = np.linalg.norm(filled_xyzw[:,:,:2] - empty_xyzw[:,:,:2], axis=-1)
#        print 'dist_array ', dist_array.shape
        if dist_array.shape[0] != 0:
            nearest_filled = np.argmin(dist_array, axis=0)
            grid_xyzw[empty, 2] = filled_xyzw[nearest_filled, 0, 2]
        return grid_xyzw

    def smooth_ground(self, grid_xyzw, kernel_size):
        vect = grid_xyzw[:,:,:2].reshape(1, -1, 2) - grid_xyzw[:,:,:2].reshape(-1, 1, 2)
        dsq = (vect ** 2).sum(axis=-1)
        z_orig = grid_xyzw[:,:,2].reshape(-1)
        wgt = grid_xyzw[:,:,3].reshape(-1)
        coeff = 0.5 / kernel_size ** 2
        fill_wgt = wgt * np.exp(-dsq * coeff)
        z_smooth = (z_orig * fill_wgt).sum(axis=-1) / fill_wgt.sum(axis=-1)
        grid_xyzw[:,:,2].reshape(-1)[:] = z_smooth
        return grid_xyzw

    def get_mesh_ground(self):
        return self.cvt_gridvtx2mesh(self.grid_xyzw) if self.grid_xyzw is not None else None

    def cvt_gridvtx2mesh(self, grid_vtx, double_sided=True):
        ngrid_x = grid_vtx.shape[0]
        ngrid_y = grid_vtx.shape[1]
        vertices = np.array(grid_vtx[:,:,:3].reshape(-1,3))
        triangles = []
        for i_x in range(grid_vtx.shape[0] - 1):
            for i_y in range(grid_vtx.shape[1] - 1):
                ivert_base = i_x * ngrid_y + i_y
                triangles.append([ivert_base, ivert_base+ngrid_y, ivert_base+1])
                triangles.append([ivert_base+ngrid_y+1, ivert_base+1, ivert_base+ngrid_y])
        triangles = np.array(triangles)
        if double_sided:
            triangles = np.concatenate([triangles, triangles[:,::-1]], axis=0)
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)
        mesh.paint_uniform_color([0.4, 0.4, 0.4])
        mesh.compute_vertex_normals()
        return mesh

    def clustering(self, pcl, eps=DBSCAN_EPS, min_points=DBSCAN_MINPOINTS):
        n_points = pcl.shape[0]
        print('Clustering {} points ...'.format(n_points),)
        pcd = self.cvt_numpy2open3d(pcl)
        labels_orig = np.array(
            pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        n_cluster = labels_orig.max() + 1
        print('Found {} clusters.'.format(n_cluster))
        cls_flg = (np.arange(n_cluster).reshape(-1,1) == labels_orig.reshape(1,-1))
        n_points_in_cls = cls_flg.sum(axis=-1)
        sortidx_cls = np.argsort(n_points_in_cls)[::-1]
        labels = np.ones(n_points, dtype=np.int32) * -1
        for i_cls in range(n_cluster):
            labels[cls_flg[sortidx_cls[i_cls]]] = i_cls
        colors = CMAP_CLUSTER(labels)
        colors[labels < 0] = 0.8
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        return labels, pcd

    def line_fit(self, pcl, labels):
        MAX_ITER_LINEFIT = 3
        RANSAC_N_ITER = 500
        CUT_PERCENTILE = 0.8
        DTHRES_INLIER = 0.020
        MAX_ROOT_Z = 0.20

        line_list = []
        n_cluster = labels.max() + 1
        print("Line fit on %d clusters ..." % n_cluster)

        do_break = False
        for i_cluster in range(n_cluster):
            pcl_cluster = pcl[(labels == i_cluster)]
            print("Cluster #{} : {} points".format(i_cluster, pcl_cluster.shape[0]))
            pcl_to_fit = pcl_cluster
            for i_iter in range(MAX_ITER_LINEFIT):
                n_to_fit = pcl_to_fit.shape[0]
                print(" - Iteration {} : {} points".format(i_iter, n_to_fit)),
                if n_to_fit < CLUSTER_MINPOINTS:
                    print(" - Too small!")
                    if i_iter == 0:
                        do_break = True
                    break

                length, tfm_mtx, is_outlier = ransac_line_fit(pcl_to_fit, n_iter=RANSAC_N_ITER, dthres_inlier=DTHRES_INLIER, cut_percentile=CUT_PERCENTILE, max_root_z=(MAX_ROOT_Z if i_iter==0 else -1.))
                if tfm_mtx is None:
                    print(" - Bad fit!")
                    break
                print(" - Good fit!")
                line_list.append(FittedLine(length, tfm_mtx, i_cluster))
                pcl_to_fit = pcl_to_fit[is_outlier]
            if do_break:
                break

        print("Found {} lines.".format(len(line_list)))

        return line_list


    def merge_lines(self, line_list):
        MERGE_THRES_COS = math.cos(15. * pi / 180.)
        MERGE_THRES_DIST = 0.10

        z_array = np.array([line.position[2] for line in line_list])
        sorted_idx = np.argsort(z_array)
        n_line = len(line_list)
        for i_line in range(n_line):
            line = line_list[sorted_idx[i_line]]

            for i_line2 in range(i_line + 1, n_line):
                line2 = line_list[sorted_idx[i_line2]]
                if not line2.parent is None:
                    continue
                to_line2 = line2.position - line.position_center
                dist_to_line2 = np.linalg.norm(to_line2)
                dir_to_line2 = to_line2 / dist_to_line2
                cos_to_line2 = np.dot(dir_to_line2, line.direction)
                if cos_to_line2 < MERGE_THRES_COS:
                    continue
                if dist_to_line2 > MERGE_THRES_DIST + line.length * 0.5:
                    continue
                line2.parent = line


    def count_lines_in_cluster(self, line_list):
        counts = {}
        for line in line_list:
            if not line.cluster_id in counts:
                counts[line.cluster_id] = 0
            counts[line.cluster_id] += 1
        return counts

    def mark_multiline_clusters(self, line_list):
        counts = self.count_lines_in_cluster(line_list)
        for line in line_list:
            if counts[line.cluster_id] > 1:
                line.is_multiline_cluster = True

    def extend_lines_to_ground(self, line_list, grid_xyzw):
        N_AVERAGE = 4
        MAX_R = GRID_SIZE
        MIN_SOLITARY_LEN = 0.100
        MAX_EXTEND_LEN = 0.200
        MAX_GROUNDED_EXTEND_LEN = 0.060
        COSZ_THRESHOLD = math.cos(45. * pi / 180.)
        flatten_grid_xyz = grid_xyzw[:,:,:3].reshape(-1, 3)
        for line in line_list:
            if not line.parent is None:
                continue
            if line.is_solitary and line.length < MIN_SOLITARY_LEN:
                continue
            if line.direction[2] < COSZ_THRESHOLD:
                continue
            flatten_grid_local_frame = line.tfm_to_local_frame(flatten_grid_xyz)
            flatten_grid_r = np.linalg.norm(flatten_grid_local_frame[:,:2], axis=-1)
            idx_sort = np.argsort(flatten_grid_r)[0:N_AVERAGE]
            weight = np.clip((MAX_R - flatten_grid_r[idx_sort]) / MAX_R, 0., 1.)
            weight_sum = weight.sum()
            if not weight_sum > 0.:
                continue
            ground_z_local_frame = np.dot(flatten_grid_local_frame[idx_sort,2], weight) / weight_sum
#            idx_min = idx_sort[0]
#            if flatten_grid_r[idx_min] > MAX_R:
#                continue
#            ground_z_local_frame = flatten_grid_local_frame[idx_min, 2]
            extend_len = -ground_z_local_frame
            if extend_len > MAX_EXTEND_LEN:
                continue
            line.extend_root(extend_len)
            line.is_grounded = (extend_len <= MAX_GROUNDED_EXTEND_LEN)


    def is_in_image(self, xyd, image_shape):
        TOP_MARGIN = 20
        SIDE_MARGIN = 20
        BOTTOM_MARGIN = 0
        x_val = xyd[0]
        y_val = xyd[1]
        if (y_val > SIDE_MARGIN
          and y_val < image_shape[0] - SIDE_MARGIN
          and x_val > TOP_MARGIN
          and x_val < image_shape[1] - BOTTOM_MARGIN):
            return True
        else:
            return False

    def check_line_truncation(self, line_list):
        SEEK_MARGIN = [10, 50]
        OPENING_ANGLE = 4.
        SECTOR_COLOR = 1
        DEPTH_MARGIN = 0.015
        MAX_OCCLUDING_PIXELS = 5

        sector_mask = np.zeros(self.depth_img.shape, dtype=np.uint8)

        for line in line_list:
            line.sector_mask = {}
            line.occlusion_mask = {}

            root_is_contained = 0
            tip_is_contained = 0
            is_occluded = False
            sector_mask = sector_mask

            xyd_ends = self.cvt_to_2d_image_xyd(line.position_ends, self.camera_param)
            line.xyd_ends = xyd_ends
            root_is_contained += self.is_in_image(xyd_ends[0], sector_mask.shape)
            tip_is_contained += self.is_in_image(xyd_ends[1], sector_mask.shape)
            if line.is_solitary and line.is_grounded:
                root_to_tip_xy = (xyd_ends[1] - xyd_ends[0])[:2]
                sector_angle = math.atan2(root_to_tip_xy[1], root_to_tip_xy[0]) / math.pi * 180.
                sector_radius = int(np.linalg.norm(root_to_tip_xy) * 0.5 + (SEEK_MARGIN[1] + SEEK_MARGIN[0]) * 0.5)
                center = (xyd_ends.sum(axis=0) * 0.5).astype(np.int32)
                sector_mask[:] = 0
                cv2.ellipse(sector_mask, (center[0], center[1]), (sector_radius, sector_radius), sector_angle, -OPENING_ANGLE * 0.5, +OPENING_ANGLE * 0.5, SECTOR_COLOR, SEEK_MARGIN[1] - SEEK_MARGIN[0])

                # TODO: what if tip is right on ?
                # TODO: handle cases where sector_mask goes out of image
                depth_in_sector = self.depth_img * sector_mask
                occlusion_mask = (depth_in_sector < xyd_ends[1, 2] + DEPTH_MARGIN) * (depth_in_sector > 0.)

                # TODO: Handle cases where the sector is out of frame in one camera
                if occlusion_mask.sum() > MAX_OCCLUDING_PIXELS:
                    is_occluded = True

                line.sector_mask = sector_mask.astype(np.bool)
                line.occlusion_mask = occlusion_mask
                
            line.tip_is_contained = (tip_is_contained != 0)
            line.is_contained = ((root_is_contained * tip_is_contained) != 0)
            line.is_occluded = is_occluded

    def final_selection(self, line_list):
        target_cosz_min = math.cos(self.target_max_tilt * pi / 180.)
        for line in line_list:
            if not (line.length > self.target_min_len and line.length < self.target_max_len):
                continue
            line_dist = line.xy_distance
            if not (line_dist > self.target_min_dist and line_dist < self.target_max_dist):
                continue
            if line.direction[2] < target_cosz_min:
                continue
            line.is_final = True


    def bkg_postprocess(self, line_list):
        EXTEND_LEN = 1.
        MIN_LEN = 0.2
        target_cosz_min = math.cos(self.target_max_tilt * pi / 180.)
        for line in line_list:
            if line.is_good:
                continue
            if line.direction[2] < target_cosz_min:
                continue
            if line.length < MIN_LEN:
                continue
            if not (line.length < self.target_max_len) or not line.tip_is_contained:
                line.extend_tip(EXTEND_LEN)


    def remove_noise_lines(self, line_list, grid_xyzw):
        MIN_LEN = 0.050
        n_orig = len(line_list)
        max_ground_z = np.max(grid_xyzw[:,:,2])
        z_threshold = max_ground_z + 0.40
        r_threshold = self.target_max_dist
        n_remove = 0
        for line in line_list:
            if line.is_good:
                continue
            if ((line.xy_distance > r_threshold and line.position[2] > z_threshold)
                or line.length < MIN_LEN):
                line.is_ignored = True
                n_remove += 1
        print('Noise line removal : {} -> {}'.format(n_orig, n_orig - n_remove))


    def print_line_info(self, line_list):
        print('### Candidate line info #############################')
        print('  Good  flg=[sol, nmlc, ground, tip, ends, unoccl, final]')
        print('-----------------------------------------------------')
        for line in line_list:
#            if not (line.is_solitary and not line.is_multiline_cluster and line.is_grounded):
            if line.length < 0.200:
                continue
            flags = [
                line.is_solitary,
                not line.is_multiline_cluster,
                line.is_grounded,
                line.tip_is_contained,
                line.is_contained,
                not line.is_occluded,
                line.is_final]
            print('  {} flg={} len={:.3f} dist={:.3f} tilt={:.1f}deg'.format(line.is_good, flags, line.length, line.xy_distance, math.acos(line.direction[2]) / pi * 180.))
        print('#####################################################')


    def get_line_fit_geometry(self, line_list):
        mesh_cylinders = []
        for line in line_list:
#            if line.is_ignored:
#                continue
            line_color = CMAP_CLUSTER(line.cluster_id)[:3]

            if line.length <= 0.0:
                print('`line.length` has non-positive value: {}'.format(line.length))
                continue
            mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.005, height=line.length)
            mesh_cylinder.compute_vertex_normals()
            mesh_cylinder.paint_uniform_color(line_color)
            mesh_cylinder.translate([0., 0., line.length * 0.5])
            mesh_cylinder.transform(line.tfm_mtx)
            mesh_cylinders.append(mesh_cylinder)
            line.add_mesh(mesh_cylinder)

            if False:
                mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.010)
                mesh_sphere.compute_vertex_normals()
                mesh_sphere.paint_uniform_color(line_color)
                mesh_sphere.transform(line.tfm_mtx)
                mesh_cylinders.append(mesh_sphere)
                line.add_mesh(mesh_sphere)

        return mesh_cylinders
