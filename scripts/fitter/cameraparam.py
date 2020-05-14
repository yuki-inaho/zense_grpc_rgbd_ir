import numpy as np
import math
import toml


def get_camera_param(toml_path):
    dict_toml = toml.load(open(toml_path))
    n_camera = int(dict_toml['General']['n_camera'])
    camera_param = {}
    for i_cam in range(n_camera):
        key = 'Camera{}'.format(i_cam)
        camera_name = dict_toml[key]['camera_name']
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

        camera_param[camera_name] = CameraParam()
        camera_param[camera_name].set_camera_param(K, R, P, size)
        camera_param[camera_name].set_tf_rot_and_trans([roll, pitch, yaw], [tx, ty, tz])
    return camera_param

class CameraParam:
    def __init__(self):
        self._K = None
        self._R = None
        self._P = None
        self._shape = None
        self._transform_matrix = np.identity(4, dtype=np.float64)
        self.set_tf_matrix(self._transform_matrix)

    def set_camera_param(self, k, r, p, shape):
        self._K = k
        self._R = r
        self._P = p
        self._shape = shape

    def set_tf_rot_and_trans(self, rpy, xyz):
        tfm_mtx = np.identity(4, dtype=np.float64)
        tfm_mtx[:3,:3] = np.dot(self._axis_rot_mtx(2, rpy[2]), np.dot(self._axis_rot_mtx(1, rpy[1]), self._axis_rot_mtx(0, rpy[0])))
        tfm_mtx[:3,3] = np.array(xyz)
        self.set_tf_matrix(tfm_mtx)

    def _axis_rot_mtx(self, axis, deg):
        rad = deg / 180. * math.pi
        mtx = np.zeros((3,3), dtype=np.float64)
        i_0 = axis % 3
        i_1 = (axis + 1) % 3
        i_2 = (axis + 2) % 3
        mtx[i_0, i_0] = 1.
        mtx[i_1, i_1] = +math.cos(rad)
        mtx[i_1, i_2] = -math.sin(rad)
        mtx[i_2, i_1] = +math.sin(rad)
        mtx[i_2, i_2] = +math.cos(rad)
        return mtx

    def set_tf_matrix(self, matrix):
        assert matrix.shape == (4, 4)
        self._transform_matrix[:] = matrix[:]
        self._inv_rot_mtx = np.linalg.inv(self.rot_mtx)

    @property
    def translation(self):
        return self._transform_matrix[:3,3]

    @property
    def rot_mtx(self):
        return self._transform_matrix[:3,:3]

    @property
    def inv_rot_mtx(self):
        return self._inv_rot_mtx

    @property
    def height(self):
        return self._shape[0]

    @property
    def width(self):
        return self._shape[1]

    @property
    def shape(self):
        return self._shape

    @property
    def size(self):
        return self._shape[0] * self._shape[1]

    @property
    def focal_xy(self):
        return self._K[0], self._K[4]

    @property
    def center_xy(self):
        return self._K[2], self._K[5]
