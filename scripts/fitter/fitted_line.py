import numpy as np

def flag_to_binary(array):
    b = '0b'
    for el in array:
        b += str(int(el))
    return b


class FittedLine(object):

    def __init__(self, length, tfm_mtx, cluster_id):
        self._length = length
        self._tfm_mtx = tfm_mtx
        self._cluster_id = cluster_id
        self._parent = None
        self._children = []
        self._meshes = []
        self.xyd_ends = {}
        # TODO make class which express status of asparagas and move flag evaluation method there
        self.is_multiline_cluster = False
        self.is_grounded = False
        self.is_contained = False
        self.is_occluded = True
        self.is_final = False
        self.is_ignored = False

    @property
    def cluster_id(self):
        return self._cluster_id

#    @cluster_id.setter
#    def cluster_id(self, value):
#        self._cluster_id = value
    @property
    def status(self):
        return 100 + int(flag_to_binary([self.is_solitary, not self.is_multiline_cluster, self.is_grounded,
                                         self.is_contained, not self.is_occluded, self.is_final]), 2)
    @property
    def flag(self):
        # TODO define flag
        return 1 if self.is_good else self.status

    @staticmethod
    def flag_to_status(flag):
        if flag == 1:
            return [True] * 6
        status = []
        for a in bin(flag - 100)[2:]:
            status.append(a == '1')
        return status


    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, value):
        if not self._parent is None:
            print('Parent is already set!')
            return
        self._parent = value
        self._cluster_id = self._parent.cluster_id
        self._parent._children.append(self)

    @property
    def n_children(self):
        return len(self._children)

    @property
    def length(self):
        return self._length

    @property
    def tfm_mtx(self):
        return self._tfm_mtx

    @property
    def position(self):
        return self._tfm_mtx[:3,3]

    @property
    def direction(self):
        return self._tfm_mtx[:3,2]

    @property
    def position_center(self):
        return self.position + self.direction * (self.length * 0.5)

    @property
    def position_tip(self):
        return self.position + self.direction * (self.length * 1.)

    @property
    def position_ends(self):
        return np.stack([self.position, self.position_tip], axis=0)

    @property
    def xy_distance(self):
        return np.linalg.norm(self.position[:2])

    @property
    def distance(self):
        return np.linalg.norm(self.position)

    def tfm_to_local_frame(self, points):
        if points.ndim != 2 or points.shape[-1] != 3:
            print('Input must be a 2D array of shape (n, 3)!')
            return None
        if not hasattr(self, '_inv_rot_mtx'):
            self._inv_rot_mtx = np.linalg.inv(self._tfm_mtx[:3, :3])
        points_local = np.dot((points - self.position), self._inv_rot_mtx.transpose())
        return points_local

    def extend_root(self, extend_len):
        self._length += extend_len
        self._tfm_mtx[:3,3] -= extend_len * self.direction

    def extend_tip(self, extend_len):
        self._length += extend_len

    @property
    def is_solitary(self):
        return (self._parent is None and self.n_children == 0)

    @property
    def is_good(self):
        return (self.is_solitary and not self.is_multiline_cluster and self.is_grounded
                and self.is_contained and not self.is_occluded and self.is_final)

    def add_mesh(self, mesh):
        self._meshes.append(mesh)

    def delete_mesh(self):
        self._meshes = []

    @property
    def meshes(self):
        return self._meshes

