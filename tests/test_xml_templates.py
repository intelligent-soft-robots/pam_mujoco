import numpy as np
from scipy.spatial.transform import Rotation

import pam_mujoco.xml_templates as m


def test_mujoco_quaternion():
    quat_xyzw = np.array([0.09365858, 0.18731716, 0.28097574, 0.93658581])
    quat_wxyz = np.array([0.93658581, 0.09365858, 0.18731716, 0.28097574])

    rot = Rotation.from_quat(quat_xyzw)
    result = m._mujoco_quaternion(rot)

    np.testing.assert_array_almost_equal(result, quat_wxyz)
