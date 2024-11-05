import pathlib
import re

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import pam_mujoco
import pam_mujoco.xml_templates as m


def test_str() -> None:
    assert m._str("foo") == "foo"
    assert m._str(42) == "42"
    assert m._str([1, 2, 3]) == "1 2 3"
    assert m._str((1, 2, 3)) == "1 2 3"
    assert m._str(np.array((1, 2, 3))) == "1 2 3"


def test_from_template() -> None:
    tmpl = "Test {foo} template {bar}"
    assert m._from_template(tmpl, foo="hello", bar=42) == "Test hello template 42"
    # unused values should silently be ignored
    assert (
        m._from_template(tmpl, foo="hello", bar=42, baz="unused")
        == "Test hello template 42"
    )

    with pytest.raises(KeyError):
        m._from_template(tmpl, foo="hello")  # bar missing


def test_mujoco_quaternion() -> None:
    quat_xyzw = np.array([0.09365858, 0.18731716, 0.28097574, 0.93658581])
    quat_wxyz = np.array([0.93658581, 0.09365858, 0.18731716, 0.28097574])

    rot = Rotation.from_quat(quat_xyzw)
    result = m._mujoco_quaternion(rot)

    np.testing.assert_array_almost_equal(result, quat_wxyz)


def test_get_table_xml() -> None:
    out_xml, out_name_plate_geom, out_name_net_geom = m.get_table_xml(
        "MODEL-NAME",
        "NAME",
        [0, 1, 2],
        [1, 2, 0.1],
        (1.0, 0, 0, 1.0),
        Rotation.from_quat(np.array([0.0, 0.0, 0.4472136, 0.89442719])),
    )

    # strip whitespaces
    out_xml = re.sub(" +", " ", out_xml.strip())

    assert out_xml == (
        '<body pos="0 1 2" name="NAME"'
        ' quat="0.8944271889999159 0.0 0.0 0.4472135994999579">\n'
        '<geom name="NAME_plate" type="box" size="1 2 0.1" rgba="1.0 0 0 1.0" />\n'
        '<geom name="NAME_net" pos="0.0 0.0 0.07625" type="box"'
        ' size="0.7825 0.02 0.07625" rgba="0.1 0.1 0.1 0.4"/>\n'
        '<geom name="leg_0" pos="-0.9 -1.8 -0.38" type="box" size="0.02 0.02 0.38"'
        ' rgba="0.1 0.1 0.1 1.0"/>\n'
        '<geom name="leg_1" pos="-0.9 1.8 -0.38" type="box" size="0.02 0.02 0.38"'
        ' rgba="0.1 0.1 0.1 1.0"/>\n'
        '<geom name="leg_2" pos="0.9 -1.8 -0.38" type="box" size="0.02 0.02 0.38"'
        ' rgba="0.1 0.1 0.1 1.0"/>\n'
        '<geom name="leg_3" pos="0.9 1.8 -0.38" type="box" size="0.02 0.02 0.38"'
        ' rgba="0.1 0.1 0.1 1.0"/>\n'
        "</body>"
    )

    assert out_name_plate_geom == "NAME_plate"
    assert out_name_net_geom == "NAME_net"


def test_get_robot_xml_with_orientation() -> None:
    model_name = "pam_mujoco_test_xml_template"

    out_xml, out_joint, out_geom_racket = m.get_robot_xml(
        model_name,
        "NAME",
        [0, 1, 2],
        Rotation.from_quat(np.array([0.0, 0.0, 0.4472136, 0.89442719])),
        muscles=False,
        robot_type=pam_mujoco.RobotType.PAMY2,
    )

    # strip whitespaces
    out_xml = re.sub(" +", " ", out_xml.strip())

    assert out_xml == (
        '<body name="NAME" pos="0 1 2"'
        ' quat="0.8944271889999159 0.0 0.0 0.4472135994999579">\n'
        ' <include file="robot_body_NAME.xml"/>\n'
        " </body>"
    )
    assert out_joint == "NAME_joint_base_rotation"
    assert out_geom_racket == "NAME_racket"

    assert pathlib.Path(
        "/tmp/pam_mujoco_test_xml_template/robot_body_NAME.xml"
    ).is_file()
