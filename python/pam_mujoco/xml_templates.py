import itertools
import typing as t
from collections.abc import Iterable

import numpy as np
from scipy.spatial.transform import Rotation

from . import paths
from .robot_type import RobotType


def _str(a):
    if isinstance(a, str):
        return a
    if isinstance(a, Iterable):
        return " ".join([str(a_) for a_ in a])
    return str(a)


def _from_template(template: str, /, **kwargs) -> str:
    """Replace variables in template based on given keyword arguments.

    Iterable values are converted to a string "v1 v2 v3 ...", all other values are
    converted using ``str()``.

    Args:
        template: A template with variables that are compatible with `str.format()`
            (i.e. something like "value='{foo}'").
        **kwargs: Value for the template variables.  Names need to match with the ones
            used in the template.

    Returns:
        Template string with all variables replaced with actual values.
    """
    values = {k: _str(v) for k, v in kwargs.items()}
    result = template.format(**values)

    return result


def _mujoco_quaternion(rotation: Rotation) -> np.ndarray:
    """Convert a Rotation object to a MuJoCo-compatible quaternion (w, x, y, z)."""
    # roll moves the `w` from the last position to the first
    return np.roll(rotation.as_quat(), 1)


def get_free_joint_body_xml(model_name, name, geom_type, position, size, color, mass):
    template = str(
        '<body pos="{position}" name="{name}">\n'
        + '<geom name="{name_geom}" type="{geom_type}" '
    )
    if mass > 0:
        template += 'size="{size}" rgba="{color}" pos="{position}" mass="{mass}"/>\n'
    else:
        template += 'size="{size}" rgba="{color}" pos="{position}" />\n'

    template += str('<joint type="free" name="{name_joint}"/>\n' + "</body>\n")

    name_geom = name + "_geom"
    name_joint = name + "_joint"

    xml = _from_template(template, **locals())

    nb_bodies = 1

    return xml, name_geom, name_joint, nb_bodies


def get_table_xml(model_name, name, position, size, color, xy_axes):
    template_body = '<body pos = "{position}" name = "{name}" euler="{xy_axes}">'
    template_geom_plate = str(
        '<geom name="{name_plate_geom}" type="box"' + ' size="{size}" rgba="{color}" />'
    )

    template_geom_leg = str(
        '<geom name="leg_{index}" pos= "{position}"'
        + ' type="box" size="0.02 0.02 0.38" rgba="0.1 0.1 0.1 1.0"/>'
    )
    template_geom_net = str(
        '<geom name="{name_net_geom}" pos = "0.0 0.0 0.07625"'
        + ' type="box" size="0.7825 0.02 0.07625" rgba="0.1 0.1 0.1 0.4"/>'
    )

    name_plate_geom = name + "_plate"
    name_net_geom = name + "_net"

    xml = [
        _from_template(template_body, **locals()),
        _from_template(template_geom_plate, **locals()),
        _from_template(template_geom_net, **locals()),
    ]

    # computing the positions of the legs
    legs = np.array([0.9 * s for s in size[:2]])
    combs = np.array(list(itertools.product([-1, 1], repeat=2)))
    legs = legs * combs
    legs_z = np.array([-0.38] * 4).reshape(-1, 1)
    legs = np.append(legs, legs_z, axis=1)

    for index, position in enumerate(legs):
        xml_leg = _from_template(template_geom_leg, **locals())
        xml.append(xml_leg)

    xml.append("</body>\n")

    nb_bodies = 7  # plate, net, 4 legs

    return "\n".join(xml), name_plate_geom, name_net_geom, nb_bodies


def get_goal_xml(model_name, name, position, radius1, radius2, color1, color2):
    template = str(
        '<body pos = "{position}" name="{name}">\n'
        + '<geom name="{name_inner}" type="cylinder" '
        + ' pos="0 0 0" size="{radius1} 0.0005" rgba="{color1}"/>\n'
        + '<geom name="{name_outer}" type="cylinder" pos="0 0 0" '
        + 'size="{radius2} 0.0004" rgba="{color2}"/>\n'
        + "</body>\n"
    )

    name_inner = name + "_inner"
    name_outer = name + "_outer"

    xml = _from_template(template, **locals())

    nb_bodies = 1

    return xml, 1


def get_robot_xml(
    model_name: str,
    name: str,
    position: t.Union[str, t.Iterable[float]],
    orientation: t.Optional[Rotation],
    muscles: bool,
    robot_type: RobotType,
) -> t.Tuple[str, str, str, int]:
    # using the robot template xml file in pam_mujoco/models/robot_templates
    # to generate robot description xml file that will be included in the main
    # xml file
    filename = paths.write_robot_body_xml(model_name, name, muscles, robot_type)

    optional = {}
    if orientation is not None:
        optional["mujoco_quat"] = _mujoco_quaternion(orientation)

        template = """
            <body name="{name}" pos="{position}" quat="{mujoco_quat}">
              <include file="{filename}"/>
            </body>
        """
    else:
        template = """
            <body name="{name}" pos="{position}">
              <include file="{filename}"/>
            </body>
        """

    xml = _from_template(
        template, name=name, position=position, filename=filename, **optional
    )

    nb_bodies = 25  # just from counting in the xml template file

    # as according to the template xml files
    # in pam_configuration/config/pam_mujoco/models/robot_templates/
    joint = name + "_joint_base_rotation"
    geom_racket = name + "_racket"

    return xml, joint, geom_racket, nb_bodies


def get_contacts_xml(robots, balls, tables, solrefs, gaps):
    template = '<pair geom1="{geom1}" geom2="{geom2}" {attrs} />'

    def get_attrs(geom1, geom2):
        damping = None
        gap = None
        if geom1 in solrefs:
            if geom2 in solrefs[geom1]:
                damping = _str(solrefs[geom1][geom2])
        if geom1 in gaps:
            if geom2 in gaps[geom1]:
                gap = str(gaps[geom1][geom2])
        r = ""
        if damping:
            r += ' solref="' + damping + '" '
        if gap:
            r += ' gap="' + gap + '"'

        return r

    def get_xml(geom1, geom2, type1, type2):
        c = template.replace("{geom1}", geom1)
        c = c.replace("{geom2}", geom2)
        c = c.replace("{attrs}", get_attrs(type1, type2))
        return c

    contacts = []

    for ball in balls:
        contacts.append(get_xml(ball.geom, "floor", "ball", "floor"))
        for robot in robots:
            contacts.append(get_xml(ball.geom, robot.geom_racket, "ball", "racket"))
        for table in tables:
            contacts.append(get_xml(ball.geom, table.geom_plate, "ball", "table"))
            contacts.append(get_xml(ball.geom, table.geom_net, "ball", "net"))

    for robot in robots:
        contacts.append(get_xml("floor", robot.geom_racket, "floor", "racket"))

    return "\n".join(contacts)
