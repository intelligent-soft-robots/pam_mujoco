import os
from typing import Optional, Union

import pam_configuration
from .robot_type import RobotType

_tmp_folder = "/tmp/"

_MODEL_SUFFIX_PATH = "pam_mujoco" + os.sep + "models"


def get_models_path() -> str:
    config_path = str(pam_configuration.get_path())
    abs_path = config_path + os.sep + _MODEL_SUFFIX_PATH
    return abs_path


def get_main_template_xml() -> str:
    path = get_models_path() + os.sep + "template.xml"
    with open(path, "r") as f:
        template = f.read()
    return template


def get_robot_templates_path(robot_type: Optional[RobotType] = None) -> str:
    root = get_models_path() + os.sep + "robot_templates"
    if robot_type is None:
        return root
    if robot_type == RobotType.PAMY1:
        return root + os.sep + "pamy1"
    return root + os.sep + "pamy2"


def get_tmp_path(model_name: Union[str, os.PathLike]) -> str:
    global _tmp_folder
    path = os.path.join(_tmp_folder, model_name)
    try:
        os.mkdir(path)
    except FileExistsError:
        pass
    return path


def get_robot_xml(filename: str, ir: str, robot_type: Optional[RobotType]) -> str:
    path = get_robot_templates_path(robot_type) + os.sep + filename + ".xml"
    with open(path, "r") as f:
        template = f.read()
    template = template.replace("?id?", str(ir))
    return template


def get_robot_body_xml(ir: str, muscles: bool, robot_type: RobotType) -> str:
    if muscles:
        return get_robot_xml("body_template", ir, robot_type)
    return get_robot_xml("body_template_no_muscles", ir, robot_type)


def get_robot_tendon_xml(ir: str) -> str:
    return get_robot_xml("tendon_template", ir, robot_type=None)


def get_robot_actuator_xml(ir: str) -> str:
    return get_robot_xml("actuator_template", ir, robot_type=None)


def write_robot_body_xml(
    model_name: Union[str, os.PathLike], ir: str, muscles: bool, robot_type: RobotType
) -> str:
    xml = get_robot_body_xml(ir, muscles, robot_type)
    filename = "robot_body_" + str(ir) + ".xml"
    with open(get_tmp_path(model_name) + os.sep + filename, "w+") as f:
        f.write(xml)
    return filename


def write_robot_actuator_xml(model_name: Union[str, os.PathLike], ir: str) -> str:
    xml = get_robot_actuator_xml(ir)
    filename = "robot_actuator_" + str(ir) + ".xml"
    with open(get_tmp_path(model_name) + os.sep + filename, "w+") as f:
        f.write(xml)
    return filename


def get_model_path(model_name: str) -> str:
    return get_tmp_path(model_name) + os.sep + model_name + ".xml"


def write_model_xml(model_name: str, xml_content: str) -> str:
    path = get_model_path(model_name)
    with open(path, "w+") as f:
        f.write(xml_content)
    return path
