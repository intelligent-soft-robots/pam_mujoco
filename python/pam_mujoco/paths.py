import os

# models folder expected to have been
# installed in /opt/mpi-is/models.
# see: CMakeLists.txt
_models_folder = "/opt/mpi-is/models"
_tmp_folder = "/tmp/"


def get_models_path():
    #
    # deprecated : used to use the in-source
    # models folder
    #
    # from catkin_pkg import workspaces
    # packages = workspaces.get_spaces()
    # context_pkg_path = [ p for p in packages
    #                     if p.endswith("pam_mujoco") ][0]
    # return os.path.join(context_pkg_path,
    #                    "models")
    global _models_folder
    return _models_folder


def get_main_template_xml():
    path = get_models_path() + os.sep + "template.xml"
    with open(path, "r") as f:
        template = f.read()
    return template


def get_robot_templates_path(pamy1: bool = None) -> str:
    root = get_models_path() + os.sep + "robot_templates"
    if pamy1 is None:
        return root
    if pamy1:
        return root + os.sep + "pamy1"
    return root + os.sep + "pamy2"


def get_tmp_path(model_name):
    global _tmp_folder
    path = os.path.join(_tmp_folder, model_name)
    try:
        os.mkdir(path)
    except FileExistsError:
        pass
    return path


def get_robot_xml(filename: str, ir: str, pamy1: bool) -> str:
    path = get_robot_templates_path(pamy1) + os.sep + filename + ".xml"
    with open(path, "r") as f:
        template = f.read()
    template = template.replace("?id?", str(ir))
    return template


def get_robot_body_xml(ir: str, muscles: bool, pamy1: bool) -> str:
    if muscles:
        return get_robot_xml("body_template", ir, pamy1)
    return get_robot_xml("body_template_no_muscles", ir, pamy1)


def get_robot_tendon_xml(ir):
    return get_robot_xml("tendon_template", ir, pamy1=None)


def get_robot_actuator_xml(ir):
    return get_robot_xml("actuator_template", ir, pamy1=None)


def write_robot_body_xml(model_name: str, ir: str, muscles: bool, pamy1: bool) -> str:
    xml = get_robot_body_xml(ir, muscles, pamy1)
    filename = "robot_body_" + str(ir) + ".xml"
    with open(get_tmp_path(model_name) + os.sep + filename, "w+") as f:
        f.write(xml)
    return filename


def write_robot_actuator_xml(model_name, ir):
    xml = get_robot_actuator_xml(ir)
    filename = "robot_actuator_" + str(ir) + ".xml"
    with open(get_tmp_path(model_name) + os.sep + filename, "w+") as f:
        f.write(xml)
    return filename


def get_model_path(model_name):
    return get_tmp_path(model_name) + os.sep + model_name + ".xml"


def write_model_xml(model_name, xml_content):
    path = get_model_path(model_name)
    with open(path, "w+") as f:
        f.write(xml_content)
    return path
