import pam_models
import pam_interface


class MujocoRobot:

    NO_CONTROL = 0
    JOINT_CONTROL = 1
    PRESSURE_CONTROL = 2

    CONSTANT_CONTROL = 1
    COMMAND_ACTIVE_CONTROL = 2

    def __init__(
        self,
        pamy1: bool,
        segment_id,
        position=[0.0, 0.0, 1.21],
        orientation="0.0 0.0 0.0",
        control=NO_CONTROL,
        active_only_control=CONSTANT_CONTROL,
        json_control_path=pam_interface.Pamy2DefaultConfiguration.get_path(),
        json_ago_hill_path=pam_models.get_default_config_path(),
        json_antago_hill_path=pam_models.get_default_config_path(),
    ):

        self.pamy1 = pamy1
        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
        self.control = control
        self.active_only_control = active_only_control
        self.json_control_path = json_control_path
        self.json_ago_hill_path = json_ago_hill_path
        self.json_antago_hill_path = json_antago_hill_path
