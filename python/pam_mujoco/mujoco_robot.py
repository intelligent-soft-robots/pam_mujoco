import pathlib
import typing as t

from scipy.spatial.transform import Rotation

import pam_models
import pam_interface
from .robot_type import RobotType


class MujocoRobot:
    NO_CONTROL = 0
    JOINT_CONTROL = 1
    PRESSURE_CONTROL = 2

    CONSTANT_CONTROL = 1
    COMMAND_ACTIVE_CONTROL = 2

    def __init__(
        self,
        robot_type: RobotType,
        segment_id: str,
        position: t.Sequence[float] = (0.0, 0.0, 1.21),
        orientation: t.Optional[Rotation] = None,
        control: int = NO_CONTROL,
        active_only_control: int = CONSTANT_CONTROL,
        json_control_path: t.Optional[pathlib.Path] = None,
        json_ago_hill_path: t.Optional[pathlib.Path] = None,
        json_antago_hill_path: t.Optional[pathlib.Path] = None,
    ) -> None:
        if orientation is None:
            orientation = Rotation.identity()
        if json_control_path is None:
            json_control_path = pathlib.Path(
                pam_interface.Pamy2DefaultConfiguration.get_path(True)
            )
        if json_ago_hill_path is None:
            json_ago_hill_path = pam_models.get_default_config_path()
        if json_antago_hill_path is None:
            json_antago_hill_path = pam_models.get_default_config_path()

        self.robot_type = robot_type
        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
        self.control = control
        self.active_only_control = active_only_control
        self.json_control_path = json_control_path
        self.json_ago_hill_path = json_ago_hill_path
        self.json_antago_hill_path = json_antago_hill_path
