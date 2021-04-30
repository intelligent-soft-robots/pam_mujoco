from . import models
import pam_mujoco_wrp


class MujocoItem:

    def __init__(self,
                 color=None,
                 control=False,
                 until_table_contact=False
                 until_robot_contact=False):
        self.color=color
        self.control=control


class MujocoHandle:

    def __init__(mujoco_id:str,
                 burst_mode:bool,
                 accelerated_time:bool,
                 time_step: Optional[float] = 0.002,
                 table: Optional[bool] = False,
                 robot1: Optional[bool] = False,
                 robot1_position: Optional[list]=[0.1,0.0,-0.44],
                 robot2: Optional[bool] = False,
                 robot2_position: Optional[list] = [1.6,3.4,-0.44],
                 robot2_orientation: Optional[list] = [-1,0,0,0,-1,0],
                 balls: Optional[list[MujocoItem]]=[],
                 goals: Optional[list[MujocoItem]]=[],
                 hit_points: Optional[list[MujocoItem]]=[],
                 muscles: Optional[bool]=True):

        mf_kwargs = {k:v for k.v in locals().items()
                     if k!="self"}
        items = model_factory(**locals())
    
        config = pam_mujoco_wrp.Config(mujoco_id)
        config.set_burst_mode(burst_mode)
        config.set_accelerated_time(accelerated_time)
        config.set_model_path(items["path"])

        if balls:
            balls = items["balls"]
            for ball in balls:
                
