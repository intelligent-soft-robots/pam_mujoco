from . import models
import pam_mujoco_wrp


class MujocoItem:

    NO_CONTROL=0
    CONSTANT_CONTROL=1
    COMMAND_ACTIVE_CONTROL=2
    
    def __init__(self,
                 segment_id,
                 color=None,
                 control=False,
                 contact_type=None
                 configuration_path=None):
        self.segment_id=segment_id,
        self.color=color
        self.control=control
        self.contact_type=contact_type,
        self.configuration_path=configuration_path
        

def _get_mujoco_item_control(mujoco_item_type:pam_mujoco.MujocoItemTypes,
                             mujoco_item:MujocoItem,
                             model_item:dict):
                             
    
    if mujoco_item.control == self.NO_CONTROL:
        return None

    if mujoco_item.control == self.CONSTANT_CONTROL:
        active_only=False

    if mujoco_item.control == self.CONSTANT_CONTRO:
        active_only=True
        
    return pam_mujoco.MujocoItemControl(mujoco_item_type,
                                        mujoco_item.segment_id,
                                        model_item.joint,
                                        model_item.index_qpos,
                                        model_item.index_qvel,
                                        model_item.geom,
                                        active_only,
                                        mujoco_item.configuration_path,
                                        mujoco_item.contact_type)
                                        
        
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
                 ball_colors: Optional[list]=[],
                 goals: Optional[list[MujocoItem]]=[],
                 hit_points: Optional[list[MujocoItem]]=[],
                 muscles: Optional[bool]=True):

        items = model_factory(mujoco_id,
                              time_step=time_step,
                              table=table,
                              balls=len(balls),
                              ball_colors=ball_colors,
                              robot1=robot1,
                              robot1_position=robot1_position,
                              robot2=robot2,
                              robot2_position=robot2_position,
                              robot2_orientation=robot2_orientation,
                              goals=len(goals),
                              hit_points=len(hit_points),
                              muscles=muscles)
                              
        config = pam_mujoco_wrp.Config(mujoco_id)
        config.set_burst_mode(burst_mode)
        config.set_accelerated_time(accelerated_time)
        config.set_model_path(items["path"])
        config.set_racket_and_table(items["racket"].geom,
                                    items["table"].geom)
        
        _get_ball = partial(_get_mujoco_item_control,
                            pam_mujoco.MujocoItemTypes.ball)
        _get_hit_point = partial(_get_mujoco_item_control,
                                 pam_mujoco.MujocoItemTypes.hit_point)
        _get_goal = partial(_get_mujoco_item_control,
                            pam_mujoco.MujocoItemTypes.goal)
        
        mujoco_item_controls = [_get_ball(mujoco_item,model_item)
                                for mujoco_item,model_item
                                in zip(balls,items["balls"])]
        
        mujoco_item_controls.extend([_get_hit_point(mujoco_item,model_item)
                                     for mujoco_item,model_item
                                     in zip(balls,items["hit_points"])])

        mujoco_item_controls.extend([_get_goal(mujoco_item,model_item)
                                     for mujoco_item,model_item
                                     in zip(balls,items["goals"])])

        for mujoco_item_control in mujoco_item_controls:
            config.add_control(mujoco_item_control)

            
        pam_mujoco.set_mujoco_config(config)        
