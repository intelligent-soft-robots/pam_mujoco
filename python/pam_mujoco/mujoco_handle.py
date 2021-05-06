import logging
from functools import partial
from .mujoco_robot import MujocoRobot
from .mujoco_item import MujocoItem
from . import models
import shared_memory
import pam_mujoco_wrp
import o80_pam
        
def _get_mujoco_item_control(mujoco_item_type:pam_mujoco_wrp.MujocoItemTypes,
                             mujoco_item:MujocoItem,
                             model_item:dict) -> pam_mujoco_wrp.MujocoItemControl:
                             
    
    if mujoco_item.control == MujocoItem.NO_CONTROL:
        return None

    if mujoco_item.control == MujocoItem.CONSTANT_CONTROL:
        active_only=False

    if mujoco_item.control == MujocoItem.COMMAND_ACTIVE_CONTROL:
        active_only=True
        
    return pam_mujoco_wrp.MujocoItemControl(mujoco_item_type,
                                            mujoco_item.segment_id,
                                            model_item.joint,
                                            model_item.index_qpos,
                                            model_item.index_qvel,
                                            model_item.geom,
                                            active_only,
                                            mujoco_item.configuration_path,
                                            mujoco_item.contact_type)
                                        

def _get_mujoco_robot_control(mujoco_robot: MujocoRobot,
                              model_item: dict):

    active_only = (mujoco_robot.active_only_control == MujocoRobot.COMMAND_ACTIVE_CONTROL)
    
    if mujoco_robot.control == MujocoRobot.JOINT_CONTROL:
        return pam_mujoco_wrp.MujocoRobotJointControl(mujoco_robot.segment_id,
                                                      model_item.joint,
                                                      active_only)
    if mujoco_robot.control == MujocoRobot.PRESSURE_CONTROL:
        return pam_mujoco_wrp.MujocoRobotPressureControl(mujoco_robot.segment_id,
                                                         model_item.joint,
                                                         active_only,
                                                         mujoco_robot.configuration)
    return None

        

class MujocoHandle:
    
    def __init__(self,
                 mujoco_id:str,
                 burst_mode: bool=False,
                 accelerated_time:bool=False,
                 time_step: float = 0.002,
                 table: bool = False,
                 robot1: MujocoRobot = None,
                 robot2: MujocoRobot = None,
                 balls: list=[],
                 goals: list=[],
                 hit_points: list=[]):

        self._mujoco_id = mujoco_id

        # creating the mujoco xml model file

        logging.info("creating the xml model file for {}".format(mujoco_id))

        items = models.model_factory(mujoco_id,
                                     time_step=time_step,
                                     table=table,
                                     balls=balls,
                                     goals=goals,
                                     hit_points=hit_points,
                                     robot1=robot1,
                                     robot2=robot2)


        # creating the mujoco config

        logging.info("creating mujoco configuration for {}".format(mujoco_id))
        
        config = pam_mujoco_wrp.MujocoConfig(mujoco_id)
        config.set_burst_mode(burst_mode)
        config.set_accelerated_time(accelerated_time)
        config.set_model_path(items["path"])

        if items["robot1"]:
            config.set_racket_robot1(items["robot1"].geom_racket)

        if items["robot2"]:
            config.set_racket_robot2(items["robot2"].geom_racket)

        if items["table"]:
            config.set_table(items["table"].geom_plate)
        
        _get_ball = partial(_get_mujoco_item_control,
                            pam_mujoco_wrp.MujocoItemTypes.ball)
        _get_hit_point = partial(_get_mujoco_item_control,
                                 pam_mujoco_wrp.MujocoItemTypes.hit_point)
        _get_goal = partial(_get_mujoco_item_control,
                            pam_mujoco_wrp.MujocoItemTypes.goal)

        mujoco_item_controls=[]
        if balls:
            mujoco_item_controls.extend([_get_ball(mujoco_item,model_item)
                                         for mujoco_item,model_item
                                         in zip(balls,items["balls"])])

        if hit_points:
            mujoco_item_controls.extend([_get_hit_point(mujoco_item,model_item)
                                         for mujoco_item,model_item
                                         in zip(hit_points,items["hit_points"])])

        if goals:
            mujoco_item_controls.extend([_get_goal(mujoco_item,model_item)
                                         for mujoco_item,model_item
                                         in zip(goals,items["goals"])])

        for mujoco_item_control in mujoco_item_controls:
            config.add_control(mujoco_item_control)


        for key,robot in zip(("robot1","robot2"),(robot1,robot2)):
            if robot:
                r = _get_mujoco_robot_control(robot,items[key])
                if r:
                    config.add_control(r)

        # writing the mujoco config in the shared memory.
        # the mujoco executable is expected to read it and start

        logging.info("sharing mujoco configuration for {}".format(mujoco_id))
        
        pam_mujoco_wrp.set_mujoco_config(config)        

        # waiting for mujoco to report it is ready

        logging.info("waiting for mujoco executable {}".format(mujoco_id))
        
        pam_mujoco_wrp.wait_for_mujoco(mujoco_id)

        # creating o80 frontends
        
        frontends = {}
        interfaces = {}
        
        for item in balls:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info("creating o80 frontend for ball {} /  {}".format(mujoco_id,item.segment_id))
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80Ball(item.segment_id,frontend)
                frontends[item.segment_id]=frontend
                interfaces[item.segment_id]=interface
                
        for item in goals:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info("creating o80 frontend for goal {} /  {}".format(mujoco_id,item.segment_id))
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80Goal(item.segment_id,frontend)
                frontends[item.segment_id]=frontend
                interfaces[item.segment_id]=interface

        for item in hit_points:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info("creating o80 frontend for hit point {} /  {}".format(mujoco_id,item.segment_id))
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80HitPoint(item.segment_id,frontend)
                frontends[item.segment_id]=frontend
                interfaces[item.segment_id]=interface
                
        for robot in robot1,robot2:
            if robot:
                if robot.control==MujocoRobot.JOINT_CONTROL:
                    logging.info("creating o80 frontend for joint control of {} /  {}".format(mujoco_id,robot.segment_id))
                    frontends[robot.segment_id]=o80_pam.JointFrontEnd(robot.segment_id)
                if robot.control==MujocoRobot.PRESSURE_CONTROL:
                    logging.info("creating o80 frontend for pressure control of {} /  {}".format(mujoco_id,robot.segment_id))
                    frontends[robot.segment_id]=o80_pam.FrontEnd(robot.segment_id)



    def reset(self):
        shared_memory.set_bool(self._mujoco_id,"reset",True)

    def pause(self,value):
        shared_memory.set_bool(self._mujoco_id,"pause",value)
                    
            
        
                    
