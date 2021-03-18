from pam_mujoco_wrp import *
from .models import Ball,Table,Goal,Robot,HitPoint,generate_model,model_factory
from . import start_mujoco
from .segment_ids import segment_ids
from .mujoco_ids import mujoco_ids


# convenience aliases

def add_o80_ball_control(ball_id,ball):
    add_mirror_free_joint(ball_id,
                          ball.joint,
                          ball.index_qpos,ball.index_qvel)

def add_o80_goal_control(goal_id,goal):
    add_mirror_free_joint(goal_id,
                          goal.joint,
                          goal.index_qpos,goal.index_qvel)

def add_o80_hit_point_control(hit_point_id,hit_point):
    add_mirror_free_joint(hit_point_id,
                          hit_point.joint,
                          hit_point.index_qpos,hit_point.index_qvel)

def add_table_contact(contact_id,item,table):
    add_table_contact_free_joint(contact_id,
                                 item.index_qpos,item.index_qvel,
                                 item.geom,table.geom_plate)

def add_robot_contact(contact_id,item,robot):
    add_robot1_contact_free_joint(contact_id,
                                  item.index_qpos,item.index_qvel,
                                  item.geom,robot.geom_racket)

def add_o80_ball_control_until_contact(ball_id,contact_id,ball):
    add_mirror_until_contact_free_joint(ball_id,
                                        ball.joint,
                                        ball.index_qpos,ball.index_qvel,
                                        contact_id)

def add_o80_joint_control(joint_control_id,robot):
    add_mirror_robot(joint_control_id,robot.joint)
