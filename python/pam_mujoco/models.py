import os
import itertools
import numpy as np
from .mujoco_robot import MujocoRobot
from .mujoco_item import MujocoItem
from . import xml_templates
from . import paths


class HitPoint:
    
    def __init__(self,
                 model_name,
                 name,
                 position=[0,0,0],
                 size=[0.03,0.0007],
                 color=[1.0,0.65,0.1,1]):
        self.model_name=model_name
        self.name = name
        self.position = position
        self.size = size
        self.color = color
        # will be filled by the "generate_model"
        # function (in this file)
        self.geom = None
        self.joint = None
        self.index_qpos = -1
        self.index_qvel = -1
        
    def get_xml(self):
        (xml,name_geom,
         name_joint,nb_bodies) = xml_templates.get_free_joint_body_xml(self.model_name,
                                                                       self.name,
                                                                       "cylinder",
                                                                       self.position,
                                                                       self.size,
                                                                       self.color,
                                                                       0)
        return (xml,name_geom,
                name_joint,nb_bodies)


class Goal(HitPoint):

    def __init__(self,
                 model_name,
                 name,
                 position=[0,0,0],
                 size=[0.05,0.0005],
                 color=[1.0,0.2,0.2,1]):
        HitPoint.__init__(self,
                          model_name,
                          name,
                          position=position,
                          size=size,
                          color=color)
        

class Ball:

    def __init__(self,
                 model_name,
                 name,
                 size=0.02,
                 color=[1.0,0.65,0.1,1.0],
                 position=[0,0,0],
                 mass=0.0027):
        self.model_name = model_name
        self.name = name
        self.size = size
        self.color = color
        self.position = position
        self.mass = mass
        # will be filled by the "generate_model"
        # function (in this file)
        self.geom = None
        self.joint = None
        self.index_qpos = -1
        self.index_qvel = -1

    def get_xml(self):
        (xml,name_geom,
         name_joint,nb_bodies) = xml_templates.get_free_joint_body_xml(self.model_name,
                                                                       self.name,
                                                                       "sphere",
                                                                       self.position,
                                                                       self.size,
                                                                       self.color,
                                                                       self.mass)
        return (xml,name_geom,
                name_joint,nb_bodies)


class Table:

    def __init__(self,
                 model_name,
                 name,
                 position=[0.8,1.7,-0.475],
                 size=[0.7625,1.37,0.02],
                 color=[0.05,0.3,0.23,1.0]):
        self.model_name = model_name
        self.name = name
        self.position = position
        self.size = size
        self.color = color
        # will be filled by the "generate_model"
        # function (in this file)
        self.geom_plate = None
        self.geom_net = None
        
    def get_xml(self):
        (xml,name_plate_geom,
         name_net_geom,nb_bodies) = xml_templates.get_table_xml(self.name,
                                                                self.model_name,
                                                                self.position,
                                                                self.size,
                                                                self.color)
        return (xml,name_plate_geom,
                name_net_geom,nb_bodies)


class Robot:

    def __init__(self,
                 model_name,
                 name,
                 position,
                 xy_axes,
                 muscles):
        self.model_name = model_name
        self.name = name
        self.position = position
        self.xy_axes = xy_axes
        self.muscles=muscles
        # will be filled by the "generate_model"
        # function (in this file)
        self.geom_racket = None
        self.geom_net = None
        self.joint = None
        self.index_qpos = -1
        self.index_qvel = -1
        
    def get_xml(self):
        (xml,joint,geom_racket,
         geom_racket_handle,
         nb_bodies)= xml_templates.get_robot_xml(self.model_name,
                                                 self.name,
                                                 self.position,
                                                 self.xy_axes,
                                                 self.muscles)
        return (xml,joint,geom_racket,geom_racket_handle,nb_bodies)

    
def defaults_solrefs():
    
    return { "ball":{ "racket":(-0.1,-0.1),
                      "floor":(0.003,0.25),
                      "racket_handle":(0.003,0.35),
                      "table":(-0.1,-0.1),
                      "net":(0.003,10.0) },
             "floor":{ "racket":(0.003,0.9) } },


def defaults_gaps():

    return {"ball":{"floor":0.0,"table":0.02}}


def generate_model(model_name,
                   time_step=0.002,
                   robots=[],
                   balls=[],
                   tables=[],
                   goals=[],
                   hit_points=[],
                   solrefs = defaults_solrefs(),
                   gaps = defaults_gaps(),
                   muscles=False):

    template = paths.get_main_template_xml()
    template = template.replace("$timestep$",str(time_step))
    template = template.replace("$models_path$",paths.get_models_path())
    
    bodies = []
    index_qpos = 0
    index_qvel = 0

    # ball: instance of Ball (in this file)
    for ball in balls:
        xml,geom,joint,nb_bodies = ball.get_xml()
        bodies.append(xml)
        ball.index_qpos = index_qpos
        ball.index_qvel = index_qvel
        ball.geom = geom
        ball.joint = joint
        index_qpos+=7
        index_qvel+=6

    # hit_point, instance of HitPoint
    for hit_point in hit_points:
        xml,geom,joint,nb_bodies = hit_point.get_xml()
        bodies.append(xml)
        hit_point.index_qpos = index_qpos
        hit_point.index_qvel = index_qvel
        hit_point.geom = geom
        hit_point.joint = joint
        index_qpos+=nb_bodies*7
        index_qvel+=nb_bodies*6

    # goal, instance of Goal
    for goal in goals:
        xml,geom,joint,nb_bodies = goal.get_xml()
        bodies.append(xml)
        goal.index_qpos = index_qpos
        goal.index_qvel = index_qvel
        goal.geom = geom
        goal.joint = joint
        index_qpos+=nb_bodies*7
        index_qvel+=nb_bodies*6
        
    # ...
    for table in tables:
        (xml,name_plate_geom,
         name_net_geom,nb_bodies) = table.get_xml()
        bodies.append(xml)
        index_qpos+=nb_bodies*7
        index_qvel+=nb_bodies*6
        table.geom_plate = name_plate_geom
        table.geom_net = name_net_geom

    # ...
    for robot in robots:
        (xml,joint,geom_racket,
         geom_racket_handle,nb_bodies) = robot.get_xml()
        bodies.append(xml)
        robot.geom_racket = geom_racket
        robot.geom_racket_handle = geom_racket_handle
        robot.index_qpos = index_qpos
        robot.index_qvel = index_qvel
        robot.joint = joint
        index_qpos+=nb_bodies*7
        index_qvel+=nb_bodies*6
        
    template = template.replace("<!-- bodies -->","\n".join(bodies))

    if any([r.muscles for r in robots]):
        actuations = ["<tendon>"]
        for robot in robots:
            xml_tendon = paths.get_robot_tendon_xml(robot.name)
            actuations.append(xml_tendon)
        actuations.append("</tendon>")
        actuations.append("<actuator>")
        for robot in robots:
            xml_actuator = paths.get_robot_actuator_xml(robot.name)
            actuations.append(xml_actuator)
        actuations.append("</actuator>")
        template = template.replace("<!-- actuations -->","\n".join(actuations))

    contacts = xml_templates.get_contacts_xml(robots,balls,tables,solrefs,gaps)

    template = template.replace("<!-- contacts -->",contacts)

    path = paths.write_model_xml(model_name,template)

    print("created mujoco xml model file:",path)
    
    return path
        
        

def model_factory(model_name: str,
                  time_step: float = 0.002,
                  table: bool =False,
                  balls: list=[],
                  goals: list=[],
                  hit_points: list=[],
                  robot1: MujocoRobot=None,
                  robot2: MujocoRobot=None):

    r = {}
    
    tables = []
    if table :
        table = Table(model_name,"table")
        tables.append(table)
        r["table"]=table
    else:
        r["table"]=None
        
    xml_balls = [Ball(model_name,
                      ball.segment_id,
                      color=ball.color)
                 for ball in balls]

    robots = []
    for key,robot in zip(("robot1","robot2"),(robot1,robot2)):
        if robot:
            muscles=(robot.control==MujocoRobot.PRESSURE_CONTROL)
            instance=Robot(model_name,
                           robot.segment_id,
                           robot.position,
                           robot.orientation,
                           muscles)
            robots.append(instance)
            r[key]=instance
        else:
            r[key]=None
            
    xml_goals = [Goal(model_name,
                      goal.segment_id,
                      color=goal.color)
                 for goal in goals]

    xml_hit_points = [HitPoint(model_name,
                               hit_point.segment_id,
                               color=hit_point.color)
                      for hit_point in hit_points]


    model_path = generate_model(model_name,
                                time_step = time_step,
                                robots=robots,
                                balls=xml_balls,
                                tables=tables,
                                goals=xml_goals,
                                hit_points=xml_hit_points)

    r["balls"] = xml_balls
    r["goals"] = xml_goals
    r["hit_points"] = xml_hit_points
    r["path"] = model_path

    return r
