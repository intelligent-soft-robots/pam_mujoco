import signal_handler
import pam_mujoco
import time
import numpy as np


model_name = "pam_mujoco_model_demo"
mujoco_id = "pam_mujoco_model_demo"

# adding two robots
robots = []
robots.append(pam_mujoco.Robot(model_name,"robot1",[0,0,-0.5],None))
robots.append(pam_mujoco.Robot(model_name,"robot2",[1.6,3.4,-0.5],[-1,0,0,0,-1,0]))

# adding a "grid" of balls
grid = np.mgrid[-1:1.2:0.2, -1:1.2:0.2].reshape(2,-1).T
balls = []
for index,position in enumerate(grid):
    position = list(position)+[1] # adding z
    name = "ball"+str(index)
    color = [abs(position[0]),0,abs(position[1]),1]
    mass = 0.0027
    size = 0.02
    balls.append(pam_mujoco.Ball(model_name,name,color=color,position=position,
                                 size=size,mass=mass))

# adding a table (defaults params except position)
tables = [pam_mujoco.Table(model_name,"table",position=[0.8,1.7,-0.475])]

# adding 2 goals
goals = [pam_mujoco.Goal(model_name,"goal1",position=[0.6,1.4,-0.45]),
         pam_mujoco.Goal(model_name,"goal2",position=[0.4,2.0,-0.45],
                         size=[0.05,0.0005],
                         color=[1.0,0.2,0.2,1.0])]

# adding a "hit point"
hit_points = [pam_mujoco.HitPoint(model_name,"hp",position=[0.7,1.0,-0.45])]

# generating the model xml file (/tmp/test/test.xml)
path = pam_mujoco.generate_model(model_name,
                                 robots=robots,
                                 balls=balls,
                                 tables=tables,
                                 goals=goals,
                                 hit_points=hit_points)

# starting mujoco using the generated model
config = pam_mujoco.MujocoConfig()
config.graphics = True
config.extended_graphics = False
config.realtime = True
pam_mujoco.init_mujoco(config)
pam_mujoco.execute(mujoco_id,path)

signal_handler.init()
print("ctrl+c for exit")
while not signal_handler.has_received_sigint():
    time.sleep(0.01)

pam_mujoco.request_stop(mujoco_id)
