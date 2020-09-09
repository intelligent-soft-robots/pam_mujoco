import pam_mujoco
import time
import numpy as np


model = "test"
mujoco_id = "test"

# adding two robots
robots = []
robots.append(pam_mujoco.Robot("robot1",[0,0,-0.5],None))
robots.append(pam_mujoco.Robot("robot2",[1.6,3.4,-0.5],[-1,0,0,0,-1,0]))

# adding a "grid" of balls
grid = np.mgrid[-1:1.2:0.2, -1:1.2:0.2].reshape(2,-1).T
balls = []
for index,position in enumerate(grid):
    position = list(position)+[1] # adding z
    name = "ball"+str(index)
    color = [abs(position[0]),0,abs(position[1]),1]
    mass = 0.0027
    size = 0.02
    balls.append(pam_mujoco.Ball(name,color=color,position=position,
                             size=size,mass=mass))

# adding a table (defaults params except position)
tables = [pam_mujoco.Table("table",position=[0.8,1.7,-0.475])]

# adding 2 goals
goals = [pam_mujoco.Goal("goal1",position=[0.6,1.4,-0.45]),
         pam_mujoco.Goal("goal2",position=[0.4,2.0,-0.45],
                         radius1=0.1, radius2=0.2,
                         color1=[1.0,0.2,0.2,1.0],color2=[1.0,0.2,0.2,0.05])]

# adding a "hit point"
hitting_points = [pam_mujoco.HittingPoint("hp",position=[0.7,1.0,-0.45])]

# generating the model xml file (in pam_mujoco/modes/tmp/test.xml)
pam_mujoco.generate_model(model,
                          robots=robots,
                          balls=balls,
                          tables=tables,
                          goals=goals,
                          hitting_points=hitting_points)

# starting mujoco using the generated model
pam_mujoco.init_mujoco()
pam_mujoco.execute(mujoco_id,model)

while True:
    time.sleep(0.01)

pam_mujoco.request_stop(mujoco_id)
