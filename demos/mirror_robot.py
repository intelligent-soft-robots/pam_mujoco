import time
import o80
import pam_mujoco
import numpy as np

frontend = pam_mujoco.FrontEnd("pam_mujoco")

position = -np.pi/4.0
velocity = None

while position < np.pi/4.0 :

    if velocity is None:
        velocity = 0
    else :
        velocity = 0.1
    
    state = o80.State2d(position,velocity)
    for dof in range(4):
        frontend.add_command(dof,state,o80.Mode.OVERWRITE)
    frontend.pulse()

    time.sleep(0.01)

    position+= 0.01


state.set(0,np.pi/4.0)
state.set(1,0)
for dof in range(4):
    frontend.add_command(dof,state,o80.Mode.OVERWRITE)
frontend.pulse()

velocity = None

while position > -np.pi/4.0 :

    if velocity is None:
        velocity = 0
    else :
        velocity = -0.1
    
    state = o80.State2d(position,velocity)
    for dof in range(4):
        frontend.add_command(dof,state,o80.Mode.OVERWRITE)
    frontend.pulse()

    time.sleep(0.01)

    position-= 0.01


state.set(0,-np.pi/4.0)
state.set(1,0)
for dof in range(4):
    frontend.add_command(dof,state,o80.Mode.OVERWRITE)
frontend.pulse()






