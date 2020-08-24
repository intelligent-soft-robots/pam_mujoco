import time
import o80
import pam_mujoco
import numpy as np
import multiprocessing

def execute_mujoco(mujoco_id,model_path):

    controllers = set([1])

    pam_mujoco.execute(mujoco_id,model_path,controllers,1)

    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)

mujoco_id = "mj"
model_path = "/home/vberenz/Workspaces/pam/workspace/src/pam_mujoco/models/pamy.xml"

        
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model_path,))
process.start()

time.sleep(3)

segment_id = pam_mujoco.get_mirror_external_robot_segment_id(mujoco_id)
frontend = pam_mujoco.MirrorRobotFrontEnd(segment_id)

nb_iterations = 3000
iterations = o80.Iteration(nb_iterations,True,True)

state = o80.State2d(np.pi/4.0,0)
for dof in range(4):
    frontend.add_command(dof,state,iterations,o80.Mode.OVERWRITE)
frontend.burst(nb_iterations)

for _ in range(4):

    state = o80.State2d(-np.pi/4.0,0)
    for dof in range(4):
        frontend.add_command(dof,state,iterations,o80.Mode.OVERWRITE)
    frontend.burst(nb_iterations)

    state = o80.State2d(+np.pi/4.0,0)
    for dof in range(4):
        frontend.add_command(dof,state,iterations,o80.Mode.OVERWRITE)
    frontend.burst(nb_iterations)

state = o80.State2d(0,0)
for dof in range(4):
    frontend.add_command(dof,state,iterations,o80.Mode.OVERWRITE)
frontend.burst(nb_iterations)

pam_mujoco.request_stop("mj")
process.join()


