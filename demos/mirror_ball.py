import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

def execute_mujoco(mujoco_id,model):

    controllers = set([pam_mujoco.ControllerTypes.MIRROR_ONE_BALL])
    bursting_segment_id = pam_mujoco.get_mirror_one_ball_segment_id(mujoco_id)
    pam_mujoco.execute(mujoco_id,model,controllers,
                       bursting_segment_id)
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)

mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/
        
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model,))
process.start()

time.sleep(1)

segment_id = pam_mujoco.get_mirror_one_ball_segment_id(mujoco_id)
frontend = pam_mujoco.MirrorOneBallFrontEnd(segment_id)

trajectory_points = list(context.BallTrajectories().random_trajectory())

nb_iterations = 50
for index,traj_point in enumerate(trajectory_points):
    iteration = o80.Iteration((index+1)*nb_iterations)
    for dim in range(3):
        frontend.add_command(2*dim,
                             o80.State1d(traj_point.position[dim]),
                             iteration,
                             o80.Mode.QUEUE)
        frontend.add_command(2*dim+1,
                             o80.State1d(traj_point.velocity[dim]),
                             iteration,
                             o80.Mode.QUEUE)

frontend.burst(len(trajectory_points)*nb_iterations)

pam_mujoco.request_stop("mj")

frontend.final_burst()

process.join()




