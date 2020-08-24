import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

def execute_mujoco(mujoco_id,model):

    controllers = set([pam_mujoco.ControllerTypes.MIRROR_ONE_BALL])
    bursting_segment_id = "" # i.e. not bursting mode
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

duration = o80.Duration_us.milliseconds(10)
for traj_point in trajectory_points:
    for dim in range(3):
        frontend.add_command(2*dim,
                             o80.State1d(traj_point.position[dim]),
                             duration,
                             o80.Mode.QUEUE)
        frontend.add_command(2*dim+1,
                             o80.State1d(traj_point.velocity[dim]),
                             duration,
                             o80.Mode.QUEUE)

frontend.pulse_and_wait()

pam_mujoco.request_stop("mj")

process.join()




