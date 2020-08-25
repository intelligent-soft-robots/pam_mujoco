import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing


mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/

# running mujoco process
# (note: could be started by another executable) 
def execute_mujoco(mujoco_id,model):
    # adding controller for mirroring ball
    pam_mujoco.add_mirror_one_ball(mujoco_id)
    # starting mujoco's process
    pam_mujoco.execute(mujoco_id,model)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)

        
# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model,))
process.start()
time.sleep(1)

# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
segment_id = pam_mujoco.get_mirror_one_ball_segment_id(mujoco_id)
frontend = pam_mujoco.MirrorOneBallFrontEnd(segment_id)

# reading a pre-recorded ball trajectory and 
# sending it to the mujoco process.
# duration of 10ms : sampling rate of the trajectory
duration = o80.Duration_us.milliseconds(10)
for traj_point in context.BallTrajectories().random_trajectory():
    # looping over x,y,z
    for dim in range(3):
        # setting position for dimension
        frontend.add_command(2*dim,
                             o80.State1d(traj_point.position[dim]),
                             duration,
                             o80.Mode.QUEUE)
        # setting velocity for dimension
        frontend.add_command(2*dim+1,
                             o80.State1d(traj_point.velocity[dim]),
                             duration,
                             o80.Mode.QUEUE)

# sending for full trajectory and wait for it to be executed
frontend.pulse_and_wait()

# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()




