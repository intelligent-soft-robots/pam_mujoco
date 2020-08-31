import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

segment_id = "play_trajectory"
mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/

# running mujoco thread
def execute_mujoco(segment_id,mujoco_id,model):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding the mirror ball controller
    pam_mujoco.add_mirror_one_ball_robot(segment_id,mujoco_id,
                                         "ball_free_jnt")
    # staring the thread
    pam_mujoco.execute(mujoco_id,model)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(segment_id,mujoco_id,model,))
process.start()
time.sleep(1)

# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
frontend = pam_mujoco.MirrorOneBallFrontEnd(segment_id)

# reading a random pre-recorded ball trajectory
trajectory_points = list(context.BallTrajectories().random_trajectory())

# sending the full ball trajectory to the mujoco thread.
# duration of 10ms : sampling rate of the trajectory
duration = o80.Duration_us.milliseconds(10)
for traj_point in trajectory_points:
    # looping over x,y,z
    for dim in range(3):
        # setting position for dimension (x, y or z)
        frontend.add_command(2*dim,
                             o80.State1d(traj_point.position[dim]),
                             duration,
                             o80.Mode.QUEUE)
        # setting velocity for dimension (x, y or z)
        frontend.add_command(2*dim+1,
                             o80.State1d(traj_point.velocity[dim]),
                             duration,
                             o80.Mode.QUEUE)

# sending for full trajectory and wait for it to be executed
frontend.pulse_and_wait()

# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()



