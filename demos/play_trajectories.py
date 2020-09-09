import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

mujoco_id = "mj"
nb_balls = 20
segment_ids = ["play_trajectories_"+str(index)
               for index in range(nb_balls)]

# generates model in pam_mujoco/models/tmp/trajectories.xml
model_name = "trajectories"
items = pam_mujoco.model_factory(model_name,table=True,nb_balls=nb_balls)
balls = items[0]

# running mujoco thread
def execute_mujoco(segment_ids,mujoco_id,model_name,balls):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding the mirror ball controllers
    for segment_id,ball in zip(segment_ids,balls):
        pam_mujoco.add_mirror_free_joint(segment_id,
                                         ball.joint,
                                         ball.index_qpos,
                                         ball.index_qvel)
    # staring the thread
    pam_mujoco.execute(mujoco_id,model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(segment_ids,mujoco_id,model_name,balls,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)


# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
frontends = []
trajectories = []
ball_trajectories = context.BallTrajectories()
for segment_id in segment_ids:
    frontends.append(pam_mujoco.MirrorOneBallFrontEnd(segment_id))
    trajectories.append(list(ball_trajectories.random_trajectory()))

# sending the full ball trajectories to the mujoco thread.
# duration of 10ms : sampling rate of the trajectory
duration = o80.Duration_us.milliseconds(10)
for _ in range(1):
    for segment_id,frontend,trajectory_points in zip(segment_ids,frontends,trajectories):
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
        
for frontend in frontends:
    frontend.pulse()

time.sleep(6)
    
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()




