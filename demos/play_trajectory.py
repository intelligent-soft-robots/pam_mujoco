import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

segment_id = "play_trajectory"
mujoco_id = "mj"

# generates pamy.xml in pam_mujoco/models/tmp/
model_name = "trajectory"
items = pam_mujoco.model_factory(model_name,
                                 table=True)

# getting the items
ball = items["ball"]
table = items["table"]

# o80 segment ids
segment_ids = { "ball":"ball",
                "contact":"contact" }

# running mujoco thread
def execute_mujoco(segment_ids,
                   mujoco_id,model_name,
                   ball,table):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding the mirror ball controller
    pam_mujoco.add_mirror_free_joint(segment_ids["ball"],
                                     ball.joint,
                                     ball.index_qpos,ball.index_qvel)
    # starting the thread
    pam_mujoco.execute(mujoco_id,model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(segment_ids,mujoco_id,model_name,
                                         ball,table,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)

# initializing o80 frontend for sending ball/hit_point position/velocity
# to mujoco thread
frontend_ball = pam_mujoco.MirrorFreeJointFrontEnd(segment_ids["ball"])

# reading a random pre-recorded ball trajectory
trajectory_points = list(context.BallTrajectories().random_trajectory())

# sending the full ball trajectory to the mujoco thread.
# duration of 10ms : sampling rate of the trajectory
duration = o80.Duration_us.milliseconds(10)
for traj_point in trajectory_points:
    # looping over x,y,z
    for dim in range(3):
        # setting position for dimension (x, y or z)
        frontend_ball.add_command(2*dim,
                             o80.State1d(traj_point.position[dim]),
                             duration,
                             o80.Mode.QUEUE)
        # setting velocity for dimension (x, y or z)
        frontend_ball.add_command(2*dim+1,
                             o80.State1d(traj_point.velocity[dim]),
                             duration,
                             o80.Mode.QUEUE)

# sending for full trajectory
frontend_ball.pulse_and_wait()
    
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()



