import time
import context
import o80
import o80_pam
import pam_mujoco
import numpy as np
import multiprocessing

mujoco_id = "mj"
nb_balls = 5
segment_ids = ["play_trajectories_" + str(index) for index in range(nb_balls)]

# generates model in pam_mujoco/models/tmp/trajectories.xml
model_name = "trajectories"
ball_colors = [
    [float(index) / float(nb_balls), 0.0, 1.0 - float(index) / float(nb_balls), 1.0]
    for index in range(nb_balls)
]
items = pam_mujoco.model_factory(
    model_name, table=True, nb_balls=nb_balls, ball_colors=ball_colors
)

# getting the balls
balls = items["balls"]

# running mujoco thread
def execute_mujoco(segment_ids, mujoco_id, model_name, balls):
    # init mujoco
    config = pam_mujoco.MujocoConfig()
    config.graphics = True
    config.extended_graphics = False
    config.realtime = False
    pam_mujoco.init_mujoco(config)
    # adding the mirror ball controllers
    for segment_id, ball in zip(segment_ids, balls):
        pam_mujoco.add_mirror_free_joint(
            segment_id, ball.joint, ball.index_qpos, ball.index_qvel
        )
    # staring the thread
    pam_mujoco.execute(mujoco_id, model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# clearing shared memory from previous run
pam_mujoco.clear(mujoco_id)

# starting mujoco thread
process = multiprocessing.Process(
    target=execute_mujoco,
    args=(
        segment_ids,
        mujoco_id,
        model_name,
        balls,
    ),
)
process.start()

# waiting for mujoco to startup
pam_mujoco.wait_for_mujoco(mujoco_id)


# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
# reading ball trajectories from files
frontends = []
trajectories = []
ball_trajectories = context.BallTrajectories()
for segment_id in segment_ids:
    frontends.append(o80_pam.MirrorFreeJointFrontEnd(segment_id))
    trajectories.append(ball_trajectories.random_trajectory()[1])

# preparing the full ball trajectories
# duration of 10ms : sampling rate of the trajectory
duration = o80.Duration_us.milliseconds(10)
for _ in range(3):
    for segment_id, frontend, trajectory_points in zip(
        segment_ids, frontends, trajectories
    ):
        for traj_point in trajectory_points:
            # looping over x,y,z
            for dim in range(3):
                # setting position for dimension (x, y or z)
                frontend.add_command(
                    2 * dim,
                    o80.State1d(traj_point.position[dim]),
                    duration,
                    o80.Mode.QUEUE,
                )
                # setting velocity for dimension (x, y or z)
                frontend.add_command(
                    2 * dim + 1,
                    o80.State1d(traj_point.velocity[dim]),
                    duration,
                    o80.Mode.QUEUE,
                )


# sending the full ball trajectories to mujoco
for frontend in frontends:
    frontend.pulse_prepare_wait()

# waiting for all trajectories to finish
for frontend in frontends:
    frontend.wait()

# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()
