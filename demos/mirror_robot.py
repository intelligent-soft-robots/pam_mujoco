import time
import o80
import o80_pam
import pam_mujoco
import numpy as np
import multiprocessing

segment_id = "mirror_robot"
mujoco_id = "mj"

# generates pamy.xml in pam_mujoco/models/tmp/
model_name = "pamy"
items = pam_mujoco.model_factory(model_name, table=True, robot1=True)
robot = items["robot"]


# running the mujoco thread
def execute_mujoco(segment_id, mujoco_id, model_name, robot):
    # init mujoco
    config = pam_mujoco.MujocoConfig()
    config.graphics = True
    config.extended_graphics = False
    config.realtime = True
    pam_mujoco.init_mujoco(config)
    # adding mirroring robot controller
    pam_mujoco.add_mirror_robot(segment_id, robot.joint)
    # setting bursting mode
    pam_mujoco.add_bursting(mujoco_id, segment_id)
    # starting the mujoco thread (reads pamy.xml in pam_mujoco/models/tmp/)
    pam_mujoco.execute(mujoco_id, model_name)
    # runnign it until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting the mujoco thread
process = multiprocessing.Process(
    target=execute_mujoco,
    args=(
        segment_id,
        mujoco_id,
        model_name,
        robot,
    ),
)
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)


# initializing the o80 frontend for sending
# robot posture commands
frontend = o80_pam.MirrorRobotFrontEnd(segment_id)

# getting to init posture (all joints to pi/4)
# in 3000 iteration
nb_iterations = 500
iterations = o80.Iteration(nb_iterations, True, True)
state = o80.State2d(np.pi / 4.0, 0)
# adding a command for each joint
for dof in range(4):
    frontend.add_command(dof, state, iterations, o80.Mode.OVERWRITE)
# requesting mujoco to run 3000 iteration
frontend.burst(nb_iterations)

# moving 4 times the joints from -pi/4 to pi/4 and back
for _ in range(4):
    state = o80.State2d(-np.pi / 4.0, 0)
    for dof in range(4):
        frontend.add_command(dof, state, iterations, o80.Mode.OVERWRITE)
    frontend.burst(nb_iterations)
    state = o80.State2d(+np.pi / 4.0, 0)
    for dof in range(4):
        frontend.add_command(dof, state, iterations, o80.Mode.OVERWRITE)
    frontend.burst(nb_iterations)

# back to all joints at 0
state = o80.State2d(0, 0)
for dof in range(4):
    frontend.add_command(dof, state, iterations, o80.Mode.OVERWRITE)
frontend.burst(nb_iterations)

# ending the mujoco thread
pam_mujoco.request_stop("mj")
frontend.final_burst()
process.join()
