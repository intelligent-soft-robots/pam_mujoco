import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing


mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/


# running mujoco thread
def execute_mujoco(mujoco_id,model):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding the mirror ball controller
    pam_mujoco.add_mirror_one_ball_robot(mujoco_id)
    # starting the thread
    pam_mujoco.execute(mujoco_id,model)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model,))
process.start()

time.sleep(4)

pam_mujoco.request_stop("mj")
process.join()


