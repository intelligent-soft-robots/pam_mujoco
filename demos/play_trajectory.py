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
                                 table=True,
                                 hit_point=True)

# getting the items
ball = items["ball"]
table = items["table"]
hit_point = items["hit_point"]

# o80 segment ids
segment_ids = { "ball":"ball",
                "hit_point":"hit_point",
                "contact":"contact" }

# running mujoco thread
def execute_mujoco(segment_ids,
                   mujoco_id,model_name,
                   ball,hit_point,table):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding the mirror ball controller
    pam_mujoco.add_mirror_free_joint(segment_ids["ball"],
                                     ball.joint,
                                     ball.index_qpos,ball.index_qvel)
    # adding the hit point controller
    pam_mujoco.add_mirror_free_joint(segment_ids["hit_point"],
                                     hit_point.joint,
                                     hit_point.index_qpos,hit_point.index_qvel)
    # adding detection of contact between ball and table
    pam_mujoco.add_contact_free_joint(segment_ids["contact"],
                                      segment_ids["contact"]+"_reset",
                                      ball.joint,ball.geom,table.geom_plate)
    # starting the thread
    pam_mujoco.execute(mujoco_id,model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(segment_ids,mujoco_id,model_name,
                                         ball,hit_point,table,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)

# initializing o80 frontend for sending ball/hit_point position/velocity
# to mujoco thread
frontend_ball = pam_mujoco.MirrorFreeJointFrontEnd(segment_ids["ball"])
frontend_hit_point = pam_mujoco.MirrorFreeJointFrontEnd(segment_ids["hit_point"])

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
frontend_ball.pulse()

# monitoring contact ball/table
# and moving hit point to contact location
while True:
    contacts = pam_mujoco.get_contact(segment_ids["contact"])
    if contacts.contact_occured:
        # contact detected: moving the hit point at the contact
        # position
        position = contacts.position
        print("contact:",position)
        for dim,p in enumerate(position):
            # position
            frontend_hit_point.add_command(2*dim,
                                           o80.State1d(p),
                                           duration,
                                           o80.Mode.QUEUE)
            # velocity (0)
            frontend_hit_point.add_command(2*dim+1,
                                           o80.State1d(0),
                                           duration,
                                           o80.Mode.QUEUE)

        frontend_hit_point.pulse()
        break

    
time.sleep(2)
    
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()



