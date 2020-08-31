import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

segment_id_contacts = "contacts"
segment_id_contacts_reset = "contacts_reset"
segment_id_ball = "ball"
mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/

# running mujoco thread
def execute_mujoco(segment_id_contacts,
                   segment_id_contacts_reset,
                   segment_id_ball,
                   mujoco_id,
                   model):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding contact detection between ball and table
    pam_mujoco.add_contact_ball(segment_id_contacts,
                                segment_id_contacts_reset,
                                "ball_free_jnt",
                                "ball1",
                                "plate")

    # adding the mirror ball controller
    pam_mujoco.add_mirror_one_ball_robot(segment_id_ball,
                                         mujoco_id,
                                         "ball_free_jnt")
    # starting the thread
    pam_mujoco.execute(mujoco_id,model)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(segment_id_contacts,
                                         segment_id_contacts_reset,
                                         segment_id_ball,
                                         mujoco_id,model,))
process.start()
time.sleep(1)

# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
frontend = pam_mujoco.MirrorOneBallFrontEnd(segment_id_ball)

# reading a random pre-recorded ball trajectory
trajectory_points = list(context.BallTrajectories().random_trajectory())

# having the ball falling off slowly
start_point = [1,1,1]
end_point = [1,1,-3]
duration = 5.0 # seconds
velocity = [0,0,(end_point[2]-start_point[2])/duration]
# "teleportation" to start point
for dim in range(3):
    frontend.add_command(2*dim,
                         o80.State1d(start_point[dim]),
                         o80.Mode.QUEUE)
# falling down to end point
for dim in range(3):
    frontend.add_command(2*dim,
                         o80.State1d(end_point[dim]),
                         o80.Duration_us.seconds(int(duration)),
                         o80.Mode.QUEUE)
    frontend.add_command(2*dim+1,
                         o80.State1d(velocity[dim]),
                         o80.Duration_us.seconds(int(duration)),
                         o80.Mode.QUEUE)

# sending ball trajectory
frontend.pulse();

# bug ? during first iterations, contact are detected ...
time.sleep(1)

# reading shared memory for contact informations
time_start = time.time()
first_contact_detected = False
while time.time()-time_start < duration:
    contacts = pam_mujoco.get_contact(segment_id_contacts)
    if contacts.contact_occured and not first_contact_detected:
        print("contact !")
        first_contact_detected=True;
    time.sleep(0.1)
    
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()

