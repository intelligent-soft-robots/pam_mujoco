import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

segment_id_contacts = "contacts"
segment_id_contacts_reset = "contacts_reset"
segment_id_ball = "ball"
segment_id_robot = "robot"
mujoco_id = "mj"
model = "pamy" # i.e pamy.xml in pam_mujoco/models/

# running mujoco thread
def execute_mujoco(segment_id_contacts,
                   segment_id_contacts_reset,
                   segment_id_ball,
                   segment_id_robot,
                   mujoco_id,
                   model):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding contact detection between ball and table
    # note: best to add first (i.e. before mirror robot and
    # mirror ball), as controllers are called in sequence,
    # and contact info will change behavior of mirror ball controller
    pam_mujoco.add_contact_ball(segment_id_contacts,
                                segment_id_contacts_reset,
                                "ball_free_jnt",
                                "ball1",
                                "racket")
    # adding the mirror ball controller
    # (mirroring will be interrupted upon contact with the racket:
    # the contact controller (added right above) will write contact information
    # in the shared memory (segment: segment_id_contacts). The ball mirroring 
    # controller will stop mirroring the ball after the data shared in the memory
    # shows a contact occured).
    pam_mujoco.add_mirror_until_contact_one_ball(segment_id_ball,
                                                 mujoco_id,
                                                 "ball_free_jnt",
                                                 segment_id_contacts)
    # adding robot mirroring controller
    pam_mujoco.add_mirror_robot(segment_id_robot,
                                mujoco_id,
                                "joint_base_rotation")
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
                                         segment_id_robot,
                                         mujoco_id,model,))
process.start()
time.sleep(3)

# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
frontend_ball = pam_mujoco.MirrorOneBallFrontEnd(segment_id_ball)

# initializing o80 frontend for sending robot joints position/velocity
# to mujoco thread
frontend_robot = pam_mujoco.MirrorRobotFrontEnd(segment_id_robot)

# having the ball falling off slowly
ball_x = 0.8
ball_y = 0.1
ball_z_start = 1.0
ball_z_end = -3.0
start_point = [ball_x,ball_y,ball_z_start]
end_point = [ball_x,ball_y,ball_z_end]
duration = 5.0 # seconds
velocity = [0,0,(end_point[2]-start_point[2])/duration]
# "teleportation" to start point
for dim in range(3):
    frontend_ball.add_command(2*dim,
                         o80.State1d(start_point[dim]),
                         o80.Mode.QUEUE)
# falling down to end point
for dim in range(3):
    frontend_ball.add_command(2*dim,
                         o80.State1d(end_point[dim]),
                         o80.Duration_us.seconds(int(duration)),
                         o80.Mode.QUEUE)
    frontend_ball.add_command(2*dim+1,
                         o80.State1d(velocity[dim]),
                         o80.Duration_us.seconds(int(duration)),
                         o80.Mode.QUEUE)


# having the robot going to a reference position
robot_target = [np.pi/4.0,
                0.0,
                np.pi/2.0,
                np.pi/4.0]
robot_target = [o80.State2d(rt,0) for rt in robot_target]
for dof,target in enumerate(robot_target):
    frontend_robot.add_command(dof,target,
                               o80.Duration_us.milliseconds(500),
                               o80.Mode.QUEUE)
    
# sending ball trajectory and robot motion
frontend_ball.pulse()
frontend_robot.pulse()

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

