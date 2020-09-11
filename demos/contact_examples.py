import time
import context
import o80
import pam_mujoco
import numpy as np
import multiprocessing

mujoco_id = "mj"
model_name = "contacts"
items = pam_mujoco.model_factory("contacts",table=True,robot1=True)
ball = items["ball"]
table = items["table"]
robot = items["robot"]

# running mujoco thread
def execute_mujoco(ball,table,robot,mujoco_id,model_name):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding contact detection between ball and table
    # note: best to add first (i.e. before mirror robot and
    # mirror ball), as controllers are called in sequence,
    # and contact info will change behavior of mirror ball controller
    pam_mujoco.add_robot1_contact_free_joint("racket",
                                             "racket_reset",
                                             ball.index_qpos,ball.index_qvel,
                                             ball.geom,robot.geom_racket)
    # for detecting contact with the table
    pam_mujoco.add_table_contact_free_joint("table",
                                      "table_reset",
                                      ball.index_qpos,ball.index_qvel,
                                      ball.geom,table.geom_plate)
    # adding the mirror ball controller
    # (mirroring will be interrupted upon contact with the racket:
    # the contact controller (added right above) will write contact information
    # in the shared memory (segment: segment_id_contacts). The ball mirroring 
    # controller will stop mirroring the ball after the data shared in the memory
    # shows a contact occured).
    pam_mujoco.add_mirror_until_contact_free_joint("ball",
                                                   ball.joint,
                                                   ball.index_qpos,ball.index_qvel,
                                                   "racket")
    # adding robot mirroring controller
    pam_mujoco.add_mirror_robot("robot",
                                robot.joint)
    # starting the thread
    pam_mujoco.execute(mujoco_id,model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(ball,table,robot,
                                         mujoco_id,model_name,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)

# initializing o80 frontend for sending ball position/velocity
# to mujoco thread
frontend_ball = pam_mujoco.MirrorFreeJointFrontEnd("ball")

# initializing o80 frontend for sending robot joints position/velocity
# to mujoco thread
frontend_robot = pam_mujoco.MirrorRobotFrontEnd("robot")

# having the ball falling off slowly
ball_x = 0.94
ball_y = 0.45
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
                np.pi/3.5,
                np.pi/3.0,
                np.pi/4.0]
robot_target = [o80.State2d(rt,0) for rt in robot_target]
for dof,target in enumerate(robot_target):
    frontend_robot.add_command(dof,target,
                               o80.Duration_us.milliseconds(500),
                               o80.Mode.QUEUE)
    
# sending ball trajectory and robot motion
frontend_ball.pulse_prepare_wait()
frontend_robot.pulse_prepare_wait()

frontend_ball.wait()
frontend_robot.wait()

# bug ? during first iterations, contact are detected ...
time.sleep(1)

def commented():

    # reading shared memory for contact informations
    time_start = time.time()
    first_contact_racket = False
    first_contact_table = False
    while time.time()-time_start < duration:
        contacts_racket = pam_mujoco.get_contact("racket")
        if contacts_racket.contact_occured and not first_contact_racket:
            print("-- contact with racket")
            first_contact_racket=True;
        contacts_table = pam_mujoco.get_contact("table")
        if contacts_table.contact_occured and not first_contact_table:
            print("-- contact with table")
            print("\tposition:",",".join([str(a) for a in contacts_table.position]))
            first_contact_table=True;
        time.sleep(0.1)
    
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()

