import time
import context
import o80_pam
import o80
import pam_mujoco
import numpy as np
import multiprocessing

mujoco_id = "mj"

# generates pamy.xml in pam_mujoco/models/tmp/
model_name = "trajectory"
nb_balls = 2 # one playing the trajectory fully, the other until contact with table
ball_colors = [[1,0,0,1],[0,0,1,1]]
items = pam_mujoco.model_factory(model_name,
                                 table=True,nb_balls=nb_balls,
                                 ball_colors=ball_colors)

# getting the items
balls = items["balls"]
table = items["table"]

# running mujoco thread
def execute_mujoco(mujoco_id,model_name,
                   balls,table):
    # init mujoco
    pam_mujoco.init_mujoco()
    #for detecting contact with the table
    pam_mujoco.add_table_contact_free_joint("table",
                                            balls[1].index_qpos,balls[1].index_qvel,
                                            balls[1].geom,table.geom_plate)
    # adding the mirror ball controller, full trajectory
    pam_mujoco.add_mirror_free_joint("ball1",
                                     balls[0].joint,
                                     balls[0].index_qpos,balls[0].index_qvel)
    # adding the mirror ball controller, until contact with table
    pam_mujoco.add_mirror_until_contact_free_joint("ball2",
                                                   balls[1].joint,
                                                   balls[1].index_qpos,balls[1].index_qvel,
                                                   "table")
    # starting the thread
    pam_mujoco.execute(mujoco_id,model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model_name,
                                         balls,table,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)



for run in range(10):

    # initializing o80 frontend for sending ball/hit_point position/velocity
    # to mujoco thread
    frontend_ball1 = o80_pam.MirrorFreeJointFrontEnd("ball1")
    frontend_ball2 = o80_pam.MirrorFreeJointFrontEnd("ball2")
    frontends = [frontend_ball1,frontend_ball2]
    
    # reading a random pre-recorded ball trajectory
    trajectory_points = list(context.BallTrajectories().random_trajectory())

    # lowering the balls a bit to ensure contact with table
    translation = [0,0,-0.01]
    # sending the full ball trajectory to the mujoco thread.
    # duration of 10ms : sampling rate of the trajectory
    duration_s = 0.01
    duration = o80.Duration_us.milliseconds(int(duration_s*1000))
    total_duration = duration_s * len(trajectory_points)
    for traj_point in trajectory_points:
        # looping over x,y,z
        for dim in range(3):
            for frontend in frontends:
                # setting position for dimension (x, y or z)
                frontend.add_command(2*dim,
                                     o80.State1d(traj_point.position[dim]+translation[dim]),
                                     duration,
                                     o80.Mode.QUEUE)
                # setting velocity for dimension (x, y or z)
                frontend.add_command(2*dim+1,
                                     o80.State1d(traj_point.velocity[dim]),
                                     duration,
                                     o80.Mode.QUEUE)

    # sending for full trajectory
    for frontend in frontends:
        frontend.pulse()

    # sanity check: ball2 touches the table
    contact_detected=False
    time_start=time.time()
    while time.time()-time_start < total_duration:
        contacts = pam_mujoco.get_contact("table")
        if contacts.contact_occured and not contact_detected:
            print("contact !")
            contact_detected=True
        time.sleep(0.05)

    # resetting contacts
    pam_mujoco.reset_contact("table")
        
# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()



