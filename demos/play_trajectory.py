import math
import time
import context
import o80_pam
import o80
import pam_mujoco
import numpy as np
import multiprocessing
import matplotlib
import matplotlib.pyplot as plt

# In this demo, we play a recorded ball trajectory.
# (recorded from a real ball bouncing on a real table).
# We play in two manners:
# - One ball plays the full trajectory
# - The other ball plays the trajectory up to contact with the
#   table, and from there mujoco physics takes over, based on our
#   customized bounce model
# 3 trajectories are played. For all 3 trajectories, the distance
# between the 2 balls is plotted. This allow us to get an intuition
# how good (or bad) our bounce model is.


mujoco_id = "mj"

# generates the mujoco xml model
model_name = "trajectory"
nb_balls = 2 # one playing the trajectory fully, the other until contact with table
ball_colors = [[1,0,0,1],[0,0,1,1]]
# we request the model to have 2 balls, and a table
items = pam_mujoco.model_factory(model_name,
                                 table=True,nb_balls=nb_balls,
                                 ball_colors=ball_colors)

# getting the items
balls = items["balls"]
table = items["table"]

# getting path to the generated mujoco xml model
model_path = items["path"]

# running mujoco thread
def execute_mujoco(mujoco_id,model_path,
                   balls,table):
    # init mujoco
    config = pam_mujoco.MujocoConfig()
    config.graphics = True
    config.extended_graphics = False
    config.realtime = True
    pam_mujoco.init_mujoco(config)
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
    pam_mujoco.execute(mujoco_id,model_path)
    # looping until requested to stop
    #while not pam_mujoco.is_stop_requested(mujoco_id):
    #    time.sleep(0.01)


# starting mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(mujoco_id,model_path,
                                         balls,table,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id,-1)

# playing 3 trajectories
for run in range(3):

    # initializing o80 frontend for sending ball/hit_point position/velocity
    # to mujoco thread
    frontend_ball1 = o80_pam.MirrorFreeJointFrontEnd("ball1")
    frontend_ball2 = o80_pam.MirrorFreeJointFrontEnd("ball2")
    frontends = (frontend_ball1,frontend_ball2)

    # reading a random pre-recorded ball trajectory
    _,trajectory_points = list(context.BallTrajectories().random_trajectory())

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

    # first mujoco iterations place the balls to starting point
    time.sleep(0.1)
        
    # starting iteration
    first_iteration = frontend_ball1.latest().get_iteration()

    # sanity check: ball2 touches the table
    contact_detected=False
    time_start=time.time()
    while time.time()-time_start < total_duration:
        contacts = pam_mujoco.get_contact("table")
        if contacts.contact_occured and not contact_detected:
            print("contact !")
            contact_detected=True
        time.sleep(0.05)

    # computing the distance between the 2 balls
    # getting all observations (i.e. history of ball states
    # as recorded by the o80 backend in the shared memory)
    observations_ball1 = frontend_ball1.get_observations_since(first_iteration)
    observations_ball2 = frontend_ball2.get_observations_since(first_iteration) 
    distances = []
    for ball1,ball2 in zip(observations_ball1,observations_ball2):
        # state has information : [position_x,velocity_x,position_y,velocity_y,...]
        state1 = ball1.get_observed_states()
        state2 = ball2.get_observed_states()
        position1 = [state1.get(2*dim).get() for dim in range(3)]
        position2 = [state2.get(2*dim).get() for dim in range(3)]
        distance = math.sqrt(sum([(p1-p2)**2 for p1,p2 in zip(position1,position2)]))
        distances.append(distance)
    # plotting the distance
    plt.plot(range(len(distances)),distances)
    plt.show()
    
        
    # resetting contacts
    pam_mujoco.reset_contact("table")
        
# stopping the mujoco thread
pam_mujoco.request_stop(mujoco_id)
process.join()



