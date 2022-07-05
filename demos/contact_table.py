import time
import o80
import o80_pam
import pam_mujoco
import multiprocessing

segment_id = "play_trajectory"
mujoco_id = "mj"

# generates pamy.xml in pam_mujoco/models/tmp/
model_name = "trajectory"
items = pam_mujoco.model_factory(model_name, table=True, hit_point=True)

# getting the items
ball = items["ball"]
table = items["table"]
hit_point = items["hit_point"]


# running mujoco thread
def execute_mujoco(mujoco_id, model_name, ball, hit_point, table):
    # init mujoco
    config = pam_mujoco.MujocoConfig()
    config.graphics = True
    config.extended_graphics = False
    config.realtime = True
    pam_mujoco.init_mujoco(config)
    # adding the mirror ball controller
    pam_mujoco.add_mirror_until_contact_free_joint(
        "ball", ball.joint, ball.index_qpos, ball.index_qvel, "contact"
    )
    # adding the hit point controller
    pam_mujoco.add_mirror_free_joint(
        "hit_point", hit_point.joint, hit_point.index_qpos, hit_point.index_qvel
    )
    # adding detection of contact between ball and table
    pam_mujoco.add_table_contact_free_joint(
        "contact", ball.index_qpos, ball.index_qvel, ball.geom, table.geom_plate
    )
    # starting the thread
    pam_mujoco.execute(mujoco_id, model_name)
    # looping until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)


# starting mujoco thread
process = multiprocessing.Process(
    target=execute_mujoco,
    args=(
        mujoco_id,
        model_name,
        ball,
        hit_point,
        table,
    ),
)
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)

# initializing o80 frontend for sending ball/hit_point position/velocity
# to mujoco thread
frontend_ball = o80_pam.MirrorFreeJointFrontEnd("ball")
frontend_hit_point = o80_pam.MirrorFreeJointFrontEnd("hit_point")

# dropping the ball on the table
start_point = [1.0, 2.0, 1]
end_point = [1.5, 1.0, -1]
duration = 2  # seconds
velocity = [(e - s) / float(duration) for e, s in zip(end_point, start_point)]
for dim in range(3):
    frontend_ball.add_command(2 * dim, o80.State1d(start_point[dim]), o80.Mode.QUEUE)
    frontend_ball.add_command(2 * dim + 1, o80.State1d(velocity[dim]), o80.Mode.QUEUE)
for dim in range(3):
    frontend_ball.add_command(
        2 * dim,
        o80.State1d(end_point[dim]),
        o80.Duration_us.seconds(duration),
        o80.Mode.QUEUE,
    )


frontend_ball.pulse()

# monitoring contact ball/table
# and moving hit point to contact location
while True:
    contacts = pam_mujoco.get_contact("contact")
    if contacts.contact_occured:
        # contact detected: moving the hit point at the contact
        # position
        position = contacts.position
        print("contact:", position)
        for dim, p in enumerate(position):
            frontend_hit_point.add_command(2 * dim, o80.State1d(p), o80.Mode.QUEUE)
        frontend_hit_point.pulse()
        break


time.sleep(3)

# stopping the mujoco thread
pam_mujoco.request_stop("mj")
process.join()
