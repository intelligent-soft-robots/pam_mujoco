import multiprocessing
import pam_mujoco
import o80_pam
import pam_models


def _get_pam_model_configuration(segment_id,robot):
    pam_model_config_path= pam_models.get_default_config_path()
    a_init = [0.5]*8
    l_MTC_change_init = [0.0]*8
    scale_max_activation = 1.0
    scale_max_pressure = 24000
    scale_min_activation = 0.001
    scale_min_pressure = 6000
    pam_model_config = [ segment_id,
                         robot.joint,
                         scale_min_pressure,scale_max_pressure,
                         scale_min_activation, scale_max_activation,
                         pam_model_config_path,pam_model_config_path,
                         a_init,l_MTC_change_init ]
    return pam_model_config

def pseudo_real_robot(mujoco_id,segment_id):

    # creating the xml mujoco model
    items = pam_mujoco.model_factory(segment_id,
                                     robot1=True)
    robot = items["robot"]

    # artificial muscles config
    pam_model_config = _get_pam_model_configuration(segment_id,robot)
    
    # function running mujoco
    def _execute_mujoco(mujoco_id,
                        segment_id,
                        pam_model_config):

        model_name = segment_id
        
        # init mujoco
        pam_mujoco.init_mujoco()

        # adding pressure controller
        pam_mujoco.add_pressure_controller(*pam_model_config)

        # starting the thread
        pam_mujoco.execute(mujoco_id,model_name)

        # looping until requested to stop
        while not pam_mujoco.is_stop_requested(mujoco_id):
            time.sleep(0.01)

    # starting mujoco
    process  = multiprocessing.Process(target=_execute_mujoco,
                                       args=(mujoco_id,segment_id,
                                             pam_model_config,))

    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id)

    return process



def ball_and_robot(mujoco_id,
                   segment_id_robot,
                   segment_id_contact_robot,
                   segment_id_ball,
                   segment_id_burst=None):

    # creating the xml mujoco model
    items = pam_mujoco.model_factory(segment_id_robot,
                                     table=True,robot1=True)
    ball = items["ball"]
    robot = items["robot"]

    segment_ids = {"ball":segment_id_ball,
                   "robot":segment_id_robot,
                   "contact_robot":segment_id_contact_robot,
                   "burst":segment_id_burst}

    def _execute_mujoco(mujoco_id,
                        ball,
                        robot,
                        segment_ids):

        model_name = segment_ids["robot"]

        # init mujoco
        pam_mujoco.init_mujoco()

        # for detecting contact with the robot
        pam_mujoco.add_robot1_contact_free_joint(segment_ids["contact_robot"],
                                                 ball.index_qpos,ball.index_qvel,
                                                 ball.geom,robot.geom_racket)

        # adding the mirror ball controller, will play
        # recorded ball trajectories, until contact with robot
        pam_mujoco.add_mirror_until_contact_free_joint(segment_ids["ball"],
                                                       ball.joint,
                                                       ball.index_qpos,ball.index_qvel,
                                                       segment_ids["contact_robot"])

        # adding mirroring robot controller
        pam_mujoco.add_mirror_robot(segment_ids["robot"],robot.joint)

        # set bursting mode if requested
        if segment_ids["burst"] is not None:
            pam_mujoco.add_bursting(mujoco_id,segment_ids["burst"])

        # starting the thread
        pam_mujoco.execute(mujoco_id,model_name)

        # looping until requested to stop
        while not pam_mujoco.is_stop_requested(mujoco_id):
            time.sleep(0.01)

    # starting mujoco
    process  = multiprocessing.Process(target=_execute_mujoco,
                                       args=(mujoco_id,
                                             ball,
                                             robot,
                                             segment_ids))
    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id)

    return process

            
