import multiprocessing
import pam_mujoco
import o80_pam


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


def pseudo_real_robot(segment_id,
                      model_name,
                      mujoco_id):

    # creating the xml mujoco model
    items = pam_mujoco.model_factory(model_name,
                                     robot1=True)
    robot = items["robot"]

    # artificial muscles config
    pam_model_config = _get_pam_model_configuration(segment_id,robot)
    
    # function running mujoco
    def _execute_mujoco(segment_id,
                        mujoco_id,
                        model_name,
                        pam_model_config):

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
    process  = multiprocessing.Process(target=execute_mujoco,
                                       args=(segment_id,mujooc_id,model_name,
                                             pam_model_config,))
    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id)

    return process



def ball_and_robot(segment_id_table,
                   segment_id_robot,
                   segment_id_contact_robot,
                   segment_id_ball,
                   model_name,
                   mujoco_id):

    # creating the xml mujoco model
    items = pam_mujoco.model_factory(model_name,
                                     table=True,robot1=True)
    ball = items["ball"]
    table = items["table"]
    robot = items["robot"]

    segment_ids = {"ball":segment_id_ball,
                   "table",segment_id_table,
                   "robot",segment_id_robot,
                   "contact_robot",segment_id_contact_robot}

    def _execute_mujoco(ball,
                        table,
                        robot,
                        segment_ids,
                        mujoco_id,
                        model_name):

        # init mujoco
        pam_mujoco.init_mujoco()

        # for detecting contact with the robot
        pam_mujoco.add_robot1_contact_free_joint(segment_ids["robot"],
                                                 segment_ids["robot"]+"_reset",
                                                 ball.index_qpos,ball.index_qvel,
                                                 ball.geom,robot.geom_racket)

        #for detecting contact with the table
        pam_mujoco.add_table_contact_free_joint(segment_ids["table"],
                                                segment_ids["table"]+"_reset",
                                                ball.index_qpos,balls[1].index_qvel,
                                                ball.geom,table.geom_plate)

        # adding the mirror ball controller, will play
        # recorded ball trajectories, until contact with robot
        pam_mujoco.add_mirror_until_contact_free_joint(segment_ids["ball"],
                                                       ball.joint,
                                                       ball.index_qpos,ball.index_qvel,
                                                       "racket")

        # adding mirroring robot controller
        pam_mujoco.add_mirror_robot(segment_ids["robot"],robot.joint)

        # starting the thread
        pam_mujoco.execute(mujoco_id,model_name)

        # looping until requested to stop
        while not pam_mujoco.is_stop_requested(mujoco_id):
            time.sleep(0.01)

    # starting mujoco
    process  = multiprocessing.Process(target=execute_mujoco,
                                       args=(ball,
                                             table,
                                             robot,
                                             segment_ids,
                                             mujoco_id,))
    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id)

    return process

            
