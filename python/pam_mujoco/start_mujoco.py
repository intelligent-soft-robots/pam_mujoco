import multiprocessing
import pam_mujoco
import o80_pam
import pam_models
import pam_interface


def _get_pam_model_configuration(segment_id, pam_interface_config, robot):
    pam_model_config_path = pam_models.get_default_config_path()
    a_init = [0.5] * 8
    l_MTC_change_init = [0.0] * 8
    scale_min_activation = [0.001] * 8
    scale_max_activation = [1.0] * 8
    scale_min_pressure = [None] * 8
    scale_max_pressure = [None] * 8
    for dof in range(4):
        scale_min_pressure[2 * dof] = pam_interface_config.min_pressures_ago[dof]
        scale_min_pressure[2 * dof + 1] = pam_interface_config.min_pressures_antago[dof]
        scale_max_pressure[2 * dof] = pam_interface_config.max_pressures_ago[dof]
        scale_max_pressure[2 * dof + 1] = pam_interface_config.max_pressures_antago[dof]
    pam_model_config = [
        segment_id,
        robot.joint,
        scale_min_pressure,
        scale_max_pressure,
        scale_min_activation,
        scale_max_activation,
        pam_model_config_path,
        pam_model_config_path,
        a_init,
        l_MTC_change_init,
    ]
    return pam_model_config


def pseudo_real_robot(
    mujoco_id,
    segment_id,
    pam_interface_config: pam_interface.Configuration,
    graphics=True,
    extended_graphics=False,
    realtime=True,
):

    # creating the xml mujoco model
    items = pam_mujoco.model_factory(segment_id, robot1=True)
    robot = items["robot"]

    # artificial muscles config
    pam_model_config = _get_pam_model_configuration(
        segment_id, pam_interface_config, robot
    )

    # function running mujoco
    def _execute_mujoco(mujoco_id, segment_id, pam_model_config):

        model_name = segment_id

        # init mujoco
        config = pam_mujoco.MujocoConfig()
        config.graphics = graphics
        config.extended_graphics = extended_graphics
        config.realtime = realtime
        pam_mujoco.init_mujoco(config)

        # adding pressure controller
        pam_mujoco.add_pressure_controller(*pam_model_config)

        # starting the thread
        model_path = pam_mujoco.paths.get_model_path(model_name)
        pam_mujoco.execute(mujoco_id, model_path)

        # looping until requested to stop
        while not pam_mujoco.is_stop_requested(mujoco_id):
            time.sleep(0.01)

    # starting mujoco
    process = multiprocessing.Process(
        target=_execute_mujoco, args=(mujoco_id, segment_id, pam_model_config)
    )

    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id, -1)

    return process


def ball_and_robot(
    mujoco_id,
    segment_id_robot,
    segment_id_contact_robot,
    segment_id_ball,
    segment_id_burst=None,
    graphics=True,
    extended_graphics=False,
    realtime=True,
    segment_id_goal=None,
    segment_id_hit_point=None,
):

    # creating the xml mujoco model
    use_goal = True if segment_id_goal is not None else False
    use_hit_point = True if segment_id_hit_point is not None else False
    items = pam_mujoco.model_factory(
        segment_id_robot,
        table=True,
        robot1=True,
        goal=use_goal,
        hit_point=use_hit_point,
        muscles=False,
    )
    ball = items["ball"]
    robot = items["robot"]
    goal = items["goal"] if use_goal else None
    hit_point = items["hit_point"] if use_hit_point else None

    segment_ids = {
        "ball": segment_id_ball,
        "robot": segment_id_robot,
        "contact_robot": segment_id_contact_robot,
        "burst": segment_id_burst,
        "goal": segment_id_goal,
        "hit_point": segment_id_hit_point,
    }

    def _execute_mujoco(mujoco_id, ball, robot, goal, hit_point, segment_ids):

        model_name = segment_ids["robot"]

        # init mujoco
        config = pam_mujoco.MujocoConfig()
        config.graphics = graphics
        config.extended_graphics = extended_graphics
        config.realtime = realtime
        pam_mujoco.init_mujoco(config)

        active_only = True
        not_active_only = False

        # for detecting contact with the robot
        if segment_ids["contact_robot"] is not None:
            pam_mujoco.add_robot1_contact_free_joint(
                segment_ids["contact_robot"],
                ball.index_qpos,
                ball.index_qvel,
                ball.geom,
                robot.geom_racket,
            )

        # adding the mirror ball controller, will play
        # recorded ball trajectories, until contact with robot
        if segment_ids["contact_robot"] is not None:
            pam_mujoco.add_mirror_until_contact_free_joint(
                segment_ids["ball"],
                ball.joint,
                ball.index_qpos,
                ball.index_qvel,
                segment_ids["contact_robot"],
            )
        else:
            pam_mujoco.add_mirror_free_joint(
                segment_ids["ball"],
                ball.joint,
                ball.index_qpos,
                ball.index_qvel,
                active_only,
            )

        if segment_ids["goal"] is not None:
            pam_mujoco.add_mirror_free_joint(
                segment_ids["goal"],
                goal.joint,
                goal.index_qpos,
                goal.index_qvel,
                not_active_only,
            )

        if segment_ids["hit_point"] is not None:
            pam_mujoco.add_mirror_free_joint(
                segment_ids["hit_point"],
                hit_point.joint,
                hit_point.index_qpos,
                hit_point.index_qvel,
                not_active_only,
            )

        # adding mirroring robot controller
        pam_mujoco.add_mirror_robot(segment_ids["robot"], robot.joint)

        # set bursting mode if requested
        if segment_ids["burst"] is not None:
            pam_mujoco.add_bursting(mujoco_id, segment_ids["burst"])

        # starting the thread
        model_path = pam_mujoco.paths.get_model_path(model_name)
        pam_mujoco.execute(mujoco_id, model_path)

        # looping until requested to stop
        try:
            while not pam_mujoco.is_stop_requested(mujoco_id):
                time.sleep(0.01)
        except:
            pass

    # starting mujoco
    process = multiprocessing.Process(
        target=_execute_mujoco,
        args=(mujoco_id, ball, robot, goal, hit_point, segment_ids),
    )
    pam_mujoco.clear(mujoco_id)
    process.start()
    pam_mujoco.wait_for_mujoco(mujoco_id, -1)

    return process
