import time, logging, shared_memory, o80, o80_pam, pam_mujoco_wrp
from functools import partial
from .mujoco_robot import MujocoRobot
from .mujoco_item import MujocoItem,MujocoItems
from . import models


def _get_mujoco_items_control(
    mujoco_items: MujocoItems, balls: list, goals: list, hit_points: list
):

    class_name = "".join(["Mujoco", str(mujoco_items.size), "ItemsControl"])
    class_ = getattr(pam_mujoco_wrp, class_name)

    if mujoco_items.control == MujocoItem.NO_CONTROL:
        return None

    if mujoco_items.control == MujocoItem.CONSTANT_CONTROL:
        active_only = False

    if mujoco_items.control == MujocoItem.COMMAND_ACTIVE_CONTROL:
        active_only = True

    if mujoco_items.contact_type is None:
        mujoco_items.contact_type = pam_mujoco_wrp.ContactTypes.no_contact

    all_model_items = balls + goals + hit_points
    all_model_types = (
        [pam_mujoco_wrp.MujocoItemTypes.ball] * len(balls)
        + [pam_mujoco_wrp.MujocoItemTypes.goal] * len(goals)
        + [pam_mujoco_wrp.MujocoItemTypes.hit_point] * len(hit_points)
    )

    types = []
    joints = []
    qpos = []
    qvel = []
    geometries = []

    for model_type, model_item, item in zip(
        all_model_types, all_model_items, mujoco_items.iterate()
    ):
        types.append(model_type)
        joints.append(model_item.joint)
        qpos.append(model_item.index_qpos)
        qvel.append(model_item.index_qvel)
        geometries.append(model_item.geom)

    return class_(
        types,
        mujoco_items.segment_id,
        joints,
        qpos,
        qvel,
        geometries,
        active_only,
        mujoco_items.contact_type,
    )


def _get_mujoco_item_control(
    mujoco_item_type: pam_mujoco_wrp.MujocoItemTypes,
    mujoco_item: MujocoItem,
    model_item: dict,
) -> pam_mujoco_wrp.MujocoItemControl:

    if mujoco_item.control == MujocoItem.NO_CONTROL:
        return None

    if mujoco_item.control == MujocoItem.CONSTANT_CONTROL:
        active_only = False

    if mujoco_item.control == MujocoItem.COMMAND_ACTIVE_CONTROL:
        active_only = True

    if mujoco_item.contact_type is None:
        mujoco_item.contact_type = pam_mujoco_wrp.ContactTypes.no_contact

    return pam_mujoco_wrp.MujocoItemControl(
        mujoco_item_type,
        mujoco_item.segment_id,
        model_item.joint,
        model_item.index_qpos,
        model_item.index_qvel,
        model_item.geom,
        active_only,
        mujoco_item.contact_type,
    )


def _get_mujoco_robot_control(mujoco_robot: MujocoRobot, model_item: dict):

    active_only = mujoco_robot.active_only_control == MujocoRobot.COMMAND_ACTIVE_CONTROL

    if mujoco_robot.control == MujocoRobot.JOINT_CONTROL:
        return pam_mujoco_wrp.MujocoRobotJointControl(
            mujoco_robot.segment_id, model_item.joint, active_only
        )
    if mujoco_robot.control == MujocoRobot.PRESSURE_CONTROL:
        return pam_mujoco_wrp.MujocoRobotPressureControl(
            mujoco_robot.segment_id,
            model_item.joint,
            active_only,
            mujoco_robot.json_control_path,
            mujoco_robot.json_ago_hill_path,
            mujoco_robot.json_antago_hill_path,
        )
    return None


class MujocoHandle:
    def __init__(
        self,
        mujoco_id: str,
        burst_mode: bool = False,
        accelerated_time: bool = False,
        graphics: bool = True,
        time_step: float = 0.002,
        table: bool = False,
        robot1: MujocoRobot = None,
        robot2: MujocoRobot = None,
        balls: list = [],  # list of mujoco_item.MujocoItem
        goals: list = [],  # list of mujoco_item.MujocoItem
        hit_points: list = [],
        combined: MujocoItems = None,  # list of mujoco_item.MujocoItems (with 's' at the end)
        read_only: bool = False,
    ):

        self._mujoco_id = mujoco_id

        if not read_only:

            # combined (instance of mujoco_item.MujocoItems)
            # supports only a limited set of size (see source of MujocoItems)
            self.combined=combined
            if combined and (combined.size not in combined.accepted_sizes):
                raise ValueError(
                    "pam_mujoco.mujoco_item.MujocoItems supports "
                    "a limited set of size ({}). "
                    "{} provived".format(
                        ", ".format(
                            [str(a) for a in combined.accepted_sizes], combined.size
                        )
                    )
                )

            # creating the mujoco xml model file

            logging.info("creating the xml model file for {}".format(mujoco_id))

            if combined:
                all_balls = balls + combined.items["balls"]
                all_goals = goals + combined.items["goals"]
                all_hit_points = hit_points + combined.items["hit_points"]
            else:
                all_balls = balls
                all_goals = goals
                all_hit_points = hit_points

            items = models.model_factory(
                mujoco_id,
                time_step=time_step,
                table=table,
                balls=all_balls,
                goals=all_goals,
                hit_points=all_hit_points,
                robot1=robot1,
                robot2=robot2,
            )

            # creating the mujoco config

            logging.info("creating mujoco configuration for {}".format(mujoco_id))

            config = pam_mujoco_wrp.MujocoConfig(mujoco_id)
            config.set_burst_mode(burst_mode)
            config.set_accelerated_time(accelerated_time)
            config.set_model_path(items["path"])
            config.set_graphics(graphics)

            if items["robot1"]:
                config.set_racket_robot1(items["robot1"].geom_racket)

            if items["robot2"]:
                config.set_racket_robot2(items["robot2"].geom_racket)

            if items["table"]:
                config.set_table(items["table"].geom_plate)

            _get_ball = partial(
                _get_mujoco_item_control, pam_mujoco_wrp.MujocoItemTypes.ball
            )
            _get_hit_point = partial(
                _get_mujoco_item_control, pam_mujoco_wrp.MujocoItemTypes.hit_point
            )
            _get_goal = partial(
                _get_mujoco_item_control, pam_mujoco_wrp.MujocoItemTypes.goal
            )

            mujoco_item_controls = []
            if balls:
                mujoco_item_controls.extend(
                    [
                        _get_ball(mujoco_item, model_item)
                        for mujoco_item, model_item in zip(
                            balls, items["balls"][: len(balls)]
                        )
                    ]
                )

            if hit_points:
                mujoco_item_controls.extend(
                    [
                        _get_hit_point(mujoco_item, model_item)
                        for mujoco_item, model_item in zip(
                            hit_points, items["hit_points"][: len(hit_points)]
                        )
                    ]
                )

            if goals:
                mujoco_item_controls.extend(
                    [
                        _get_goal(mujoco_item, model_item)
                        for mujoco_item, model_item in zip(
                            goals, items["goals"][: len(goals)]
                        )
                    ]
                )

            for mujoco_item_control in mujoco_item_controls:
                config.add_control(mujoco_item_control)

            if combined:

                mujoco_combined_items_control = _get_mujoco_items_control(
                    combined,
                    items["balls"][len(balls) :],
                    items["goals"][len(goals) :],
                    items["hit_points"][len(hit_points) :],
                )

                # function name, depending on the number of combined mujoco items.
                # e.g. add_3_control or add_10_control, see
                # include/mujoco_config.hpp and/or srcpy/wrappers.cpp
                add_function_name = "_".join(["add", str(combined.size), "control"])

                # pointer to the function
                add_function = getattr(config, add_function_name)
                # calling the function
                add_function(mujoco_combined_items_control)

            for key, robot in zip(("robot1", "robot2"), (robot1, robot2)):
                if robot:
                    r = _get_mujoco_robot_control(robot, items[key])
                    if r:
                        config.add_control(r)

            # writing the mujoco config in the shared memory.
            # the mujoco executable is expected to read it and start

            logging.info("sharing mujoco configuration for {}".format(mujoco_id))

            pam_mujoco_wrp.set_mujoco_config(config)

        # waiting for mujoco to report it is ready

        logging.info("waiting for mujoco executable {}".format(mujoco_id))

        pam_mujoco_wrp.wait_for_mujoco(mujoco_id)

        # if read only, we did not create the mujoco configuration,
        # (which has been written by another process)
        # so we read it from the shared memory
        
        if read_only:

            config = pam_mujoco_wrp.get_mujoco_config(mujoco_id)
            balls, goals, hit_points = [], [], []
            for item in config.item_controls:
                if item.active_only:
                    control = MujocoItem.COMMAND_ACTIVE_CONTROL
                else:
                    control = MujocoItem.CONSTANT_CONTROL
                instance = MujocoItem(item.segment_id, control=control)
                if item.type == pam_mujoco_wrp.MujocoItemTypes.ball:
                    balls.append(instance)
                elif item.type == pam_mujoco_wrp.MujocoItemTypes.goal:
                    goals.append(instance)
                else:
                    hit_points.append(instance)
            robots = []
            for joint_robot in config.joint_controls:
                r = MujocoRobot(
                    joint_robot.segment_id, control=MujocoRobot.JOINT_CONTROL
                )
                robots.append(r)
            for pressure_robot in config.pressure_controls:
                r = MujocoRobot(
                    joint_robot.segment_id, control=MujocoRobot.PRESSURE_CONTROL
                )
                robots.append(r)
            try:
                robot1 = robots[0]
            except:
                robot1 = None
            try:
                robot2 = robots[1]
            except:
                robot2 = None

        # if bursting mode, creating a burster client
        if burst_mode:
            self._burster_client = o80.BursterClient(mujoco_id)
        else:
            self._burster_client = None

        # creating o80 frontends

        self.frontends = {}
        self.interfaces = {}

        for item in balls:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info(
                    "creating o80 frontend for ball {} /  {}".format(
                        mujoco_id, item.segment_id
                    )
                )
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80Ball(item.segment_id, frontend)
                self.frontends[item.segment_id] = frontend
                self.interfaces[item.segment_id] = interface

        for item in goals:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info(
                    "creating o80 frontend for goal {} /  {}".format(
                        mujoco_id, item.segment_id
                    )
                )
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80Goal(item.segment_id, frontend)
                self.frontends[item.segment_id] = frontend
                self.interfaces[item.segment_id] = interface

        for item in hit_points:
            if item.control != MujocoItem.NO_CONTROL:
                logging.info(
                    "creating o80 frontend for hit point {} /  {}".format(
                        mujoco_id, item.segment_id
                    )
                )
                frontend = o80_pam.BallFrontEnd(item.segment_id)
                interface = o80_pam.o80HitPoint(item.segment_id, frontend)
                self.frontends[item.segment_id] = frontend
                self.interfaces[item.segment_id] = interface

        for robot in robot1, robot2:
            if robot:
                if robot.control == MujocoRobot.JOINT_CONTROL:
                    logging.info(
                        "creating o80 frontend for joint control of {} /  {}".format(
                            mujoco_id, robot.segment_id
                        )
                    )
                    frontend = o80_pam.JointFrontEnd(robot.segment_id)
                    interface = o80_pam.o80RobotMirroring(
                        robot.segment_id,
                        frontend=frontend,
                        burster=self._burster_client,
                    )
                    self.frontends[robot.segment_id] = frontend
                    self.interfaces[robot.segment_id] = interface
                if robot.control == MujocoRobot.PRESSURE_CONTROL:
                    logging.info(
                        "creating o80 frontend for pressure control of {} /  {}".format(
                            mujoco_id, robot.segment_id
                        )
                    )
                    frontend = o80_pam.FrontEnd(robot.segment_id)
                    interface = o80_pam.o80Pressures(
                        robot.segment_id,
                        frontend=frontend,
                        burster=self._burster_client,
                    )
                    self.frontends[robot.segment_id] = frontend
                    self.interfaces[robot.segment_id] = interface

        if combined:

            # e.g. Balls3Frontend for 3 balls.
            # see pam_mujoco/srcpy/wrappers.cpp
            frontend_class_name = "".join(["Balls",str(combined.size),"FrontEnd"])
            frontend_class = getattr(pam_mujoco_wrp,frontend_class_name)
            self.frontends[combined.segment_id]=frontend_class(combined.segment_id)

        # for tracking contact
        self.contacts = {}

        for item in list(balls) + list(goals) + list(hit_points):
            if item.contact_type == pam_mujoco_wrp.ContactTypes.table:
                self.contacts[item.segment_id] = item.segment_id + "_table"
            if item.contact_type == pam_mujoco_wrp.ContactTypes.racket1:
                self.contacts[item.segment_id] = item.segment_id + "_racket1"
            if item.contact_type == pam_mujoco_wrp.ContactTypes.racket2:
                self.contacts[item.segment_id] = item.segment_id + "_racket2"
                
        if combined:
            # see src/add_controllers.cpp, function add_items_control
            for index,item in enumerate(list(combined.iterate())):
                if item.contact_type == pam_mujoco_wrp.ContactTypes.table:
                    self.contacts[item.segment_id] = combined.segment_id + "_table_" +str(index)
                if item.contact_type == pam_mujoco_wrp.ContactTypes.racket1:
                    self.contacts[item.segment_id] = combined.segment_id + "_racket1_" +str(index)
                if item.contact_type == pam_mujoco_wrp.ContactTypes.racket2:
                    self.contacts[item.segment_id] = combined.segment_id + "_racket2_" +str(index)

        for sid in self.contacts.keys():
            self.reset_contact(sid)
                
    def reset(self):
        # sharing the reset commands
        for _, interface in self.interfaces.items():
            interface.reset()
        if self.combined:
            self.frontends[self.combined.segment_id].add_reinit_command()
            self.frontends[self.combined.segment_id].pulse()
        shared_memory.set_bool(self._mujoco_id, "reset", True)
        # waiting one iteration to make sure reset has been active
        if self._burster_client is not None:
            self.burst(nb_iterations=1)
        else:
            start_mstep = self.get_mujoco_step()
            while self.get_mujoco_step() == start_mstep:
                time.sleep(0.0005)

    def get_mujoco_step(self):
        return shared_memory.get_long_int(self._mujoco_id, "nbsteps")

    def pause(self, value):
        shared_memory.set_bool(self._mujoco_id, "pause", value)

    def get_contact(self, segment_id):
        return pam_mujoco_wrp.get_contact(self.contacts[segment_id])

    def reset_contact(self, segment_id):
        return pam_mujoco_wrp.reset_contact(self.contacts[segment_id])

    def activate_contact(self, segment_id):
        return pam_mujoco_wrp.activate_contact(self.contacts[segment_id])

    def deactivate_contact(self, segment_id):
        return pam_mujoco_wrp.deactivate_contact(self.contacts[segment_id])

    def burst(self, nb_iterations=1):
        self._burster_client.burst(nb_iterations)

    def mujoco_exit(self):
        shared_memory.set_bool(self._mujoco_id, "exit", True)
        if self._burster_client:
            self._burster_client.final_burst()
