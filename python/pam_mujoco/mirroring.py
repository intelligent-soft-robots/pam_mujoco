import multiprocessing
import pam_mujoco
import o80_pam
import pam_interface
import o80

def _mirroring(mujoco_id,
               segment_id_mirror_robot,
               segment_id_pressure,
               period_ms):
    
    frontend_mirroring = pam_mujoco.MirrorRobotFrontEnd(segment_id_mirror_robot)
    frontend_pressures = o80_pam.FrontEnd(segment_id_pressure)

    state = o80.State2d(0,0)
    duration = o80.Duration_us.milliseconds(int(period_ms))

    frequency_manager = o80.FrequencyManager(1.0/(period_ms/1000.0))
    
    while not pam_mujoco.is_stop_requested(mujoco_id):

        # reading position/velocity of the pseudo-real robot
        # (robot_state is an instance of RobotState, defined in package
        # pam_interface)
        robot_state = frontend_pressures.pulse().get_extended_state()
        robot_joints = [robot_state.get_position(dof)
                        for dof in range(4)]
        robot_joint_velocities = [robot_state.get_velocity(dof)
                                       for dof in range(4)]

        # sending mirroring commands
        for dof,(position,velocity) in enumerate(zip(robot_joints,
                                                     robot_joint_velocities)):
            state.set(0,position)
            state.set(1,velocity)
            frontend_mirroring.add_command(dof,state,duration,o80.Mode.OVERWRITE)
        frontend_mirroring.pulse()

        # running at desired period
        frequency_manager.wait()


def start_mirroring(mujoco_id,
                    segment_id_mirror_robot,
                    segment_id_pressure_robot,
                    period_ms):

    process  = multiprocessing.Process(target=_mirroring,
                                       args=(mujoco_id,segment_id_mirror_robot,
                                             segment_id_pressure_robot,period_ms,))
    process.start()
    return process


