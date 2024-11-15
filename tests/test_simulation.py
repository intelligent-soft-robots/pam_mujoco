import subprocess
import time

import numpy as np
import pytest

import pam_mujoco


def test_realtime_step_duration():
    mujoco_id = "simulation"
    process = subprocess.Popen(f"launch_pam_mujoco {mujoco_id}", shell=True)
    time.sleep(1)

    try:
        robot_segment_id = "robot"
        time_step = 0.002
        robot = pam_mujoco.MujocoRobot(
            pam_mujoco.RobotType.PAMY2,
            robot_segment_id,
            control=pam_mujoco.MujocoRobot.PRESSURE_CONTROL,
        )
        handle = pam_mujoco.MujocoHandle(
            mujoco_id,
            robot1=robot,
            time_step=time_step,
            graphics=False,
            accelerated_time=False,
            burst_mode=False,
        )

        frontend = handle.frontends[robot_segment_id]

        curr_time_sim = frontend.latest().get_time_stamp()
        last_time_sim = curr_time_sim
        last_wall_time = time.time()
        steps_sim = []
        durations_wall = []
        for _ in range(100):
            while curr_time_sim == last_time_sim:
                curr_time_sim = frontend.latest().get_time_stamp()
            curr_wall_time = time.time()
            steps_sim.append(curr_time_sim - last_time_sim)
            durations_wall.append(curr_wall_time - last_wall_time)
            last_time_sim = curr_time_sim
            last_wall_time = time.time()
        assert np.all(np.array(steps_sim) == int(time_step * 1e9))
        # There is some fluctuation in the timings of individual steps,
        # but the average should match the timestep well.
        assert np.mean(durations_wall) == pytest.approx(time_step, abs=5e-5)
    finally:
        process.kill()
