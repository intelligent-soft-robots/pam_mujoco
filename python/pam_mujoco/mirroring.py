import time,threading
import o80_pam
from collections.abc import Iterable


class ParallelBurst:
    def __init__(self, mirrorings, wait=0.001):
        self._size = len(mirrorings)
        self._running = True
        self._mirrorings = mirrorings
        self._burst_done = None
        self._nb_bursts = None
        self._wait = wait
        self._threads = [
            threading.Thread(target=self._run, args=(index,))
            for index in range(self._size)
        ]
        for thread in self._threads:
            thread.start()

    def _run(self, index):
        while self._running:
            if (self._nb_bursts is not None) and not self._burst_done[index]:
                self._mirrorings[index].burst(self._nb_bursts)
                self._burst_done[index] = True
            else:
                time.sleep(self._wait)

    def burst(self, nb_bursts):
        self._burst_done = [False] * self._size
        self._nb_bursts = nb_bursts
        while not all(self._burst_done):
            time.sleep(self._wait)
        self._burst_done = [False] * self._size
        self._nb_bursts = None

    def stop(self):
        self._running = False
        for thread in self._threads:
            thread.join()

    def __del__(self):
        self.stop()


# this moves the mirrored robot to the same position as the
# (pseudo) real robot, using incremental steps to avoid
# simulation unstabilities
def align_robots(o80_pressures,o80_mirroring,step=0.01):

    if not isinstance(o80_mirroring,Iterable):
        mirrorings = [o80_mirroring]
    else:
        mirrorings = o80_mirroring
        
    _,__,target_positions,target_velocities = o80_pressures.read()

    def _one_step(arg):
        target,current=arg
        diff = target-current
        if(abs(diff)<step):
            current=target
            return True,current
        else:
            if diff>0:
                current+=step
            else:
                current-=step
            return False,current

    def _align(target_positions, target_velocities, mirroring):
    
        positions,velocities = mirroring.get()

        over=[False]*len(target_positions)

        while not all(over):

            p = list(map(_one_step,zip(target_positions,positions)))
            v = list(map(_one_step,zip(target_velocities,velocities)))

            positions = [p_[1] for p_ in p]
            velocities = [v_[1] for v_ in v]

            over = [p_[0] for p_ in p]

            mirroring.set(positions,velocities,nb_iterations=1,burst=1)

    if len(mirrorings)==1:
        _align(target_positions,target_velocities,mirrorings[0])

    else:
        threads = [threading.Thread(target=_align,args=(target_positions,target_velocities,mirroring))
                   for mirroring in mirrorings]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
            

# this has the (pseudo) real robot moving to a pressure posture
# (action : [(ago,antago),...])
# while being mirrored by the mujoco pam robot (assumed to run in bursting mode)
def go_to_pressure_posture(o80_pressures:o80_pam.o80Pressures,
                           o80_mirroring:o80_pam.o80RobotMirroring,
                           action,
                           duration_s:float,
                           accelerated_time:bool,
                           mujoco_time_step=0.002,
                           o80_time_step=0.002,
                           parallel_burst=None):

    if not isinstance(o80_mirroring,Iterable):
        mirrorings = [o80_mirroring]
    else:
        mirrorings = o80_mirroring

    def _reached_target(error=50):
        pressures_ago,pressures_antago,_,__ = o80_pressures.read()
        for dof in range(len(action)):
            if abs(pressures_ago[dof]-action[dof][0])>error:
                return False
            if abs(pressures_antago[dof]-action[dof][1])>error:
                return False
        return True

    if accelerated_time:
        o80_pressures.add_command(action,int(duration_s*1000))
        nb_bursts = int((duration_s / o80_time_step) + 0.5)+10 
        for _ in range(nb_bursts):
            o80_pressures.burst(1)
            pressures_ago,pressures_antagos,positions,velocities = o80_pressures.read()
            for mirroring in mirrorings:
                mirroring.set(positions,velocities)
            parallel_burst.burst(1)
        while not _reached_target():
            o80_pressures.burst(1)
            pressures_ago,pressures_antagos,positions,velocities = o80_pressures.read()
            for mirroring in mirrorings:
                mirroring.set(positions,velocities)
            parallel_burst.burst(1)
        return

    o80_pressures.set(action,duration_ms=int(duration_s*1000+0.5),
                      wait=False,burst=False)
    time_start = time.time()
    while time.time()-time_start < duration_s:
        pressures_ago,pressures_antago,positions,velocities = o80_pressures.read()
        for mirroring in mirrorings:
            mirroring.set(positions,velocities)
        if parallel_burst is not None:
            parallel_burst.burst(1)
        else:
            for mirroring in mirrorings:
                mirroring.burst(1)
        time.sleep(mujoco_time_step)
    while not _reached_target():
        pressures_ago,pressures_antago,positions,velocities = o80_pressures.read()
        for mirroring in mirrorings:
            mirroring.set(positions,velocities)
        if parallel_burst is not None:
            parallel_burst.burst(1)
        else:
            for mirroring in mirrorings:
                mirroring.burst(1)
        time.sleep(mujoco_time_step)
        

# this has the (pseudo) real robot moving to a position posture
# (posture: [angles in radian]
# while being mirrored by the mujocjo pam robot (assumed to run
# in bursting mode)
def go_to_position_posture(o80_pressures:o80_pam.o80Pressures,
                           o80_mirroring:o80_pam.o80RobotMirroring,
                           posture,
                           pam_config,
                           accelerated_time):

    if not isinstance(o80_mirroring,Iterable):
        mirrorings = [o80_mirroring]
    else:
        mirrorings = o80_mirroring
        
    config = o80_pam.JointPositionControllerConfig(o80_pressures,
                                           pam_config)
    joint_controller = o80_pam.JointPositionController(config,
                                                       accelerated_time)

    for _,positions,velocities,__ in joint_controller.go_to(posture):
        if positions is not None:
            for mirroring in mirrorings:
                mirroring.set(positions,velocities,nb_iterations=1,burst=1)
