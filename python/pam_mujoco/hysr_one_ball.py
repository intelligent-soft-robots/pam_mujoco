import time
import math
import o80
import o80_pam
import pam_mujoco
import context
from . import start_mujoco
from . import mirroring 


def _distance(p1,p2):
    return math.sqrt(sum([(p2_-p1_)**2
                          for p1_,p2_ in zip(p1,p2)]))


def _norm(p):
    return math.sqrt(sum([p_**2 for p_ in p]))


def _reward( min_distance_ball_target,
             min_distance_ball_racket, # None if ball hit racket
             max_ball_velocity, # None if should not be taken into account (return task)
             c,rtt_cap ): 

    # returning r_hit
    
    if min_distance_ball_racket is not None:
        return -min_distance_ball_racket

    # returning r_tt

    reward = 1.- c * min_distance_ball_target ** 0.75

    # return task
    if max_ball_velocity is None: 
        reward = max(reward,rtt_cap)
        return reward

    # smash task
    reward = reward * max_ball_velocity
    reward = max(reward,rtt_cap)
    return reward
            
        
class _BallStatus:

    def __init__(self,
                 target_position,
                 segment_id_contact_robot,
                 frontend_ball):
        
        self.target_position = target_position
        self.segment_id_contact_robot = segment_id_contact_robot
        self.frontend_ball = frontend_ball
        self.reset()

    def reset(self):

        self.min_distance_ball_racket = float("+inf") # will be None if ball hit the racket
        self.min_distance_ball_target = float("+inf")
        self.max_ball_velocity = 0
        self.table_contact_position = None
        self.min_z = float("+inf")
        self.max_y = float("-inf")
        self.ball_position = [None]*3
        self.ball_velocity = [None]*3
        
    def update(self):

       # reading position of the ball
        ball_states = self.frontend_ball.pulse().get_observed_states()
        self.ball_position = [None]*3
        self.ball_velocity = [None]*3
        for dim in range(3):
            self.ball_position[dim]=ball_states.get(2*dim).get()
            self.ball_velocity[dim]=ball_states.get(2*dim+1).get()
        self.min_z = min(self.ball_position[2],self.min_z)
        self.max_y = max(self.ball_position[1],self.max_y)

        # updating min distance ball/racket
        contacts_racket = pam_mujoco.get_contact(self.segment_id_contact_robot)
        hit_racket = False
        if contacts_racket.contact_occured:
            self.min_distance_ball_racket = None
            hit_racket = True
        else :
            self.min_distance_ball_racket = contacts_racket.minimal_distance

        # if post contact with racket, updating min distance ball/target
        if hit_racket:
            d = _distance(self.ball_position,self.target_position)
            self.min_distance_ball_target = min(d,self.min_distance_ball_target)

        # if post contact with racket, updating max ball velocity
        if hit_racket:
            v = _norm(self.ball_velocity)
            self.max_ball_velocity = max(self.max_ball_velocity,v)
        

class _Observation:

    def __init__(self,
                 pressures_ago,
                 pressures_antago,
                 ball_position,
                 ball_velocity):
        self.pressures_ago = pressures_ago
        self.pressures_antago = pressures_antago
        self.ball_position = ball_position
        self.ball_velocity = ball_velocity

    
class HysrOneBall:

    def __init__(self,mujoco_id,
                 target_position,
                 reward_normalization_constant,
                 smash_task,
                 period_ms=100,
                 rtt_cap=0.2):

        # period config
        self._period_ms = period_ms

        # reward configuration
        self._c = reward_normalization_constant 
        self._smash_task = smash_task # True or False (return task)
        self._rtt_cap = rtt_cap

        # o80 segment ids
        # for contacting backend running in pseudo-real robot
        segment_id_pressure = mujoco_id+"_pressures"
        # for contacting backends running in simulated robot/ball
        segment_id_ball = mujoco_id+"_ball"
        segment_id_mirror_robot = mujoco_id+"_robot"
        self._segment_id_contact_robot = mujoco_id+"_contact_robot"

        
        # start pseudo-real robot (pressure controlled)
        model_name_pressure = segment_id_pressure
        self._mujoco_id_pressure = "mujoco_"+segment_id_pressure
        self._process_pressures = start_mujoco.pseudo_real_robot(segment_id_pressure,
                                                                 model_name_pressure,
                                                                 self._mujoco_id_pressure)

        # start simulated ball and robot
        model_name_sim = segment_id_ball
        self._mujoco_id_sim = "mujoco_"+segment_id_ball
        self._process_sim = start_mujoco.ball_and_robot(segment_id_mirror_robot,
                                                        self._segment_id_contact_robot,
                                                        segment_id_ball,
                                                        model_name_sim,
                                                         self._mujoco_id_sim )

        # starting a process that has the simulated robot
        # mirroring the pseudo-real robot
        # (uses o80 in the background)
        self._process_mirror = mirroring.start_mirroring(self._mujoco_id_sim,
                                                         segment_id_mirror_robot,
                                                         segment_id_pressure,
                                                         1)

        # o80 instance for getting (simulated) ball information
        self._frontend_ball = pam_mujoco.MirrorFreeJointFrontEnd(segment_id_ball)

        # o80 instance for getting current pressures
        self._frontend_pressures = o80_pam.FrontEnd(segment_id_pressure)

        # will encapsulate all information
        # about the ball (e.g. min distance with racket, etc)
        self._ball_status = _BallStatus(target_position,
                                        self._segment_id_contact_robot,
                                        self._frontend_ball)
        
    def _ball_gun(self):

        # reading a random pre-recorded ball trajectory
        trajectory_points = list(context.BallTrajectories().random_trajectory())

        # sending the full ball trajectory 
        # duration of 10ms : sampling rate of the trajectory
        duration = o80.Duration_us.milliseconds(10)
        for traj_point in trajectory_points:
            # looping over x,y,z
            for dim in range(3):
                # setting position for dimension (x, y or z)
                self._frontend_ball.add_command(2*dim,
                                     o80.State1d(traj_point.position[dim]),
                                     duration,
                                     o80.Mode.QUEUE)
                # setting velocity for dimension (x, y or z)
                self._frontend_ball.add_command(2*dim+1,
                                                o80.State1d(traj_point.velocity[dim]),
                                                duration,
                                                o80.Mode.QUEUE)
        self._frontend_ball.pulse()

        
    def reset(self):

        # resetting ball info, e.g. min distance ball/racket, etc
        self._ball_status.reset()
        # resetting ball/robot contact information
        pam_mujoco.reset_contact(self._segment_id_contact_robot)
        # shooting a ball
        self._ball_gun()
        time.sleep(0.3)
        
    def _episode_over(self):

        over = False
        
        # ball falled below the table
        if self._ball_status.min_z < -0.4:
            over = True

        # ball passed the racket
        elif self._ball_status.max_y > 0.1:
            over = True

        return over
    
    # action assumed to be [(pressure ago, pressure antago), (pressure_ago, pressure_antago), ...]
    def step(self,action):

        # gathering information about simulated ball
        self._ball_status.update()

        # computing reward
        if self._smash_task:
            reward = _reward( self._ball_status.min_distance_ball_target,
                              self._ball_status.min_distance_ball_racket,
                              self._ball_status.max_ball_velocity,
                              self._c,self._rtt_cap)
        else:
            reward = _reward( self._ball_status.min_distance_ball_target,
                              self._ball_status.min_distance_ball_racket,
                              None,
                              self._c,self._rtt_cap)
            
            
        # generating observation
        pressures = self._frontend_pressures.latest().get_observed_pressures()
        pressures_ago = [pressures[dof][0] for dof in range(4)]
        pressures_antago = [pressures[dof][1] for dof in range(4)]
        observation = _Observation(pressures_ago,pressures_antago,
                                   self._ball_status.ball_position,
                                   self._ball_status.ball_velocity)
        
        # sending pressures to pseudo real robot
        for dof,(ago_pressure,antago_pressure) in enumerate(action):
            self._frontend_pressures.add_command(dof,
                                 ago_pressure,antago_pressure,
                                 o80.Duration_us.milliseconds(self._period_ms),
                                 o80.Mode.OVERWRITE)
        self._frontend_pressures.pulse()

        return observation,reward,self._episode_over()

    
    def close(self):

        # stopping pseudo real robot
        pam_mujoco.request_stop(self._mujoco_id_pressure)
        # stopping simulated robot/ball
        pam_mujoco.request_stop(self._mujoco_id_sim)
        # waiting for all corresponding processes
        # to finish
        self._process_pressures.join()
        self._process_sim.join()
        self._process_mirror.join()
