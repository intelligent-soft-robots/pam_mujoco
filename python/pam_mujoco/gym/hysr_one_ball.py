import math
import gym
import o80
import pam_mujoco


def _distance(p1,p2):
    return math.sqrt(sum([(p2_-p1_)**2
                          for p1_,p2_ in zip(p1,p2)]))

def _norm(p):
    return math.sqrt(sum([p_**2 for p in p]))



def _reward( position_hit_table, # None if did not hit
             min_distance_ball_target,
             min_distance_ball_racket, # None if ball hit racket
             max_ball_velocity, # None if should not be taken into account (return task)
             c,rtt_cap ): 

    # returning r_hit
    
    if min_distance_ball_racket is not None:
        return -min_distance_ball_racket

    # returning r_tt

    reward = 1.- c * min_ball_distance_ball_target ** 0.75

    # return task
    if max_ball_velocity is None: 
        reward = max(reward,rtt_cap)
        return reward

    # smash task
    reward = reward * max_ball_velocity
    reward = max(reward,rtt_cap)
    return reward
    
    
class _EpisodeStatus:

    def __init__(self,
                 target_position,
                 segment_id_contact_racket,
                 frontend_mirroring,
                 frontend_ball,
                 frontend_pressures)
        self.target_position = target_position
        self.segment_id_contact_racket = segment_id_contact_racket
        self.frontend_mirroring = self.frontend_mirroring
        self.frontend_ball = self.frontend_ball
        self.frontend_pressures = self.frontend_pressures
        self.min_distance_ball_racket = float("+inf") # will be None if ball hit the racket
        self.min_distance_ball_target = float("+inf")
        self.max_ball_velocity = 0
        self.robot_joints = [None]*4
        self.robot_joint_velocities = [None]*4

    def update(self):

        # reading position of the ball
        ball_states = self.frontend_ball.pulse().get_current_states()
        ball_position = [None]*3
        ball_velocity = [None]*3
        for dim in range(3):
            ball_position[dim]=ball_states.get(2*dim).value
            ball_velocity[dim]=ball_states.get(2*dim+1).value

        # updating min distance ball/racket
        contacts_racket = pam_mujoco.get_contact(self.segment_id_contact_racket)
        hit_racket = False
        if contacts_racket.contact_occured:
            self.min_distance_ball_racket = None
            hit_racket = True
        else :
            self.min_distance_ball_racket = contacts_racket.mininal_distance

        # if post contact with racket, updating min distance ball/target
        if hit_racket:
            d = _distance(ball_position,self.target_position)
            self.min_distance_ball_target = min(d,self.min_distance_ball_target)

        # if post contact with racket, updating max ball velocity
        if hit_racket:
            v = _norm(ball_velocity)
            self.max_ball_velocity = max(self.max_ball_velocity,v)
        
        
    

        
class PamEnv(gym.Env):

    def __init__(self,mujoco_id="hysr_one_ball",
                 reward_normalization_constant,
                 smash_task,
                 rtt_cap=0.2):

        # reward configuration
        self._c = reward_normalization_constant 
        self._smash_task = smash_task # True or False (return task)
        self._rtt_cap = rtt_cap

        # robot conf
        self._nb_dofs=4

        # o80 segment ids
        segment_id_pressure = mujoco_id+"_pressures"
        segment_id_ball = mujoco_id+"_ball"
        segment_id_mirror_robot = mujoco_id+"_robot"
        self._segment_id_contact_table = mujoco_id+"_contact_table"
        self._segment_id_contact_robot = mujoco_id+"_contact_robot"
        
        # start pseudo-real robot (pressure controlled)
        model_name_pressure = segment_id_pressure
        mujoco_id_pressure = segment_id_pressure
        self._process_pressures = start_mujoco.pseudo_real_robot(segment_id_pressure,
                                                                 model_name_pressure,
                                                                 mujoco_id_pressure)

        # start simulated ball and robot
        model_name_sim = mujoco_id+"_sim"
        mujoco_id_sim = mujoco_id+"_sim"
        self._process_sim = start_mujoco.ball_and_robot(segment_id_contact_table,
                                                        segment_id_mirror_robot,
                                                        segment_id_contact_robot,
                                                        segment_id_ball,
                                                        model_name_sim,
                                                        mujoco_id_sim)

        
        # o80 instances
        self._frontend_mirroring = pam_mujoco.MirrorRobotFrontEnd(segment_id_mirror_robot)
        self._frontend_ball = pam_mujoco.MirrorFreeJointFrontEnd(segment_id_ball)
        self._frontend_pressures = o80_pam.FrontEnd(segment_id_pressure)

        self._action_space = gym.spaces.Box(low=-1.0,
                                            high=1.0,
                                            shape=(self._nb_dofs,),
                                            dtype=np.float)

    def step(self):

        pass
