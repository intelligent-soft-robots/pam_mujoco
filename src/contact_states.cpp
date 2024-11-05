#include "pam_mujoco/internal/contact_states.hpp"

namespace pam_mujoco
{
  namespace internal
  {
   
    ContactStates::ContactStates() : time_stamp(-1), velocity_time_stamp(-1)
    {
    }
    
    template<typename Array>
    void print_array(std::string label, const Array& a);
    
    template<std::size_t N>
    void print_array(std::string label, const std::array<double, N>& a)
    {
      std::cout << "\t" << label << "\t";
      for(auto& elem : a)
        {
          std::cout << elem << " ";                                                                                                                                                                      
        }
      std::cout << std::endl;
    }
    
    void ContactStates::print() const
    {
      std::cout << "contact state:" << std::endl;
      std::cout << "\ttime stamp: " << time_stamp << std::endl;
      std::cout << "\tvelocity time stamp: " << velocity_time_stamp << std::endl;
      print_array(std::string("robot joints"), robot_joint_positions);
      print_array(std::string("contactee orientation"), contactee_orientation);
      print_array(std::string("contactee position"), contactee_position);
      print_array(std::string("contactee velocity"), contactee_velocity);
      print_array(std::string("ball position"), ball_position);
      print_array(std::string("ball velocity"), ball_velocity);
    }

    void save_robot_joints(const mjData* d,
                           int index_robot_qpos,
                           std::array<double, 4>& robot_joints)
    {
      if(index_robot_qpos<0)
        {
          return;
        }
      for(int i=0;i<4;i++)
        {
          robot_joints[i] = d->qpos[index_robot_qpos + i];
        }
    }
                           
    void save_contactee(const mjData* d,
                        int index_geom_contactee,
                        std::array<double,3>& contactee_position,
                        std::array<double,9>& contactee_orientation)
    {
      for(int i=0;i<3;i++)
        {
          contactee_position[i] = d->geom_xpos[index_geom_contactee*3+i];
        }
      for(int i=0;i<9;i++)
        {
          contactee_orientation[i] = d->geom_xmat[index_geom_contactee*9+i];
        }
    }
    
    void save_ball(const mjData* d,
                   int index_qpos,
                   std::array<double,3>& ball_position,
                   int index_qvel,
                   std::array<double,3>& ball_velocity)
    {
      for(int i=0;i<3;i++)
        {
          ball_position[i] = d->qpos[index_qpos + i];
          ball_velocity[i] = d->qvel[index_qvel + i];
        }
    }

    void set_contactee_velocity(const mjData* d,
                                double dt,
                                int index_geom_contactee,
                                const std::array<double,3>& contactee_position,
                                std::array<double,3>& contactee_velocity)
    {
      for (size_t i = 0; i < 3; i++)
        {
          contactee_velocity[i] =
            (d->geom_xpos[index_geom_contactee * 3 + i] -
             contactee_position[i]) /
            dt;
        }
    }
    
    void update_contactee_velocity(const mjData* d,
                                   int index_geom_contactee,
                                   bool new_step,
                                   internal::ContactStates& get_states)
    {
      // velocity of contactee computed with finite differences

      // first iteration, we can not compute finite difference yet
      if (get_states.time_stamp < 0)
        {
          for (size_t i = 0; i < 3; i++) get_states.contactee_velocity[i] = 0;
          get_states.velocity_time_stamp = d->time;
          get_states.time_stamp = d->time;
          return;
        }

      double dt = d->time - get_states.velocity_time_stamp;
      set_contactee_velocity(d,dt,index_geom_contactee,
                             get_states.contactee_position,
                             get_states.contactee_velocity);
      get_states.velocity_time_stamp = d->time;
    }
  
    /**
     * extract information from d to update the instance
     * of get_states. position of the ball, velocity of the ball,
     * position of the contactee and orientation of the contactee
     * are directly read from d.
     * the velocity of the contactee is computed via finite difference
     * using the position of the contactee as provided by get_states.
     * (i.e. get_states is also expected to encapsulte the previous
     * state).
     */
    void save_state(const mjData* d,
                    int index_robot_qpos,
                    int index_qpos,
                    int index_qvel,
                    int index_geom_contactee,
                    bool new_step,
                    internal::ContactStates& get_states)
    {
      // When running learning_table_tennis_from_scratch, the positions of
      // the robot joints are updated at a lower frequency: during the
      // intermediate iterations, this thread "sees" an immobile racket,
      // i.e. a velocity of zero, which is incorrect. We update the robot velocity
      // only after it has been updated by the learning algorithm
      if (!new_step)
        {
          return;
        }
      update_contactee_velocity(d, index_geom_contactee, new_step, get_states);
      save_robot_joints(d,index_robot_qpos, get_states.robot_joint_positions);
      save_ball(d,
                index_qpos, get_states.ball_position,
                index_qvel, get_states.ball_velocity);
      save_contactee(d, index_geom_contactee,
                     get_states.contactee_position, get_states.contactee_orientation);
      get_states.time_stamp = d->time;
    }
  
  }
  
}
