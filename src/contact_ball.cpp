#include "pam_mujoco/contact_ball.hpp"

namespace pam_mujoco
{


ContactMode::ContactMode()
    : steps_since_contact_{-1},
      steps_since_overwrite_{-1},
      index_qpos_{-1},
      index_qvel_{-1},
      mujoco_time_step_{ControllerBase::MUJOCO_TIME_STEP_MS}
{}

void ContactMode::set_contact_overwrite(mjData* d,
                                        const double* ball_position,
                                        const double* ball_velocity)
{
    steps_since_overwrite_ = 0;
    steps_since_contact_ = -1;
    contact_time_ = d->time;
    for(int i=0;i<3;i++)
        {
            ball_position_[i] = ball_position[i];
            ball_velocity_[i] = ball_velocity[i];
        }
    apply_model(d);
}

void ContactMode::apply_model(mjData* d)
{
    for (int dim = 0; dim < 3; dim++)
    {
        (&(d->qpos[index_qpos_]))[dim] = ball_position_[dim];
        (&(d->qvel[index_qvel_]))[dim] = ball_velocity_[dim];
    }
}

void ContactMode::init(int index_qpos, int index_qvel, int index_geom, int index_geom_contactee)
{
    index_qpos_ = index_qpos;
    index_qvel_ = index_qvel;
    index_geom_ = index_geom;
    index_geom_contactee_ = index_geom_contactee;
}
    
void ContactMode::reset()
{
    steps_since_contact_=-1;
    steps_since_overwrite_=-1;
}
    
bool ContactMode::contact_active(const mjModel* m, mjData* d)
{
    if(steps_since_overwrite_ >=0)
        {
            // there has been a contact and the custom model has to be applied
            // during the whole iteration
            o80::TimePoint ct{static_cast<long int>(contact_time_ * 1e9)};
            o80::TimePoint now{static_cast<long int>(d->time * 1e9)};
            if( (now-ct) < mujoco_time_step_ )
                {
                    apply_model(d);
                }
            
            // there has been a contact, and the custom contact model
            // has been applied. We turn contact off for a while, so that
            // the same contact is not applied twice.
            if(steps_since_overwrite_<NB_ITERATIONS_CONTACT_MUTED)
                {
                    steps_since_overwrite_++;
                    return false;
                }
            else
                {
                    // we muted contacts long enough, resetting
                    steps_since_overwrite_ = -1;
                }
        }
    
    if(steps_since_contact_>=0)
        {
            
            // A contact has been detected by mujoco, but not "applied" yet
            // (i.e. the custom contact model has not been applied to the ball yet)
            // If this mujoco detected contact is recent, we consider it as still
            // active
            if(steps_since_contact_<NB_ITERATIONS_CONTACT_ACTIVE)
                {
                    steps_since_contact_++;
                    return true;
                }
            else
                {
                    // we kept this contact "alive" long enough, resetting
                    steps_since_contact_ = -1;
                }
        }

    // contacts are neither "active" (from a contact detected in a previous iteration)
    // or "muted" (because a contact has been applied recently). Evaluating things anew.
    bool mujoco_contact = internal::is_in_contact(m, d,
                                                  index_geom_, index_geom_contactee_);

    if(mujoco_contact)
        {
            // entering contact mode !
            steps_since_contact_ = 0;
        }
    
    return mujoco_contact;
}

    
ContactBall::ContactBall(std::string mujoco_id,
                         std::string segment_id,
                         std::string joint,
                         std::string geom,
                         std::string robot_base,
                         std::string geom_contactee,
                         ContactItems contact_item)
    : mujoco_id_{mujoco_id},
      segment_id_{segment_id},
      contact_mode_{},
      config_{internal::get_recompute_state_config(contact_item)},
      joint_{joint},
      index_qpos_{-1},
      index_qvel_{-1},
      robot_base_{robot_base},
      index_robot_qpos_{-1},
      geom_{geom},
      geom_contactee_{geom_contactee},
      index_geom_{-1},
      index_geom_contactee_{-1},
      step_{-2}
{
    shared_memory::clear_shared_memory(segment_id_);
    shared_memory::set<bool>(segment_id_, "reset", false);
    shared_memory::set<bool>(segment_id_, "activated", true);
    shared_memory::serialize(segment_id_, segment_id_, contact_information_);
    shared_memory::set<long int>(mujoco_id_, "step", -2);
}

template <int DIM, class P, class V>
void printxd(P p, V v)
{
    for (int dim = 0; dim < DIM; dim++)
    {
        std::cout << p[dim] << " , ";
    }
    std::cout << " | ";
    for (int dim = 0; dim < DIM; dim++)
    {
        std::cout << v[dim] << " , ";
    }
}


void ContactBall::init(const mjModel* m)
{

    if (index_geom_ >= 0)
    {
        // init does something only at first call
        return;
    }
    index_qpos_ = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, joint_.c_str())];
    index_qvel_ = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, joint_.c_str())];
    index_geom_ = mj_name2id(m, mjOBJ_GEOM, geom_.c_str());
    index_geom_contactee_ = mj_name2id(m, mjOBJ_GEOM, geom_contactee_.c_str());
    if (index_robot_qpos_ < 0 && robot_base_ != "")
    {
      index_robot_qpos_ = m->jnt_qposadr[mj_name2id(
            m, mjOBJ_JOINT, robot_base_.c_str())];
    }
    contact_mode_.init(index_qpos_, index_qvel_, index_geom_, index_geom_contactee_);
}

void ContactBall::reset()
{
    contact_information_ = context::ContactInformation{};
    previous_ = internal::ContactStates{};
    contact_mode_.reset();
}


bool ContactBall::user_signals()
{
    // check if receiving from outside a request for reset
    // (e.g. a new episode starts)
    bool must_reset;
    shared_memory::get<bool>(segment_id_, "reset", must_reset);
    if (must_reset)
    {
        reset();
        shared_memory::set<bool>(segment_id_, "reset", false);
    }

    // checking if the contacts are activated
    bool activated;
    shared_memory::get<bool>(segment_id_, "activated", activated);
    if (!activated)
    {
        contact_information_.disabled = true;
        return true;
    }
    else
    {
        contact_information_.disabled = false;
        return false;
    }
}

void ContactBall::share_contact_info()
{
    // sharing contact information with the world
    shared_memory::serialize(segment_id_,
                             segment_id_,
                             contact_information_);

}

bool ContactBall::no_apply(const mjData* d)
{
    // check if the simulation is running
    if (d->time < 0.02)
        return true;

    // is this required ?
    /*
    double thres = 0.01;
    bool skip = true;
    for(int index=0;index<3;index++)
        {
            
            if(std::abs(d->qvel[index_qvel_+index])<thres || std::abs(d->qpos[index_qpos_+index])<thres)
                {
                    skip=false;
                    break;
                }
        }
    return skip;
    */

    return false;
}
    
void ContactBall::save_state(const mjData* d, bool new_step, internal::ContactStates& cs)
{
    internal::save_state(
                         d,
                         index_robot_qpos_,
                         index_qpos_,
                         index_qvel_,
                         index_geom_contactee_,
                         new_step,
                         cs
                         );
}
    
void ContactBall::execute(const mjModel* m, mjData* d)
{

    // the user may use the python API to
    // disable or reset the contacts. This method
    // check for it. We exit if the contacts are ignored.
    if(user_signals())
      {
          return ;
      }

    // in some corner case we should not apply the
    // contacts: the simulation just started, or
    // the position and speed of the ball is too close to 0
    if(no_apply(d))
        {
            return;
        }

    // True if a new algorithm step, i.e. the robot position
    // and velocity have just been updated ("mirrored") by the
    // learning algo (learning table tennis from scratch,hysr_on_ball.py)
    bool new_step = new_algo_step();

    // evaluating if there is a contact to deal with
    bool contact = contact_mode_.contact_active(m,d);

    // "fresh" iteration ?
    bool update = this->must_update(d);
    
    if(!contact)
        {
          if (update)
            {
              // no contact to deal with, exit after saving state and
              // monitoring shorter distance between ball and contactee
              save_state(d, new_step, previous_);
              double d_ball_contactee =
                mju_dist3(previous_.ball_position.data(),
                          previous_.contactee_position.data());
              contact_information_.register_distance(d_ball_contactee);
              return;
            }
        }
    
    // we deal with the contact only for "fresh" mujoco iteration
    if(!update)
        return;

    // we arrive here because there is a contact that has not been
    // applied yet (i.e. ball trajectory has not been changed
    // based on the custom model)
    // computing the custom model
    bool success = recompute_state_after_contact(config_,
                                                 previous_,
                                                 d->time,
                                                 overwrite_ball_position_,
                                                 overwrite_ball_velocity_);

    if (!success)
    {
        // failed to apply the custom model because
        // the ball and the contactee were too close.
        // we postpone to next iteration ...
        save_state(d, new_step, previous_);
        return;
    }


    // applying the custom model, and 
    // informing contact mode, so that the contact
    // is not applied a second time in the upcoming iterations
    contact_mode_.set_contact_overwrite(d, overwrite_ball_position_, overwrite_ball_velocity_);
    // informing the outside world about the contact
    // (via shared memory)
    std::array<double,3> ball_position;
    for(size_t i=0;i<3;i++)
      ball_position[i] = overwrite_ball_position_[i];
    contact_information_.register_contact(ball_position, d->time);
}

bool ContactBall::new_algo_step()
{
    long int current_step;
    shared_memory::get<long int>(mujoco_id_, "step", current_step);
    bool r = false;
    if(current_step != step_)
    {
      r = true;
    }
    step_ = current_step;
    return r;
}

void ContactBall::apply(const mjModel* m, mjData* d)
{
    init(m);
    execute(m,d);
    share_contact_info();
}


void reset_contact(const std::string& segment_id)
{
    shared_memory::set<bool>(segment_id, "reset", true);
}

void activate_contact(const std::string& segment_id)
{
    shared_memory::set<bool>(segment_id, "activated", true);
}

void deactivate_contact(const std::string& segment_id)
{
    shared_memory::set<bool>(segment_id, "activated", false);
}

}  // namespace pam_mujoco
