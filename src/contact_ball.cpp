#include "pam_mujoco/contact_ball.hpp"

namespace pam_mujoco
{
ContactBall::ContactBall(std::string segment_id,
                         int index_qpos,
                         int index_qvel,
                         std::string geom,
                         std::string geom_contactee,
                         ContactItems contact_item)
    : segment_id_{segment_id},
      config_{internal::get_recompute_state_config(contact_item)},
      index_qpos_{index_qpos},
      index_qvel_{index_qvel},
      geom_{geom},
      geom_contactee_{geom_contactee},
      index_geom_{-1},
      index_geom_contactee_{-1},
      mujoco_detected_contact_{false},
      in_contact_{false},
      nb_of_iterations_since_last_contact_{-1}
{
    shared_memory::clear_shared_memory(segment_id_);
    shared_memory::set<bool>(segment_id_, "reset", false);
    shared_memory::set<bool>(segment_id_, "activated", true);
    shared_memory::serialize(segment_id_, segment_id_, contact_information_);
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

bool ContactBall::update(const mjModel* m, mjData* d)
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
        return false;
    }
    else
    {
        contact_information_.disabled = false;
    }

    // if there have been a recent contact,
    // contacts are ignored for a while (to avoid
    // the "same" contact to be applied twice)
    if (nb_of_iterations_since_last_contact_ >= 0)
    {
        if (nb_of_iterations_since_last_contact_ < NB_ITERATIONS_CONTACT_MUTED)
        {
            nb_of_iterations_since_last_contact_++;
            return false;
        }
        else
        {
            // reactivating contact detection
            nb_of_iterations_since_last_contact_ = -1;
        }
    }
    // contact are not being ignored
    // does mujoco reports a contact ?
    bool in_contact =
        internal::is_in_contact(m, d, index_geom_, index_geom_contactee_);

    if (in_contact)
    {
        steps_contact_remaining_ = 200;
        printf("contact detected\n");
    }
        

    if (steps_contact_remaining_ > 0 and !in_contact)
    {
        printf("contact lost, but still looking\n");
        steps_contact_remaining_--;
        in_contact = true;
    }

    // no, so exiting
    if (!in_contact)
    {
        // before exiting, saving the state of the ball.
        // it will be used if at the next iteration there
        // is a contact (to compute velocity)
        internal::save_state(
            d, index_qpos_, index_qvel_, index_geom_contactee_, previous_);
        // saving also the distance between the ball
        // and the contactee.
        // computing the distance between the 2 objects
        double d_ball_contactee =
            mju_dist3(previous_.ball_position.data(),
                      previous_.contactee_position.data());

        double d_ball_contactee_current = std::sqrt(
            std::pow(d->qpos[index_qpos_ + 0] - d->geom_xpos[index_geom_contactee_ * 3 + 0], 2) +
            std::pow(d->qpos[index_qpos_ + 1] - d->geom_xpos[index_geom_contactee_ * 3 + 1], 2) +
            std::pow(d->qpos[index_qpos_ + 2] - d->geom_xpos[index_geom_contactee_ * 3 + 2], 2));


        // printing contact information
        if (d_ball_contactee_current < 0.10)
        {
            double d_ball_contactee_xz = std::sqrt(
            std::pow(previous_.ball_position[0] - previous_.contactee_position[0], 2) +
            std::pow(previous_.ball_position[2] - previous_.contactee_position[2], 2));

            double d_ball_contactee_y = previous_.ball_position[1] - previous_.contactee_position[1];

            printf("previous d_ball_contactee: %.4f d_ball_contactee_xz: %.4f, d_ball_contactee_y: %.4f, ball pos: %.4f, racket pos: %.4f\n",
                d_ball_contactee, d_ball_contactee_xz, d_ball_contactee_y, previous_.ball_position[1], previous_.contactee_position[1]);


            double d_ball_contactee_xz_current = std::sqrt(
            std::pow(d->qpos[index_qpos_ + 0] - d->geom_xpos[index_geom_contactee_ * 3 + 0], 2) +
            std::pow(d->qpos[index_qpos_ + 2] - d->geom_xpos[index_geom_contactee_ * 3 + 2], 2));

            double d_ball_contactee_y_current = d->qpos[index_qpos_ + 1] - d->geom_xpos[index_geom_contactee_ * 3 + 1];

            printf("current d_ball_contactee: %.4f d_ball_contactee_xz: %.4f, d_ball_contactee_y: %.4f, ball pos: %.4f, racket pos: %.4f\n",
                d_ball_contactee, d_ball_contactee_xz_current, d_ball_contactee_y_current, d->qpos[index_qpos_ + 1], d->geom_xpos[index_geom_contactee_ * 3 + 1]);
        }

        // previous contactee_velocity all close to zero, don't consider contact
        if (previous_.contactee_velocity[0] < 0.01 and previous_.contactee_velocity[1] < 0.01 and previous_.contactee_velocity[2] < 0.01)
        {
            return false;
        }

        // updating contact_information with this distance. register_distance
        // will check wether or not this is the smallest distance ever observed,
        // and if so, save it as minimal distance
        contact_information_.register_distance(d_ball_contactee_current);
        return false;
    }

    

    // there is a contact: applying the custom contact model
    // first, converting data encapsulated by "d"
    // into an instance of ContactStates
    internal::ContactStates current;
    internal::save_state(
        d, index_qpos_, index_qvel_, index_geom_contactee_, current);

    // the command below updates overwite_ball_position_
    // and overwrite_ball_velocity_ with the values
    // commanded by the contact_model
    bool success = recompute_state_after_contact(config_,
                                                 previous_,
                                                 current,
                                                 overwrite_ball_position_,
                                                 overwrite_ball_velocity_);
    if (!success)
    {
        // failed to applie the custom model because
        // the ball and the contactee were too close.
        // we postpone to next iteration ...
        // internal::save_state(
        //     d, index_qpos_, index_qvel_, index_geom_contactee_, previous_);
        return false;
    }

    // to "shut down" the few next contact detection
    // (which may come from the same contact)
    nb_of_iterations_since_last_contact_ = 0;
    // informing the outside world about the contact
    // (via shared memory)
    printf("register contact\n");
    steps_contact_remaining_ = -1;
    contact_information_.register_contact(current.ball_position, d->time);
    shared_memory::serialize(segment_id_, segment_id_, contact_information_);
    return true;
}

void ContactBall::apply(const mjModel* m, mjData* d)
{
    // check if the simulation is running
    if (d->time < 0.02)
        return;
    
    // check if ball position and velocity are close to zero
    if (std::abs(d->qvel[index_qvel_]) < 0.01 &&
        std::abs(d->qvel[index_qvel_ + 1]) < 0.01 &&
        std::abs(d->qvel[index_qvel_ + 2]) < 0.01 &&
        std::abs(d->qpos[index_qpos_]) < 0.01 &&
        std::abs(d->qpos[index_qpos_ + 1]) < 0.01 &&
        std::abs(d->qpos[index_qpos_ + 2]) < 0.01)
    {
        printf("ignoring ball at 0\n");
        return;
    }



    // checking if it is a new mujoco iteration
    if (this->must_update(d))
    {
        // setup the indexes (e.g. index_qpos, ...)
        // based on the model and the configuration string
        // (has effect only at first call)
        init(m);
    }
        // updating the variables:
        // - contact_information_ // information about contacts, for the
        // external
        //                           world (written in shared memory)
        // - overwrite_ball_position_ // position as computed by custom contact
        // model
        // - overwrite_ball_velocity_ // velocity as computed by custom contact
        // model

    
    in_contact_ = update(m, d);

    // sharing contact information with the world
    shared_memory::serialize(
        segment_id_, segment_id_, contact_information_);
    

    // no contact, so nothing to do
    if (!in_contact_)
    {
        if (steps_overwrite_remaining_ <= 0)
        {
            return;
        }
        else
        {
            steps_overwrite_remaining_--;
        }
    }
    else
    {
        steps_overwrite_remaining_ = 4;
    }
    

    

    // we reach here only if in contact,
    // so overwritting mujoco data
    // with custom contact model data

    printf("before overwrite ball pos %.4f, %.4f, %.4f vel: %.4f, %.4f, %.4f \n",
        d->qpos[index_qpos_], d->qpos[index_qpos_ + 1], d->qpos[index_qpos_ + 2],
        d->qvel[index_qvel_], d->qvel[index_qvel_ + 1], d->qvel[index_qvel_ + 2]);


    // check if ball new conditions are plausible (weird bug)
    if (abs(overwrite_ball_position_[0]) > 10 || abs(overwrite_ball_position_[1]) > 10 || abs(overwrite_ball_position_[2]) > 10
        || abs(overwrite_ball_velocity_[0]) > 50 || abs(overwrite_ball_velocity_[1]) > 50 || abs(overwrite_ball_velocity_[2]) > 50
        || (abs(overwrite_ball_velocity_[0]) < 0.01 && abs(overwrite_ball_velocity_[1]) < 0.01 && abs(overwrite_ball_velocity_[2]) < 0.01))
    {
        printf("contact model problem, not overwriting\n");
        printf("calculated ball pos %.4f, %.4f, %.4f vel: %.4f, %.4f, %.4f \n",
            overwrite_ball_position_[0], overwrite_ball_position_[1], overwrite_ball_position_[2],
            overwrite_ball_velocity_[0], overwrite_ball_velocity_[1], overwrite_ball_velocity_[2]);
        return;
    }
    else
    {
        for (int dim = 0; dim < 3; dim++)
        {
            (&(d->qpos[index_qpos_]))[dim] = overwrite_ball_position_[dim];
            (&(d->qvel[index_qvel_]))[dim] = overwrite_ball_velocity_[dim];
        }
    }
    
    printf("after overwrite ball pos %.4f, %.4f, %.4f vel: %.4f, %.4f, %.4f \n",
        d->qpos[index_qpos_], d->qpos[index_qpos_ + 1], d->qpos[index_qpos_ + 2],
        d->qvel[index_qvel_], d->qvel[index_qvel_ + 1], d->qvel[index_qvel_ + 2]);

}

void ContactBall::init(const mjModel* m)
{
    if (index_geom_ >= 0)
    {
        // init does something only at first call
        return;
    }
    index_geom_ = mj_name2id(m, mjOBJ_GEOM, geom_.c_str());
    index_geom_contactee_ = mj_name2id(m, mjOBJ_GEOM, geom_contactee_.c_str());
}

void ContactBall::reset()
{
    contact_information_ = context::ContactInformation{};
    previous_ = internal::ContactStates{};
    nb_of_iterations_since_last_contact_ = -1;
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
