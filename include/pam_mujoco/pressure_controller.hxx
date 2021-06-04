

template <int QUEUE_SIZE, int NB_DOFS>
PressureController<QUEUE_SIZE, NB_DOFS>::PressureController(
    std::string segment_id,
    std::string robot_joint_base,
    std::array<double, NB_DOFS * 2> scale_min_pressure,
    std::array<double, NB_DOFS * 2> scale_max_pressure,
    std::array<double, NB_DOFS * 2> scale_min_activation,
    std::array<double, NB_DOFS * 2> scale_max_activation,
    std::string muscle_json_config_path_ago,
    std::string muscle_json_config_path_antago,
    std::array<double, NB_DOFS * 2> a_init,
    std::array<double, NB_DOFS * 2> l_MTC_change_init)
    : ControllerBase{},
      backend_(segment_id),
      robot_joint_base_(robot_joint_base),
      index_q_robot_(-1),
      index_qvel_robot_(-1),
      scale_min_pressure_(scale_min_pressure),
      scale_min_activation_(scale_min_activation),
      scale_max_pressure_(scale_max_pressure),
      scale_max_activation_(scale_max_activation),
      iteration_(0)
{
    for (std::size_t dof = 0; dof < NB_DOFS; dof++)
    {
        muscles_.push_back(
            pam_models::hill::from_json(muscle_json_config_path_ago,
                                        a_init[dof * 2],
                                        l_MTC_change_init[dof * 2]));
        muscles_.push_back(
            pam_models::hill::from_json(muscle_json_config_path_antago,
                                        a_init[dof * 2 + 1],
                                        l_MTC_change_init[dof * 2 + 1]));
    }
}

template <int QUEUE_SIZE, int NB_DOFS>
void PressureController<QUEUE_SIZE, NB_DOFS>::apply(const mjModel* m, mjData* d)
{
    // init
    if (index_q_robot_ < 0)
    {
        index_q_robot_ = m->jnt_qposadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
        index_qvel_robot_ = m->jnt_dofadr[mj_name2id(
            m, mjOBJ_JOINT, robot_joint_base_.c_str())];
    }

    if (this->must_update(d))
    {
        // current state, for input to o80 (i.e. observation generation)
        States current_states;
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
            current_states.values[dof * 2].set(
                activation2pressure(dof * 2, d->act[dof * 2]));
            current_states.values[dof * 2 + 1].set(
                activation2pressure(dof * 2 + 1, d->act[dof * 2 + 1]));
        }

        // reading current robot state
        pam_interface::TimePoint time_stamp_pi(
            static_cast<long int>(d->time * 1e9));
        pam_interface::RobotState<NB_DOFS> robot_state(
            iteration_, iteration_, time_stamp_pi);
        iteration_++;
        for (std::size_t dof = 0; dof < NB_DOFS; dof++)
        {
            robot_state.set_joint(
                dof,
                activation2pressure(
                    dof * 2, d->act[dof * 2]),  // current pressure agonist
                activation2pressure(
                    dof * 2 + 1,
                    d->act[dof * 2 + 1]),       // current pressure antagonist
                d->ctrl[dof * 2],               // desired pressure agonist
                d->ctrl[dof * 2 + 1],           // desired pressure antagonist
                d->qpos[index_q_robot_ + dof],  // position
                d->qvel[index_qvel_robot_ + dof],  // velocity
                0,                                 // encoder
                true);                             // reference found
        }

        // reading desired pressure from o80
        states_ =
            backend_.pulse(o80::TimePoint(static_cast<long int>(d->time * 1e9)),
                           current_states,
                           robot_state);
    }

    for (std::size_t dof = 0; dof < NB_DOFS; dof++)
    {
        double activation_ago =
            pressure2activation(dof * 2, states_.get(dof * 2).get());
        double activation_antago =
            pressure2activation(dof * 2 + 1, states_.get(dof * 2 + 1).get());
        if (activation_ago <= -0.99 || activation_ago >= 0.99)
        {
            if (activation_ago < 0.0)
            {
                activation_ago = -0.99;
            }
            else
            {
                activation_ago = +0.99;
            }
        }
        if (activation_antago < -0.99 || activation_antago > 0.99)
        {
            if (activation_antago < 0.0)
            {
                activation_antago = -0.99;
            }
            else
            {
                activation_antago = +0.99;
            }
        }

        d->ctrl[dof * 2] = activation_ago;
        d->ctrl[dof * 2 + 1] = activation_antago;
    }
    // applying them to the muscles
    for (std::size_t muscle = 0; muscle < NB_DOFS * 2; muscle++)
    {
        int index = 2 * NB_DOFS + muscle;
        double l_MTC = d->actuator_length[index];
        double dot_l_MTC = d->actuator_velocity[index];
        double a = d->act[muscle];
        double l_CE = d->act[index];
        std::tuple<double, double> r =
            muscles_[muscle].get(l_MTC, dot_l_MTC, a, l_CE);
        d->ctrl[NB_DOFS * 2 + muscle] = std::get<1>(r);
        // bias_forces_ will be used by the get_bias method below
        bias_forces_[muscle] = std::get<0>(r);
    }
}

template <int QUEUE_SIZE, int NB_DOFS>
mjtNum PressureController<QUEUE_SIZE, NB_DOFS>::get_bias(const mjModel* m,
                                                         const mjData* d,
                                                         int id)
{
    int i_muscle = id - 2 * NB_DOFS;
    return -bias_forces_[i_muscle];  // computed in the apply function
    // note: optional filtering performed in the original code, ignored for now
}

template <int QUEUE_SIZE, int NB_DOFS>
double PressureController<QUEUE_SIZE, NB_DOFS>::PressureController::
    pressure2activation(std::size_t index, double pressure)
{
    double a;
    a = (pressure - scale_min_pressure_[index]);
    a /= (scale_max_pressure_[index] - scale_min_pressure_[index]);
    a *= (scale_max_activation_[index] - scale_min_activation_[index]);
    a += scale_min_activation_[index];
    return a;
}

template <int QUEUE_SIZE, int NB_DOFS>
double PressureController<QUEUE_SIZE, NB_DOFS>::PressureController::
    activation2pressure(std::size_t index, double activation)
{
    double p;
    p = (activation - scale_min_activation_[index]);
    p *= (scale_max_pressure_[index] - scale_min_pressure_[index]);
    p /= (scale_max_activation_[index] - scale_min_activation_[index]);
    p += scale_min_pressure_[index];
    return p;
}

template <int QUEUE_SIZE, int NB_DOFS>
void PressureController<QUEUE_SIZE, NB_DOFS>::clear(std::string segment_id)
{
    o80::clear_shared_memory(segment_id);
}
