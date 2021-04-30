#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{
const int ControllerBase::MUJOCO_TIME_STEP_MS;

ControllerBase::ControllerBase()
    : mujoco_time_step_{ControllerBase::MUJOCO_TIME_STEP_MS},
      previous_stamp_{-1},
      first_iteration_{true}
{
}

bool ControllerBase::must_update(mjData* d)
{
    if (first_iteration_)
    {
        first_iteration_ = false;
        return true;
    }
    o80::TimePoint time_stamp{static_cast<long int>(d->time * 1e9)};
    if (time_stamp - previous_stamp_ >= mujoco_time_step_)
    {
        previous_stamp_ = time_stamp;
        return true;
    }
    return false;
}

const o80::TimePoint& ControllerBase::get_time_stamp()
{
    return previous_stamp_;
}

void Controllers::add(ControllerBase& controller)
{
    Controllers::controllers_.push_back(
        std::shared_ptr<ControllerBase>(&controller));
}

void Controllers::add(std::shared_ptr<ControllerBase> controller)
{
    Controllers::controllers_.push_back(controller);
}

void Controllers::add_bias(std::shared_ptr<ActuatorBiasBase> bias)
{
    Controllers::biases_.push_back(bias);
}

void Controllers::apply(const mjModel* m, mjData* d)
{
    for (std::shared_ptr<ControllerBase> controller : Controllers::controllers_)
    {
        controller->apply(m, d);
    }
}

mjtNum Controllers::get_bias(const mjModel* m, const mjData* d, int id)
{
    mjtNum b = 0;
    for (std::shared_ptr<ActuatorBiasBase> bias : Controllers::biases_)
    {
        b += bias->get_bias(m, d, id);
    }
    return b;
}

std::vector<std::shared_ptr<ControllerBase>> Controllers::controllers_;
std::vector<std::shared_ptr<ActuatorBiasBase>> Controllers::biases_;

}  // namespace pam_mujoco
