#include "pam_mujoco/burst_controller.hpp"

namespace pam_mujoco
{
BurstController::BurstController(std::string mujoco_id, std::string segment_id)
    : ControllerBase(),
      burster_(segment_id),
      mujoco_id_(mujoco_id),
      count_{0},
      break_{true}
{
}

void BurstController::apply(const mjModel* m, mjData* d)
{
    if (this->must_update(d))
    {
        if (!is_stop_requested(mujoco_id_))
        {
            break_ = true;
            // std::cout << "->bursting in : " << count_ << "\n";
            burster_.pulse();
            // std::cout << "<-bursting out\n";
            count_++;
        }
    }
    else
    {
        if (break_)
        {
            // std::cout << ".\n";
            break_ = false;
        }
    }
}

}  // namespace pam_mujoco
