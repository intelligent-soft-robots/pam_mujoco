#include "pam_mujoco/burster_controller.hpp"

namespace pam_mujoco
{
  
BursterController::BursterController(std::string mujoco_id)
    : burster_{mujoco_id}, first_iteration_{true}
{
}

void BursterController::apply(const mjModel* m, mjData* d)
{
    (void)m;  // avoiding unused argument warning upon compilation
    if (this->must_update(d))
    {
      if(!first_iteration_)
	{
	  burster_.pulse();
	}
      // we do not block the first iteration: allows pam_mujoco
      // to start graphics properly.
      first_iteration_ = false;
    }
}

}  // namespace pam_mujoco
