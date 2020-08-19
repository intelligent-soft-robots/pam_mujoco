#include "pam_mujoco/burst_controller.hpp"


namespace pam_mujoco
{

  BurstController::BurstController(std::string segment_id)
    : ControllerBase(),
      burster_(segment_id)
  {
    o80::Burster::turn_on(segment_id);
  }

  void BurstController::apply(const mjModel* m,
			      mjData* d)
  {
    burster_.pulse();
  }

}
