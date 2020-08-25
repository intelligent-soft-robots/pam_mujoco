#include "pam_mujoco/burst_controller.hpp"


namespace pam_mujoco
{

  BurstController::BurstController(std::string mujoco_id,
				   std::string segment_id)
    : ControllerBase(),
      burster_(segment_id),
      mujoco_id_(mujoco_id)
  {
    o80::Burster::turn_on(segment_id);
  }

  void BurstController::construct(const mjModel* m,
				  const mjData* d)
  {}

  void BurstController::apply(const mjModel* m,
			      mjData* d)
  {
    if(!is_stop_requested(mujoco_id_))
      {
	burster_.pulse();
      }
  }

}
