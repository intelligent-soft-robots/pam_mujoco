#include "pam_mujoco/burst_controller.hpp"


namespace pam_mujoco
{

  BurstController::BurstController(std::string mujoco_id,
				   std::string segment_id)
    : ControllerBase(),
      burster_(segment_id),
      mujoco_id_(mujoco_id)
  {
  }

  void BurstController::apply(const mjModel* m,
			      mjData* d)
  {
    if(this->must_update(d))
      {
	if(!is_stop_requested(mujoco_id_))
	  {
	    burster_.pulse();
	  }
      }
  }

}
