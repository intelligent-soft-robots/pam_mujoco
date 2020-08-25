#pragma once

#include "o80/burster.hpp"
#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/run_management.hpp"


namespace pam_mujoco
{

  class BurstController : public ControllerBase
  {
  public:
    BurstController(std::string mujoco_id,
		    std::string segment_id);
    void construct(const mjModel* m,
		   const mjData* d);
    void apply(const mjModel* m,
	       mjData* d);
  private:
    o80::Burster burster_;
    std::string mujoco_id_;
  };
  
}
