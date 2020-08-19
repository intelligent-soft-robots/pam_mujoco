#pragma once

#include "pam_mujoco/controllers.hpp"
#include "o80/burster.hpp"

namespace pam_mujoco
{

  class BurstController : public ControllerBase
  {
  public:
    BurstController(std::string segment_id);
    void apply(const mjModel* m,
	       mjData* d);
  private:
    o80::Burster burster_;
  };
  
}
