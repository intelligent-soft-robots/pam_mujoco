#pragma once


#include <map>
#include <vector>
#include "pam_mujoco/mujoco_base.hpp"

namespace pam_mujoco
{

  class ForceComputeBase
  {
  public:
    virtual void get_force(const mjModel* m,
			   mjData* d);
  };

  class ForceComputes
  {
  public:
    static void add(ForceComputeBase& controller);
    static void apply(const mjModel* m,
		      mjData* d);
  private:
    static std::vector<ForceComputeBase*> force_computes_;
  };

  
}
