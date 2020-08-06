#pragma once

#include <vector>
#include "pam_mujoco/mujoco_base.hpp"

namespace pam_mujoco
{

  class ForceComputeBase
  {
  public:
    virtual void apply(const mjModel* m,
		       mjData* d)=0;
  };

  class ForceComputes
  {
  public:
    static void add(ForceComputeBase& force_compute);
    static void apply(const mjModel* m,
		      mjData* d);
  private:
    static std::vector<ForceComputeBase*> force_computes_;
  };

  
}
