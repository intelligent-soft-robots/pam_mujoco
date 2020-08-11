#include "pam_mujoco/force_computes.hpp"

namespace pam_mujoco
{

  void ForceComputes::add(ForceComputeBase& force_compute)
  {
    ForceComputes::force_computes_.push_back(&force_compute);
  }

  void ForceComputes::apply(const mjModel* m,
	     mjData* d)
  {
    for(ForceComputeBase* force_compute: ForceComputes::force_computes_)
      {
	force_compute->apply(m,d);
      }
  }

  std::vector<ForceComputeBase*> ForceComputes::force_computes_;
  
}
