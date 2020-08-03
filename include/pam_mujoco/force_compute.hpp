#pragma once


#include <map>
#include <vector>
#include "pam_mujoco/mujoco_base.hpp"

namespace pam_mujoco
{

  class ForceComputeBase;
  typedef std::map<int,ForceComputeBase*> ForceComputeInstances;
  ForceComputeInstances force_compute_instances_g;
  
  
  class ForceComputeBase
  {
  public:
    ForceComputeBase(int force_index)
    {
      force_compute_instances_g.insert(
				    std::pair<int,
				    ForceComputeBase*>(force_index,
						       this) );
    }
    virtual void get_force(const mjModel* m,
			   mjData* d,
			   int id)=0;
  };

  
  template<int FORCE_INDEX>
  virtual void get_force(const mjModel* m,
		       mjData* d)
  {
    static ForceComputeBase* force_compute =
      force_compute_instances_g[FORCE_INDEX];
    force_compute->get_force(m,d);
  }

  

}
