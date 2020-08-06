#pragma once

#include <map>
#include <vector>
#include "pam_mujoco/mujoco_base.hpp"

namespace pam_mujoco
{

  class ControllerBase
  {
  public:
    virtual void control(const mjModel* m,
			 mjData* d)=0;
  };

  class Controllers
  {
  public:
    static add(ControllerBase& controller)
    {
      Controllers::controllers_.push_back(&controller);
    }
    static std::vector<ControllerBase*> controllers_;
    static void apply(const mjModel* m,
		      mjData* d)
    {
      for(ControllerBase* controller: Controllers::controllers_)
	{
	  controller->control(m,d);
	}
    }
  };
  
  
}
