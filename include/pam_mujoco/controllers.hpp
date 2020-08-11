#pragma once

#include <vector>
#include "mjmodel.h"
#include "mjdata.h"


namespace pam_mujoco
{

  class ControllerBase
  {
  public:
    virtual void apply(const mjModel* m,
			 mjData* d)=0;
  };

  class Controllers
  {
  public:
    static void add(ControllerBase& controller);
    static void apply(const mjModel* m,
		      mjData* d);
  private:
    static std::vector<ControllerBase*> controllers_;
  };
  
}
