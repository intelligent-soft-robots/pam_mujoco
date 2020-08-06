#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  void Controllers::add(ControllerBase& controller)
  {
    Controllers::controllers_.push_back(&controller);
  }

  void Controllers::apply(const mjModel* m,
	     mjData* d)
  {
    for(ControllerBase* controller: Controllers::controllers_)
      {
	controller->apply(m,d);
      }
  }
  
}
