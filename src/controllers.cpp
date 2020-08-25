#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  void Controllers::add(ControllerBase& controller)
  {
    Controllers::controllers_.push_back(std::shared_ptr<ControllerBase>(&controller));
  }

  void Controllers::add(std::shared_ptr<ControllerBase> controller)
  {
    Controllers::controllers_.push_back(controller);
  }
  
  void Controllers::apply(const mjModel* m,
			  mjData* d)
  {
    for(std::shared_ptr<ControllerBase> controller: Controllers::controllers_)
      {
	controller->apply(m,d);
      }
  }


  void Controllers::construct(const mjModel* m,
			      mjData* d)
  {
    for(std::shared_ptr<ControllerBase> controller: Controllers::controllers_)
      {
	controller->construct(m,d);
      }
  }
  
  std::vector<std::shared_ptr<ControllerBase>> Controllers::controllers_;
  
}
