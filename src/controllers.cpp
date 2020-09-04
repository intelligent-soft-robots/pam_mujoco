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

  void Controllers::add_bias(std::shared_ptr<ActuatorBiasBase> bias)
  {
    Controllers::biases_.push_back(bias);
  }
  
  void Controllers::apply(const mjModel* m,
			  mjData* d)
  {
    for(std::shared_ptr<ControllerBase> controller: Controllers::controllers_)
      {
	controller->apply(m,d);
      }
  }


  std::vector<std::shared_ptr<ControllerBase>> Controllers::controllers_;
  std::vector<std::shared_ptr<ActuatorBiasBase>> biases_;
  
}
