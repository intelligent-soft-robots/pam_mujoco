#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include "mjmodel.h"
#include "mjdata.h"


namespace pam_mujoco
{

  class ActuatorBiasBase
  {
  public:
    virtual mjtNum get_bias(const mjModel* m,
			    const mjData* d,
			    int id)=0;
  };
  
  class ControllerBase
  {
  public:
    virtual void apply(const mjModel* m,
			 mjData* d)=0;
  };

  class Controllers
  {
  public:
    static void add(std::shared_ptr<ControllerBase> controller);
    static void add(ControllerBase& controller);
    static void add_bias(std::shared_ptr<ActuatorBiasBase> bias);
    static void apply(const mjModel* m,
		      mjData* d);
    static mjtNum get_bias(const mjModel* m,
			   const mjData* d,
			   int id);
  private:
    static std::vector<std::shared_ptr<ControllerBase>> controllers_;
    static std::vector<std::shared_ptr<ActuatorBiasBase>> biases_;
  };
  
}
