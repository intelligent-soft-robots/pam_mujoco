#pragma once

#include <memory>
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
    virtual void construct(const mjModel* m,
			   const mjData* d)=0;
  };

  class Controllers
  {
  public:
    static void add(std::shared_ptr<ControllerBase> controller);
    static void add(ControllerBase& controller);
    static void apply(const mjModel* m,
		      mjData* d);
    static void construct(const mjModel* m,
			  mjData* d);
  private:
    static std::vector<std::shared_ptr<ControllerBase>> controllers_;
  };
  
}
