#pragma once

#include "o80/burster.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{
class BursterController : public ControllerBase
{
public:
    BursterController(std::string mujoco_id);
    void apply(const mjModel* m, mjData* d);

private:
    o80::Burster burster_;
    long int iteration_;
};

}  // namespace pam_mujoco
