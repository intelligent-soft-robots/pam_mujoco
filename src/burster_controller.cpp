#include "pam_mujoco/burster_controller.hpp"

namespace pam_mujoco
{
BursterController::BursterController(std::string mujoco_id)
    : burster_{mujoco_id}, iteration_{0}
{
}

void BursterController::apply(const mjModel* m, mjData* d)
{
    (void)m;  // avoiding unused argument warning upon compilation
    if (this->must_update(d))
    {
        // The first two calls to the controllers happen during the start of
        // MuJoCo (during mj_loadXML). Do not block during these calls to make
        // sure that the simulation does not get stuck.
        if (iteration_ >= 2)
        {
            burster_.pulse();
        }
        iteration_++;
    }
}

}  // namespace pam_mujoco
