#pragma once

#include <mujoco.h>
#include <string>

#include <pam_mujoco/controllers.hpp>

namespace pam_mujoco
{
void save_state(const mjModel* model,
                mjData* data,
                const std::string& filename);

void managed_save_state(const mjModel* model,
                        mjData* data,
                        const std::string& filename_prefix);

void load_state(const std::string& filename,
                const mjModel* model,
                mjData* data);

void print_state_file(const std::string& filename);

class SaveStateController : public ControllerBase
{
public:
    SaveStateController(const std::string &mujoco_id) : mujoco_id_(mujoco_id)
    {
    }
    void apply(const mjModel* m, mjData* d)
    {
        managed_save_state(m, d, mujoco_id_);
    }

private:
    std::string mujoco_id_;
};

}  // namespace pam_mujoco
