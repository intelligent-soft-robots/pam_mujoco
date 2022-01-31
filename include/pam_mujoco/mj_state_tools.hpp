#pragma once

#include <mujoco.h>
#include <string>

#include <pam_mujoco/controllers.hpp>

namespace pam_mujoco
{
void save_state(const mjModel* model,
                const mjData* data,
                const std::string& filename);

void load_state(const std::string& filename,
                const mjModel* model,
                mjData* data);

void print_state_file(const std::string& filename);

class MujocoStateSaver
{
public:
    MujocoStateSaver(const std::string& filename_prefix,
                     size_t num_keep_files = 100)
        : num_keep_files_(num_keep_files), filename_prefix_(filename_prefix)
    {
    }

    void save(const mjModel* model, const mjData* data);

private:
    size_t index_ = 0;
    size_t num_keep_files_;
    std::string filename_prefix_;
};

class SaveStateController : public ControllerBase, public MujocoStateSaver
{
public:
    SaveStateController(const std::string& mujoco_id)
        : MujocoStateSaver(mujoco_id)
    {
    }
    virtual void apply(const mjModel* m, mjData* d)
    {
        save(m, d);
    }

private:
    std::string mujoco_id_;
};

}  // namespace pam_mujoco
