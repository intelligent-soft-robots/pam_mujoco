/**
 * @brief Utilities for loading/saving Mujoco states (mjData).
 */
#pragma once

#include <mujoco.h>
#include <string>

#include <pam_mujoco/controllers.hpp>

namespace pam_mujoco
{
/**
 * @brief Save the simulation state from the given mjData
 *
 * The data is stored in a binary file that can be read by load_state and
 * print_state_file.
 *
 * @param model The model is needed as it contains information about the
 *      dimensions of data.
 * @param data The Mujoco data that is to be saved.
 * @param filename Output filename.
 */
void save_state(const mjModel* model,
                const mjData* data,
                const std::string& filename);

/**
 * @brief Load simulation state from file.
 *
 * Important: Not all fields of data are stored in the file, as they can be
 * computed from others. After loading `mj_forward` or `mj_step` needs to be
 * called to populate the missing values.
 *
 * @param filename Input file.
 * @param model The model that was used when saving the state.
 * @param data Data object to which the state is written.
 */
void load_state(const std::string& filename,
                const mjModel* model,
                mjData* data);

/**
 * @brief Load a Mujoco state file and print its content in JSON format.
 *
 * @param filename Path to the data file.
 */
void print_state_file(const std::string& filename);

/**
 * @brief Save Mujoco states, keeping only the last N states.
 *
 * Every time save() is called, a new file is created with the name
 * "<filename_prefix>############.dat" where the # correspond to an increasing
 * number.
 *
 * To keep the amount of generated files under control, only the last N files
 * are kept, so when saving the N+1-th file, the oldest is deleted.
 */
class MujocoStateSaver
{
public:
    /**
     * @brief
     *
     * @param filename_prefix  Prefix for the names of the state files.
     * @param num_keep_files  Number of state files to keep.  If this number is
     *      reached, every further call of save() results in the oldest existing
     *      file being deleted.
     */
    MujocoStateSaver(const std::string& filename_prefix,
                     size_t num_keep_files = 100)
        : num_keep_files_(num_keep_files), filename_prefix_(filename_prefix)
    {
    }

    /**
     * @brief Save the given Mujoco data to file.
     *
     * @param model  The model that is used (needed as it contains information
     *      about the dimensions of the data).
     * @param data The Mujoco data that is to be saved.
     */
    void save(const mjModel* model, const mjData* data);

private:
    size_t index_ = 0;
    size_t num_keep_files_;
    std::string filename_prefix_;
};

/**
 * @brief A wrapper around MujocoStateSaver that can be added as a "Controller".
 */
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
