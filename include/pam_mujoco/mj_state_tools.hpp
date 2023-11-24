/**
 * @brief Utilities for loading/saving Mujoco states (mjData).
 */
#pragma once

#include <filesystem>
#include <string>

#include <mujoco.h>

#include <pam_mujoco/controllers.hpp>
#include <pam_mujoco/ring_buffer.hpp>

namespace pam_mujoco
{
/**
 * @brief Copy Mujoco simulation state.
 *
 * Copies the relevant fields of ``from`` to ``to`` (leaving out fields
 * that do not affect the simulation, see
 * https://mujoco.readthedocs.io/en/latest/programming.html#sistatecontrol).
 *
 * @param model The model is needed as it contains information about the
 *      dimensions of data.
 * @param from Mujoco data from which to copy.
 * @param to   Mujoco data to which to copy.
 */
void copy_mjdata(const mjModel* model, const mjData* from, mjData* to);

/**
 * @brief Save the simulation state from the given mjData
 *
 * The data is stored in a binary file that can be read by load_mjdata and
 * print_mjdata_file.
 *
 * @param model The model is needed as it contains information about the
 *      dimensions of data.
 * @param data The Mujoco data that is to be saved.
 * @param filename Output filename.
 */
void save_mjdata(const mjModel* model,
                 const mjData* data,
                 const std::filesystem::path& filename);

/**
 * @brief Load simulation state from file to an mjData instance.
 *
 * @verbatim embed:rst:leading-asterisk
 *
 * .. important::
 *
 *    ``data`` needs to be initialised (using ``mj_makeData()``) **before**
 *    calling this function.
 *
 * Not all fields of data are stored in the file, as they can be computed from
 * others. After loading ``mj_forward()`` or ``mj_step()`` needs to be called to
 * populate the missing values.
 *
 * @endverbatim
 *
 * @param filename Input file.
 * @param model The model that was used when saving the state.
 * @param data Data object to which the state is written.
 */
void load_mjdata(const std::filesystem::path& filename,
                 const mjModel* model,
                 mjData* data);

/**
 * @brief Load a Mujoco data file and print its content in JSON format.
 *
 * Expects a file that was created with @ref save_mjdata().
 *
 * @param filename Path to the data file.
 */
void print_mjdata_file(const std::filesystem::path& filename);

/**
 * @brief Check if the given Mujoco data has NaN values in relevant fields.
 *
 * The following parts of the data are checked:
 * - qpos
 * - qvel
 *
 * @param model  The model that is used (needed as it contains information
 *      about the dimensions of the data).
 * @param data The Mujoco data that is checked for NaN values.
 * @returns True if one or more NaN values are found.
 */
bool mjdata_has_nan(const mjModel* model, const mjData* data);

/**
 * @brief Exception indicating that a NaN value was detected in the Mujoco data.
 */
class NaNInMujocoDataError : public std::exception
{
};

/**
 * @brief Save Mujoco states (mjData), keeping only the last N states.
 *
 * Every time save() is called, a new file is created with the name
 * "<filename_prefix>############.dat" where the # correspond to an increasing
 * number.
 *
 * To keep the amount of generated files under control, only the last N files
 * are kept, so when saving the N+1-th file, the oldest is deleted.
 */
class MujocoDataSaver
{
public:
    const std::string FILENAME_FMT_ = "{}{:012d}.dat";

    /**
     * @brief
     *
     * @param filename_prefix  Prefix for the names of the state files.
     * @param num_keep_files  Number of state files to keep.  If this number is
     *      reached, every further call of save() results in the oldest existing
     *      file being deleted.
     */
    MujocoDataSaver(const std::string& filename_prefix,
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
 * @brief A wrapper around MujocoDataSaver that can be added as a "Controller".
 */
class SaveMujocoDataController : public ControllerBase, public MujocoDataSaver
{
public:
    SaveMujocoDataController(const std::string& mujoco_id)
        : MujocoDataSaver(mujoco_id, 10000)
    {
    }
    virtual void apply(const mjModel* m, mjData* d)
    {
        if (d->time - last_save_time_ < 0.01)
            return;
        last_save_time_ += 0.01;
        save(m, d);
        printf("save mjdata to file, time: %f\n", d->time);
    }

private:
    std::string mujoco_id_;
    double last_save_time_ = 0;
};

/**
 * @brief A "Controller" that monitors the Mujoco state and saves in case of
 *  NaN.
 *
 * Keeps copies of the Mujoco states of the last view time steps and writes them
 * to files in case a NaN value is observed.
 */
class SaveNanMujocoDataController : public ControllerBase,
                                    public MujocoDataSaver
{
public:
    static constexpr std::size_t BUFFER_SIZE_ = 5;

    /**
     * @param filename_prefix   Prefix for the files if they are written (can
     *  contain a path).
     */
    SaveNanMujocoDataController(const std::string& filename_prefix);

    ~SaveNanMujocoDataController();

    /**
     * @brief Add current data to buffer and check for NaN values.
     *
     * Copies the current data to the buffer (overwriting the oldest entry in
     * the buffer if it is full).
     * Checks if there are NaN values in relevant fields of the data (@see
     * has_nan).  If yes, all data instances from the buffer are saved to files
     * and a NaNInMujocoDataError is thrown.
     *
     * @param model
     * @param data
     * @throw NaNInMujocoDataError if NaN value is detected.
     */
    void apply(const mjModel* model, mjData* data) override;

private:
    RingBuffer<mjData*, BUFFER_SIZE_> buffer_;
};

}  // namespace pam_mujoco
