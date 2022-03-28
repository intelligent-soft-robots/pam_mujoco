#include <pam_mujoco/mj_state_tools.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <fmt/format.h>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

namespace pam_mujoco
{
void copy_data(const mjModel* model, const mjData* from, mjData* to)
{
    // Based on
    // https://mujoco.readthedocs.io/en/latest/programming.html#sistatecontrol

    // copy simulation state
    to->time = from->time;
    mju_copy(to->qpos, from->qpos, model->nq);
    mju_copy(to->qvel, from->qvel, model->nv);
    mju_copy(to->act, from->act, model->na);

    // copy mocap body pose and userdata
    mju_copy(to->mocap_pos, from->mocap_pos, 3 * model->nmocap);
    mju_copy(to->mocap_quat, from->mocap_quat, 4 * model->nmocap);
    mju_copy(to->userdata, from->userdata, model->nuserdata);

    // copy warm-start acceleration
    mju_copy(to->qacc_warmstart, from->qacc_warmstart, model->nv);

    // the following are not necessary for the state to be complete but copy
    // them anyway to be consistent with save_state
    mju_copy(to->qacc, from->qacc, model->nv);
    mju_copy(to->ctrl, from->ctrl, model->nu);
}

void save_state(const mjModel* model,
                const mjData* data,
                const std::string& filename)
{
    // The following code snippet from the documentation shows what field needs
    // to be copied to copy a state from mjData src to dst.
    // The remaining fields can be left empty and are supposed to be filled by
    // calling mj_forward or mj_step.
    //
    // https://mujoco.readthedocs.io/en/latest/programming.html#sistatecontrol
    //
    //    // copy simulation state
    //    dst->time = src->time;
    //    mju_copy(dst->qpos, src->qpos, m->nq);
    //    mju_copy(dst->qvel, src->qvel, m->nv);
    //    mju_copy(dst->act,  src->act,  m->na);
    //
    //    // copy mocap body pose and userdata
    //    mju_copy(dst->mocap_pos,  src->mocap_pos,  3*m->nmocap);
    //    mju_copy(dst->mocap_quat, src->mocap_quat, 4*m->nmocap);
    //    mju_copy(dst->userdata, src->userdata, m->nuserdata);
    //
    //    // copy warm-start acceleration
    //    mju_copy(dst->qacc_warmstart, src->qacc_warmstart, m->nv);

    constexpr int file_format_version = 1;

    float time;
    mju_n2f(&time, &data->time, 1);

    std::vector<float> qpos(model->nq);
    mju_n2f(qpos.data(), data->qpos, model->nq);

    std::vector<float> qvel(model->nv);
    mju_n2f(qvel.data(), data->qvel, model->nv);

    std::vector<float> act(model->na);
    mju_n2f(act.data(), data->act, model->na);

    std::vector<float> mocap_pos(3 * model->nmocap);
    mju_n2f(mocap_pos.data(), data->mocap_pos, 3 * model->nmocap);

    std::vector<float> mocap_quat(4 * model->nmocap);
    mju_n2f(mocap_quat.data(), data->mocap_quat, 4 * model->nmocap);

    std::vector<float> userdata(model->nuserdata);
    mju_n2f(userdata.data(), data->userdata, model->nuserdata);

    std::vector<float> qacc_warmstart(model->nv);
    mju_n2f(qacc_warmstart.data(), data->qacc_warmstart, model->nv);

    // the following fields are not needed to restore the state (see comment
    // above) but may be useful to include for debugging.

    std::vector<float> qacc(model->nv);
    mju_n2f(qacc.data(), data->qacc, model->nv);

    std::vector<float> ctrl(model->nu);
    mju_n2f(ctrl.data(), data->ctrl, model->nu);

    std::ofstream os(filename, std::ios::binary);
    // cereal::JSONOutputArchive archive(os);
    cereal::BinaryOutputArchive archive(os);

    // header info first (for validation when loading)
    archive(CEREAL_NVP(file_format_version));

    // then the actual data
    archive(CEREAL_NVP(time),
            CEREAL_NVP(qpos),
            CEREAL_NVP(qvel),
            CEREAL_NVP(act),
            CEREAL_NVP(mocap_pos),
            CEREAL_NVP(mocap_quat),
            CEREAL_NVP(userdata),
            CEREAL_NVP(qacc_warmstart),
            CEREAL_NVP(qacc),
            CEREAL_NVP(ctrl));
}

void _assert_vector_length(const std::string& name,
                           size_t vector_length,
                           int expected_length)
{
    if (vector_length != static_cast<size_t>(expected_length))
    {
        throw std::runtime_error(
            name + " length in file does not match model (file: " +
            std::to_string(vector_length) +
            ", model: " + std::to_string(expected_length) + ")");
    }
}

void load_state(const std::string& filename, const mjModel* model, mjData* data)
{
    int file_format_version = 0;

    std::ifstream is(filename, std::ios::binary);

    if (!is)
    {
        throw std::runtime_error("Failed to read file " + filename);
    }

    // cereal::JSONInputArchive archive(is);
    cereal::BinaryInputArchive archive(is);

    archive(file_format_version);

    if (file_format_version == 1)
    {
        float time;
        std::vector<float> qpos, qvel, act, mocap_pos, mocap_quat, userdata,
            qacc_warmstart, qacc, ctrl;
        archive(time,
                qpos,
                qvel,
                act,
                mocap_pos,
                mocap_quat,
                userdata,
                qacc_warmstart,
                qacc,
                ctrl);

        // check if sizes match
        _assert_vector_length("qpos", qpos.size(), model->nq);
        _assert_vector_length("qvel", qvel.size(), model->nv);
        _assert_vector_length("act", act.size(), model->na);
        _assert_vector_length("mocap_pos", mocap_pos.size(), 3 * model->nmocap);
        _assert_vector_length(
            "mocap_quat", mocap_quat.size(), 4 * model->nmocap);
        _assert_vector_length("userdata", userdata.size(), model->nuserdata);
        _assert_vector_length(
            "qacc_warmstart", qacc_warmstart.size(), model->nv);
        _assert_vector_length("qacc", qacc.size(), model->nv);
        _assert_vector_length("ctrl", ctrl.size(), model->nu);

        data->time = time;
        mju_f2n(data->qpos, qpos.data(), model->nq);
        mju_f2n(data->qvel, qvel.data(), model->nv);
        mju_f2n(data->act, act.data(), model->na);
        mju_f2n(data->mocap_pos, mocap_pos.data(), 3 * model->nmocap);
        mju_f2n(data->mocap_quat, mocap_quat.data(), 4 * model->nmocap);
        mju_f2n(data->userdata, userdata.data(), model->nuserdata);
        mju_f2n(data->qacc_warmstart, qacc_warmstart.data(), model->nv);
        mju_f2n(data->qacc, qacc.data(), model->nv);
        mju_f2n(data->ctrl, ctrl.data(), model->nu);
    }
    else
    {
        throw std::runtime_error("Invalid file format version " +
                                 std::to_string(file_format_version));
    }
}

void print_state_file(const std::string& filename)
{
    int file_format_version = 0;

    std::ifstream is(filename, std::ios::binary);
    if (is.fail())
    {
        throw std::runtime_error("Failed to open file " + filename);
    }

    cereal::BinaryInputArchive in_archive(is);
    cereal::JSONOutputArchive out_archive(std::cout);

    in_archive(file_format_version);

    if (file_format_version == 1)
    {
        float time;
        std::vector<float> qpos, qvel, act, mocap_pos, mocap_quat, userdata,
            qacc_warmstart, qacc, ctrl;

        in_archive(time,
                   qpos,
                   qvel,
                   act,
                   mocap_pos,
                   mocap_quat,
                   userdata,
                   qacc_warmstart,
                   qacc,
                   ctrl);

        out_archive(CEREAL_NVP(time),
                    CEREAL_NVP(qpos),
                    CEREAL_NVP(qvel),
                    CEREAL_NVP(act),
                    CEREAL_NVP(mocap_pos),
                    CEREAL_NVP(mocap_quat),
                    CEREAL_NVP(userdata),
                    CEREAL_NVP(qacc_warmstart),
                    CEREAL_NVP(qacc),
                    CEREAL_NVP(ctrl));
    }
    else
    {
        throw std::runtime_error("Invalid file format version " +
                                 std::to_string(file_format_version));
    }
}

bool _has_nan(const mjtNum* array, size_t length)
{
    for (size_t i = 0; i < length; ++i)
    {
        if (std::isnan(array[i]))
        {
            return true;
        }
    }
    return false;
}

bool has_nan(const mjModel* model, const mjData* data)
{
    return _has_nan(data->qpos, model->nq) or _has_nan(data->qvel, model->nv);
}

void MujocoStateSaver::save(const mjModel* model, const mjData* data)
{
    static const std::string FILENAME_FMT = "{}{:012d}.dat";

    std::string path = fmt::format(FILENAME_FMT, filename_prefix_, index_);

    save_state(model, data, path);

    // only keep the newest files, so delete old ones
    if (index_ >= num_keep_files_)
    {
        size_t del_idx = index_ - num_keep_files_;
        path = fmt::format(FILENAME_FMT, filename_prefix_, del_idx);
        if (!std::filesystem::remove(path))
        {
            std::cerr << "ERROR: file " << path << " not found.\n";
        }
    }

    index_++;
}

void SaveNaNStateController::apply(const mjModel* model, mjData* data)
{
    copy_data(model, data, &buffer_.current());

    if (has_nan(model, data))
    {
        // save all states from the buffer
        for (const mjData& d : buffer_.get_all())
        {
            save(model, &d);
        }

        // stop the program immediately in case of NaN
        std::cerr << "!!!!! Detected NaN. Save snapshots and exit."
                  << std::endl;
        std::exit(-1);
    }

    // only move on after potential save
    buffer_.next();
}
}  // namespace pam_mujoco
