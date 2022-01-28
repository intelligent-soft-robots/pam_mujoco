#include <cmath>
#include <iostream>
#include <pam_mujoco/mj_state_tools.hpp>

#include <filesystem>
#include <fstream>
#include <vector>

#include <fmt/format.h>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

namespace pam_mujoco
{
void save_state(const mjModel* model, mjData* data, const std::string& filename)
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

    std::vector<float> ctrl(model->nu);
    mju_n2f(ctrl.data(), data->ctrl, model->nu);

    std::vector<float> qacc(model->nv);
    mju_n2f(qacc.data(), data->qacc, model->nv);

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

void managed_save_state(const mjModel* model,
                        mjData* data,
                        const std::string& filename_prefix)
{
    // in the case under investigation, the nan appears at t=0.8, so only start
    // logging a bit before that
    if (data->time < 0.75 || data->time > 0.85) {
        return;
    }

    // FIXME: This function is rather dirty

    static constexpr size_t N_KEEP = 100;
    static const std::string FILENAME_FMT =
        "./mujoco_state_snapshots/{}{:012d}.dat";

    static size_t index = 0;
    std::string path = fmt::format(FILENAME_FMT, filename_prefix, index);
    index++;

    save_state(model, data, path);

    if (std::isnan(data->qvel[16])) {
        std::cerr << "!!!!! Detected NAN [" << path << "].  Exit." << std::endl;
        std::exit(-1);
    }

    // only keep the newest files, so delete old ones
    if (index >= N_KEEP)
    {
        size_t del_idx = index - N_KEEP;
        path = fmt::format(FILENAME_FMT, filename_prefix, del_idx);
        if (!std::filesystem::remove(path))
        {
            std::cerr << "ERROR: file " << path << " not found.\n";
        }
    }
}

void _assert_vector_length(const std::string& name,
                           size_t vector_length,
                           int expected_length)
{
    if (vector_length != expected_length)
    {
        throw std::runtime_error(
            name + " length in file does not match model (file: " +
            std::to_string(vector_length) +
            ", model: " + std::to_string(expected_length) + ")");
    }
}

void load_state(const std::string& filename, const mjModel* model, mjData* data)
{
    // FIXME: This needs to be updated to the changes in save_state()

    int file_format_version = 0;

    std::ifstream is(filename, std::ios::binary);
    cereal::JSONInputArchive archive(is);

    archive(file_format_version);

    if (file_format_version == 1)
    {
        std::vector<float> qpos, qvel, ctrl, mocap_pos, mocap_quat, sensordata;
        archive(qpos, qvel, ctrl, mocap_pos, mocap_quat, sensordata);

        // check if sizes match
        _assert_vector_length("qpos", qpos.size(), model->nq);
        _assert_vector_length("qvel", qvel.size(), model->nv);
        _assert_vector_length("ctrl", ctrl.size(), model->nu);
        _assert_vector_length("mocap_pos", mocap_pos.size(), 3 * model->nmocap);
        _assert_vector_length(
            "mocap_quat", mocap_quat.size(), 4 * model->nmocap);
        _assert_vector_length(
            "sensordata", sensordata.size(), model->nsensordata);

        mju_f2n(data->qpos, qpos.data(), model->nq);
        mju_f2n(data->qvel, qvel.data(), model->nv);
        mju_f2n(data->ctrl, ctrl.data(), model->nu);
        mju_f2n(data->mocap_pos, mocap_pos.data(), 3 * model->nmocap);
        mju_f2n(data->mocap_quat, mocap_quat.data(), 4 * model->nmocap);
        mju_f2n(data->sensordata, sensordata.data(), model->nsensordata);
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
    int nq, nv, nu, nmocap, nsensordata;

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
}  // namespace pam_mujoco
