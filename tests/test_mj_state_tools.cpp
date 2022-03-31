#include <stdlib.h>
#include <filesystem>
#include <fstream>
#include <limits>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <mujoco.h>

#include <pam_mujoco/mj_state_tools.hpp>

using namespace pam_mujoco;

class TestMjStateTools : public ::testing::Test
{
protected:
    mjModel *model_;
    mjData *data_;

    //! Temporary directory in which files can be saved
    std::filesystem::path tmp_dir_;

    void SetUp() override
    {
        char tmpdir_name[] = "/tmp/test_mj_state_toolsXXXXXXX";
        ASSERT_NE(mkdtemp(tmpdir_name), nullptr);
        tmp_dir_ = tmpdir_name;

        // create a temporary model file so the model can be loaded
        std::string model_file = tmp_dir_ / "model.xml";
        std::ofstream f(model_file);
        // Some nonsensical minimal model for initialising model_ and data_
        // such that all relevant fields of data_ are non-empty.
        f << R"(
<mujoco model="pendulum">

  <size nuserdata="3" />

  <worldbody>
    <body name="pole">
      <joint name="j1" type="hinge" axis="1 0 0" />
      <joint name="j2" type="hinge" axis="0 1 0" />
      <geom name="mass" type="sphere" size="1" mass="1" />
      <site name="end1" pos="0.5 0 0.6" type="sphere" size="0.01"/>
    </body>
    <body name="mocap_body" mocap="true">
      <geom name="mocap_geom" type="sphere" size="1" mass="1" />
      <site name="end2" pos="0.5 0 0.2" type="sphere" size="0.01"/>
    </body>
  </worldbody>

  <tendon>
    <spatial limited="true" range="0 0.6" width="0.005">
      <site site="end1"/>
      <site site="end2"/>
    </spatial>
  </tendon>

  <actuator>
    <motor name="act_motor" joint="j1" ctrlrange="-1 1" />
    <muscle name="act_muscle" joint="j2" ctrlrange="-1 1" />
  </actuator>
</mujoco>
            )";
        f.close();

        // load model and initialise data
        char error[500] = "";
        mj_activate("/opt/mujoco/mjkey.txt");
        model_ = mj_loadXML(model_file.c_str(), NULL, error, 500);
        if (!model_)
        {
            FAIL() << error;
        }
        data_ = mj_makeData(model_);

        // before writing to data_, verify that all its fields have the expected
        // length
        ASSERT_EQ(model_->nq, 2);
        ASSERT_EQ(model_->nv, 2);
        ASSERT_EQ(model_->na, 1);
        ASSERT_EQ(model_->nmocap, 1);
        ASSERT_EQ(model_->nuserdata, 3);
        ASSERT_EQ(model_->nu, 2);

        // fill data with some values
        data_->time = 12345;
        data_->qpos[0] = 1;
        data_->qpos[1] = 2;
        data_->qvel[0] = 1.1;
        data_->qvel[1] = 1.2;
        data_->act[0] = -1;
        data_->mocap_pos[0] = 2.0;
        data_->mocap_pos[1] = 2.2;
        data_->mocap_pos[2] = 2.4;
        data_->mocap_quat[0] = 0.1;
        data_->mocap_quat[1] = 0.2;
        data_->mocap_quat[2] = 0.3;
        data_->mocap_quat[3] = 0.4;
        data_->userdata[0] = 13;
        data_->userdata[1] = 23;
        data_->userdata[2] = 42;
        data_->ctrl[0] = -0.4;
        data_->ctrl[1] = -0.8;
    }

    void TearDown() override
    {
        std::filesystem::remove_all(tmp_dir_);

        mj_deleteData(data_);
        mj_deleteModel(model_);
    }
};

void expect_data_eq(mjData *d1, mjData *d2)
{
    EXPECT_FLOAT_EQ(d1->time, d2->time);
    EXPECT_FLOAT_EQ(d1->qpos[0], d2->qpos[0]);
    EXPECT_FLOAT_EQ(d1->qpos[1], d2->qpos[1]);
    EXPECT_FLOAT_EQ(d1->qvel[0], d2->qvel[0]);
    EXPECT_FLOAT_EQ(d1->qvel[1], d2->qvel[1]);
    EXPECT_FLOAT_EQ(d1->act[0], d2->act[0]);
    EXPECT_FLOAT_EQ(d1->mocap_pos[0], d2->mocap_pos[0]);
    EXPECT_FLOAT_EQ(d1->mocap_pos[1], d2->mocap_pos[1]);
    EXPECT_FLOAT_EQ(d1->mocap_pos[2], d2->mocap_pos[2]);
    EXPECT_FLOAT_EQ(d1->mocap_quat[0], d2->mocap_quat[0]);
    EXPECT_FLOAT_EQ(d1->mocap_quat[1], d2->mocap_quat[1]);
    EXPECT_FLOAT_EQ(d1->mocap_quat[2], d2->mocap_quat[2]);
    EXPECT_FLOAT_EQ(d1->mocap_quat[3], d2->mocap_quat[3]);
    EXPECT_FLOAT_EQ(d1->userdata[0], d2->userdata[0]);
    EXPECT_FLOAT_EQ(d1->userdata[1], d2->userdata[1]);
    EXPECT_FLOAT_EQ(d1->userdata[2], d2->userdata[2]);
    EXPECT_FLOAT_EQ(d1->ctrl[0], d2->ctrl[0]);
    EXPECT_FLOAT_EQ(d1->ctrl[1], d2->ctrl[1]);
}

TEST_F(TestMjStateTools, test_copy_data)
{
    // first the data object needs to be initialised based on the model
    mjData *data_cpy = mj_makeData(model_);

    copy_data(model_, data_, data_cpy);
    expect_data_eq(data_cpy, data_);

    // cleanup
    mj_deleteData(data_cpy);
}

TEST_F(TestMjStateTools, test_save_and_load)
{
    save_state(model_, data_, tmp_dir_ / "data.dat");

    // first the data object needs to be initialised based on the model
    mjData *loaded_data = mj_makeData(model_);

    load_state(tmp_dir_ / "data.dat", model_, loaded_data);

    expect_data_eq(loaded_data, data_);

    // cleanup
    mj_deleteData(loaded_data);
}

TEST_F(TestMjStateTools, test_has_nan_none)
{
    ASSERT_FALSE(has_nan(model_, data_));
}

TEST_F(TestMjStateTools, test_has_nan_in_qpos)
{
    data_->qpos[0] = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(has_nan(model_, data_));
}

TEST_F(TestMjStateTools, test_has_nan_in_qvel)
{
    data_->qvel[0] = std::numeric_limits<float>::quiet_NaN();
    ASSERT_TRUE(has_nan(model_, data_));
}

TEST_F(TestMjStateTools, test_mujoco_state_saver)
{
    const std::string prefix = (tmp_dir_ / "test").string();
    const size_t num_keep_files = 3;
    MujocoStateSaver state_saver(prefix, num_keep_files);

    data_->time = 0;
    state_saver.save(model_, data_);

    // verify files gets saved
    std::string filename = fmt::format(state_saver.FILENAME_FMT_, prefix, 0);
    ASSERT_TRUE(std::filesystem::exists(filename));

    // save more states with increasing time stamps
    data_->time = 1;
    state_saver.save(model_, data_);
    data_->time = 2;
    state_saver.save(model_, data_);
    data_->time = 3;
    state_saver.save(model_, data_);

    // since only 3 files should be kept, the first one should be deleted now
    filename = fmt::format(state_saver.FILENAME_FMT_, prefix, 0);
    EXPECT_FALSE(std::filesystem::exists(filename));

    // of the four files, only the last three should be kept
    for (int i = 1; i <= 3; ++i)
    {
        filename = fmt::format(state_saver.FILENAME_FMT_, prefix, i);
        ASSERT_TRUE(std::filesystem::is_regular_file(filename));

        // load the file and check the time stamp
        mjData *loaded_data = mj_makeData(model_);
        load_state(filename, model_, loaded_data);
        EXPECT_EQ(loaded_data->time, i);

        // cleanup
        mj_deleteData(loaded_data);
    }
}

TEST_F(TestMjStateTools, test_save_nan_state_controller)
{
    std::string prefix = tmp_dir_ / "test";
    SaveNaNStateController ctrl(prefix);

    // Make sure the buffer size is as expected (in case it changes, the test
    // likely needs to be adapted).
    ASSERT_EQ(SaveNaNStateController::BUFFER_SIZE_, 5u);

    // add a number of "good" states
    for (int i = 0; i < 7; i++)
    {
        data_->time = i;
        ctrl.apply(model_, data_);
    }
    // add a state with NaN
    data_->time++;
    data_->qpos[0] = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(ctrl.apply(model_, data_), NaNInMujocoDataError);

    // check if files were saved
    const int time_of_first_snapshot = 3;
    for (size_t i = 0; i < ctrl.BUFFER_SIZE_; ++i)
    {
        std::string filename = fmt::format(ctrl.FILENAME_FMT_, prefix, i);
        ASSERT_TRUE(std::filesystem::is_regular_file(filename));

        // load the file and check the time stamp
        mjData *loaded_data = mj_makeData(model_);
        load_state(filename, model_, loaded_data);
        EXPECT_EQ(loaded_data->time, time_of_first_snapshot + i);
        mj_deleteData(loaded_data);
    }
}
