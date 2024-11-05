#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <queue>
#include <thread>

#include "mujoco.h"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

class MujocoState
{
public:
    MujocoState()
    {
        robot_positions.fill(0);
        robot_velocities.fill(0);
        ball_positions.fill(0);
        ball_velocities.fill(0);
    }
    double time_stamp;
    std::array<float, 4> robot_positions;
    std::array<float, 4> robot_velocities;
    std::array<float, 3> ball_positions;
    std::array<float, 3> ball_velocities;
};

class MujocoStatePrinter
{
public:
    MujocoStatePrinter(std::string filepath);
    ~MujocoStatePrinter();
    void add(const MujocoState& state);
    void exit();

private:
    void write_to_file();

private:
    std::ofstream m_file_;
    std::queue<MujocoState> m_queue_;
    std::thread m_thread_;
    std::mutex m_mutex_;
    std::condition_variable m_cond_var_;
    bool m_exit_;
};

class MujocoStatePrinterController : public ControllerBase
{
public:
    MujocoStatePrinterController(std::string mujoco_id,
                                 std::string folder,
                                 std::string robot_joint_,
                                 std::string ball_joint);
    void apply(const mjModel* m, mjData* d);

private:
    void init(const mjModel* m);

private:
    MujocoState state_;
    MujocoStatePrinter msp_;
    std::string robot_joint_;
    int qpos_robot_;
    int qvel_robot_;
    std::string ball_joint_;
    int qpos_ball_;
    int qvel_ball_;
};

}  // namespace pam_mujoco
