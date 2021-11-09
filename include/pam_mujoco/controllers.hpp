#pragma once

#include <iostream>
#include <memory>
#include <vector>

// clang-format off
// NOTE: mjmodel.h MUST be included before mjdata.h.
#include "mjmodel.h"
#include "mjdata.h"
// clang-format on

#include "o80/time.hpp"

namespace pam_mujoco
{

class ActuatorBiasBase
{
public:
    virtual mjtNum get_bias(const mjModel* m, const mjData* d, int id) = 0;
};

class ControllerBase
{
public:
    ControllerBase();
    virtual void apply(const mjModel* m, mjData* d) = 0;
    bool must_update(mjData* d);
    const o80::TimePoint& get_time_stamp();
    void reset_time();

private:
    o80::Milliseconds mujoco_time_step_;
    o80::TimePoint previous_stamp_;

private:
    static const int MUJOCO_TIME_STEP_MS = 2;
};

class Controllers
{
public:
    static void add(std::shared_ptr<ControllerBase> controller);
    static void add(ControllerBase& controller);
    static void add_bias(std::shared_ptr<ActuatorBiasBase> bias);
    static void reset_time();
    static void apply(const mjModel* m, mjData* d);
    static mjtNum get_bias(const mjModel* m, const mjData* d, int id);

private:
    static std::vector<std::shared_ptr<ControllerBase>> controllers_;
    static std::vector<std::shared_ptr<ActuatorBiasBase>> biases_;
};

}  // namespace pam_mujoco
