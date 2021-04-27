#pragma once

#include <iostream>
#include <memory>
#include <vector>
#include "mjdata.h"
#include "mjmodel.h"
#include "shared_memory/shared_memory.hpp"

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

protected:
    bool reset_requested(const std::string& segment_id) const;

private:
    o80::Milliseconds mujoco_time_step_;
    o80::TimePoint previous_stamp_;
    bool first_iteration_;

private:
    static const int MUJOCO_TIME_STEP_MS = 2;
};

class Controllers
{
public:
    static void add(std::shared_ptr<ControllerBase> controller);
    static void add(ControllerBase& controller);
    static void add_bias(std::shared_ptr<ActuatorBiasBase> bias);
    static void apply(const mjModel* m, mjData* d);
    static mjtNum get_bias(const mjModel* m, const mjData* d, int id);

private:
    static std::vector<std::shared_ptr<ControllerBase>> controllers_;
    static std::vector<std::shared_ptr<ActuatorBiasBase>> biases_;
};

}  // namespace pam_mujoco
