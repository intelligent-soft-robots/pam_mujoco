#pragma once

#include <tuple>
#include "o80/state2d.hpp"

namespace pam_mujoco
{
class JointState : public o80::State2d
{
public:
    JointState() : o80::State2d()
    {
    }
    JointState(double position, double velocity)
        : o80::State2d(position, velocity)
    {
    }
};
}  // namespace pam_mujoco
