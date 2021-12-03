// Copyright (c) 2021 Max Planck Gesellschaft
// Author : Vincent Berenz

#pragma once

#include <array>
#include "shared_memory/serializer.hpp"

namespace pam_mujoco
{
template <int NB_BALLS>
class ExtraBallsExtendedState
{
public:
    ExtraBallsExtendedState();
    ExtraBallsExtendedState(const std::array<bool, NB_BALLS>& contacts,
                            long int episode,
                            const std::array<double, 3>& robot_position);

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(contacts, robot_position, episode);
    }

public:
    std::array<bool, NB_BALLS> contacts;
    long int episode;
    std::array<double, 3> robot_position;
};

#include "extra_balls_extended_state.hxx"

}  // namespace pam_mujoco
