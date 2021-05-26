#pragma once

#include "shared_memory/shared_memory.hpp"

namespace pam_mujoco
{
class Listener
{
public:
    Listener(std::string segment_id, std::string object_id);
    bool do_once();
    bool is_on();

private:
    std::string segment_id_;
    std::string object_id_;
};

}  // namespace pam_mujoco
