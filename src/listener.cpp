#include "pam_mujoco/listener.hpp"

namespace pam_mujoco
{
Listener::Listener(std::string segment_id, std::string object_id)
    : segment_id_{segment_id}, object_id_{object_id}
{
    shared_memory::set<bool>(segment_id, object_id, false);
}

bool Listener::do_once()
{
    bool v;
    shared_memory::get<bool>(segment_id_, object_id_, v);
    if (v)
    {
        shared_memory::set<bool>(segment_id_, object_id_, false);
    }
    return v;
}

bool Listener::is_on()
{
    bool v;
    shared_memory::get<bool>(segment_id_, object_id_, v);
    return v;
}

}  // namespace pam_mujoco
