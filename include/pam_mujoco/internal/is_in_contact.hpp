#pragma once

#include "mujoco.h"

namespace pam_mujoco
{
namespace internal
{
bool is_in_contact(const mjModel* m,
                   mjData* d,
                   int index_geom,
                   int index_geom_contactee);
}
}  // namespace pam_mujoco
