#include <array>
#include "pam_models/muscle.hpp"

#define N_MUSCLES 16
#define N_DOFS 4

namespace pam_mujoco
{

  class PamRobot
  {
  private:
    std::array<pam_models::Muscle,N_MUSCLES> muscles_;
  };
  
  
}
