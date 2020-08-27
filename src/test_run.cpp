#include "pam_mujoco/run_mujoco.hpp"

int main()
{
  
  std::string mujoco_id("test");
  std::string model("pamy");
  //pam_mujoco::add_mirror_one_ball(mujoco_id);
  pam_mujoco::init_mujoco();
  pam_mujoco::execute(mujoco_id,model);
}
