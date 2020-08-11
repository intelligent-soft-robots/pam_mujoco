#include "pam_mujoco/mtest.hpp"
#include "pam_mujoco/mirror_external_robot.hpp"

#define SEGMENT_ID "mujoco_pam"
#define NB_DOFS 4
#define QUEUE_SIZE 500000
#define MODEL_PATH pam_model_file

using namespace pam_mujoco;

bool run_g = true;
std::string error_message_g("no error");
  
void exit(const char* text)
{
  run_g = false;
  error_message_g = std::string(text);
}

void execute(std::string segment_id,
	     std::string model_path)
{

  // initialize everything
  init();
    
  mju_strncpy(filename, model_path.c_str(), 1000);
  loadmodel();

  pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
  pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS> mirroring(segment_id,
  								m,d);

  pam_mujoco::Controllers::add(mirroring);
    
  mjcb_control = pam_mujoco::Controllers::apply;
  mju_user_warning = exit;

  // start simulation thread
  run();
}

int main()
{
  std::string segment_id("pam_mujoco");
  std::string model_path("/home/vberenz/Workspaces/pam/workspace/src/pam_mujoco/models/pamy.xml");
  execute(segment_id,model_path);
}
  
