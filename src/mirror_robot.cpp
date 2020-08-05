
#include "pam_mujoco/mirror_external_robot.hpp"

#DEFINE SEGMENT_ID "mujoco_pam"
#DEFINE NB_DOFS 4
#DEFINE QUEUE_SIZE 500000
#DEFINE MODEL_PATH pam_model_file

namespace pam_mujoco
{

  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  void run(std::string segment_id,
	   std::string model_path)
  {

    mju_strncpy(filename, model_path, 1000);
    loadmodel();
    int index_q_robot = m->jnt_qposadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
    int index_qvel_robot = m->jnt_dofadr[mj_name2id(m, mjOBJ_JOINT, "joint_base_rotation")];
    
    pam_mujoco::MirrorExternalRobot::clear(segment_id);
    pam_mujoco::MirrorExternalRobot<1,QUEUE_SIZE,NB_DOFS> mirroring(segment_id,
								    index_q_robot,
								    index_qvel_robot
								    d);
    
    mujoco::mjcb_control = control<1>;
    mujoco::mju_user_warning = exit;

    // initialize everything
    init();
    
    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d); 
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    mj_deactivate();


  }
  

}
