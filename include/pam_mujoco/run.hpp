#pragma once

#include "pam_mujoco/muscles.hpp"


namespace pam_mujoco
{

  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  
  void run(int nb_dofs,
	   std::vector<double> a_init,
	   std::string model_path,
	   std::string json_params1,
	   std::string json_params2)
  {

    MusclesController<1,1> muscles_controller(nb_dofs,
					      json_params1,
					      json_params2,
					      a_init);
    
    mujoco::mjcb_control = control<1>;
    mujoco::mjcb_act_bias = get_force<1>;
    mujoco::mju_user_warning = exit;

    mju_strncpy(filename, argv[1], 1000);
    settings.loadrequest = 2;

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if( settings.loadrequest==1 )
            loadmodel();
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;

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
