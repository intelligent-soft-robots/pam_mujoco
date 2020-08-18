#include "pam_mujoco/run_mujoco.hpp"

namespace pam_mujoco
{

  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  std::string get_mirror_external_robot_segment_id(std::string mujoco_id)
  {
    return std::string(mujoco_id+std::string("_")+
		       SEGMENT_ID_PREFIX+MIRROR_EXTERNAL_ROBOT_SUFFIX);
  }

  void add_mirror_external_robot(std::string segment_id,
				 const mjModel* m,
				 const mjData* d_init)
  {
    pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
      std::make_shared<mer>(segment_id,
			    m,d);
    
    pam_mujoco::Controllers::add(mirroring);
  }

  void construct_controllers(std::string mujoco_id,
			     std::set<int> controller_ids,
			     const mjModel* m,
			     const mjData* d_init)
  {
    if(controller_ids.find(MIRROR_EXTERNAL_ROBOT)!=controller_ids.end())
      {
	std::cout << "adding mirror external robot controller" << std::endl;
	add_mirror_external_robot(get_mirror_external_robot_segment_id(mujoco_id),
				  m,d);
	std::cout << "... controller added" << std::endl;
      }
  }

  void execute(std::string mujoco_id, std::string model_path,
	       std::set<int> controller_ids)
  {

    // initialize everything
    init();

    // reading model from file and loading it
    mju_strncpy(filename, model_path.c_str(), 1000);
    loadmodel();

    // constructing the requested controllers
    // (m and d are global variables defined in mujoco_base.hpp)
    construct_controllers(mujoco_id,
			  controller_ids,
			  m,d);

    // setting the constructed controllers as mujoco controllers
    // (how it works: construct_controller aboves populate the (global)
    // vector pam_mujoco::Controllers::controllers_ with instances of controllers.
    // The pam_mujoco::Controllers::apply has these controllers being called
    // sequentially at each mujoco iteration).
    mjcb_control = pam_mujoco::Controllers::apply;

    // exiting on error
    mju_user_warning = exit;

    // set the shared memory segment mujoco_id to true,
    // indicating the mujoco thread started below
    // the job should run until user code calls the function
    // "request_stop" (see run_management.hpp)
    set_started(mujoco_id);


    run(&mujoco_id);
    
    // start simulation thread (run is a function defined mujoco_base.hpp)
    /*real_time_tools::RealTimeThread thread;
    thread.block_memory();
    thread.create_realtime_thread(run,&mujoco_id);

    // running until "request_stop" function is called
    while (! is_stop_requested(mujoco_id))
      {
	std::cout << "looping ... " << std::endl;
	usleep(2000);
      }
    
      thread.join();*/

    
  }

  
}
