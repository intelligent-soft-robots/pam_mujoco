#include "pam_mujoco/run_mujoco.hpp"

namespace pam_mujoco
{

  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  std::string get_mirror_robot_segment_id(std::string mujoco_id)
  {
    return std::string(mujoco_id+std::string("_")+
		       SEGMENT_ID_PREFIX+MIRROR_ROBOT_SUFFIX);
  }

  void add_mirror_robot(std::string segment_id)
  {
    pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorExternalRobot<QUEUE_SIZE,NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
      std::make_shared<mer>(segment_id);
    pam_mujoco::Controllers::add(mirroring);
  }

  std::string get_mirror_one_ball_segment_id(std::string mujoco_id)
  {
    return std::string(mujoco_id+std::string("_")+
		       SEGMENT_ID_PREFIX+MIRROR_ONE_BALL_SUFFIX);
  }
  
  void add_bursting_controller(std::string mujoco_id,
			       std::string segment_id)
  {
    std::shared_ptr<BurstController> bc
      = std::make_shared<BurstController>(mujoco_id,segment_id);
    pam_mujoco::Controllers::add(bc);
  }
  
  void construct_controllers(std::string mujoco_id,
			     std::set<ControllerTypes> controller_types,
			     std::string burster_segment_id,
			     const mjModel* m,
			     const mjData* d_init)
  {
    if(controller_types.find(ControllerTypes::MIRROR_ROBOT)!=controller_types.end())
      {
	add_mirror_robot(get_mirror_robot_segment_id(mujoco_id));
      }
    if(controller_types.find(ControllerTypes::MIRROR_ONE_BALL)!=controller_types.end())
      {
	add_mirror_balls<1>(get_mirror_one_ball_segment_id(mujoco_id));
      }
    if(std::string("").compare(burster_segment_id)!=0)
      {
	add_bursting_controller(mujoco_id,burster_segment_id);
      }
  }

  void execute(std::string mujoco_id, std::string model_name,
	       std::set<ControllerTypes> controller_types,
	       std::string burster_segment_id)
  {

    // initialize everything
    init();

    // reading model from file and loading it
    // (MODEL_PATH set in the CMakeLists.txt file
    // as the abs path to the models folder of the pam_mujoco
    // catkin package)
    std::string model_path = MODEL_PATHS+model_name+std::string(".xml");
    mju_strncpy(filename, model_path.c_str(), 1000);
    loadmodel();

    // constructing the requested controllers
    // (m and d are global variables defined in mujoco_base.hpp)
    construct_controllers(mujoco_id,
			  controller_types,
			  burster_segment_id,
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


    // running mujoco (run defined in mujoco_base.hpp)
    run(&mujoco_id);
    
    
  }

  
}
