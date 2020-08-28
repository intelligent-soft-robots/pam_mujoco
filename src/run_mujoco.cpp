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

  std::string get_mirror_one_ball_segment_id(std::string mujoco_id)
  {
    return std::string(mujoco_id+std::string("_")+
		       SEGMENT_ID_PREFIX+MIRROR_ONE_BALL_SUFFIX);
  }
  
  void add_mirror_robot(std::string mujoco_id, std::string robot_joint_base)
  {
    std::string segment_id = get_mirror_robot_segment_id(mujoco_id);
    pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
      std::make_shared<mer>(segment_id,robot_joint_base);
    pam_mujoco::Controllers::add(mirroring);
  }

  void add_mirror_one_ball(std::string mujoco_id, std::string ball_obj_joint)
  {
    std::string segment_id = get_mirror_one_ball_segment_id(mujoco_id);
    add_mirror_balls<1>(segment_id,ball_obj_joint);
  }
  
  void add_bursting(std::string mujoco_id,
		    std::string segment_id)
  {
    std::shared_ptr<BurstController> bc
      = std::make_shared<BurstController>(mujoco_id,segment_id);
    pam_mujoco::Controllers::add(bc);
  }
  
  void init_mujoco()
  {
    init();
  }
  
  void execute(std::string mujoco_id, std::string model_name)
  {

    // reading model from file and loading it
    // (MODEL_PATH set in the CMakeLists.txt file
    // as the abs path to the models folder of the pam_mujoco
    // catkin package)
    std::string model_path = MODEL_PATHS+model_name+std::string(".xml");
    mju_strncpy(filename, model_path.c_str(), 1000);
    loadmodel();

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
