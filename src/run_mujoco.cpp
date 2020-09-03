#include "pam_mujoco/run_mujoco.hpp"

namespace pam_mujoco
{

  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  void add_mirror_robot(std::string segment_id,
			std::string mujoco_id,
			std::string robot_joint_base)
  {
    pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
      std::make_shared<mer>(segment_id,robot_joint_base);
    pam_mujoco::Controllers::add(mirroring);
  }

  void add_mirror_one_ball(std::string segment_id,
			   std::string mujoco_id,
			   std::string ball_obj_joint)
  {
    std::map<int,std::string> empty;
    add_mirror_balls<1>(segment_id,ball_obj_joint,empty);
  }

  void add_mirror_until_contact_one_ball(std::string segment_id,
					 std::string mujoco_id,
					 std::string ball_obj_joint,
					 std::string contact_segment_id)
  {
    std::map<int,std::string> ball_index_segment_id;
    ball_index_segment_id[0]=contact_segment_id;
    add_mirror_balls<1>(segment_id,ball_obj_joint,
			ball_index_segment_id);
  }

  void add_4dofs_pressure_controller(std::string segment_id,
				     int scale_min_pressure, int scale_max_pressure,
				     int scale_min_activation, int scale_max_activation,
				     std::string muscle_json_config_path_ago,
				     std::string muscle_json_config_path_antago,
				     std::array<double,8> a_init,
				     std::array<double,8> l_MTC_change_init)
  {
    add_pressure_controller<4>(segment_id,
			       scale_min_pressure, scale_max_pressure,
			       scale_min_activation, scale_max_activation,
			       muscle_json_config_path_ago,
			       muscle_json_config_path_antago,
			       a_init,
			       l_MTC_change_init);
  }
  
  void add_bursting(std::string mujoco_id,
		    std::string segment_id)
  {
    std::shared_ptr<BurstController> bc
      = std::make_shared<BurstController>(mujoco_id,segment_id);
    pam_mujoco::Controllers::add(bc);
  }

  void add_contact_ball(std::string segment_id_contact,
			std::string segment_id_reset,
			std::string ball_obj_joint,
			std::string ball_geom,
			std::string contactee_geom,
			const RecomputeStateConfig& config)
  {
    std::shared_ptr<ContactBall> cb
      = std::make_shared<ContactBall>(segment_id_contact,
				      segment_id_reset,
				      config,
				      ball_obj_joint,
				      ball_geom,
				      contactee_geom);
    pam_mujoco::Controllers::add(cb);
  }

  void add_default_contact_ball(std::string segment_id_contact,
			std::string segment_id_reset,
			std::string ball_obj_joint,
			std::string ball_geom,
			std::string contactee_geom)
  {
    RecomputeStateConfig config;
    add_contact_ball(segment_id_contact,
		     segment_id_reset,
		     ball_obj_joint,
		     ball_geom,
		     contactee_geom,
		     config);
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
