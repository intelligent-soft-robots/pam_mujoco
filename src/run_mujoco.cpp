#include "pam_mujoco/run_mujoco.hpp"

namespace pam_mujoco
{

  void exit(const char* text)
  {
    run_g = false;
    error_message_g = std::string(text);
  }

  void add_mirror_robot(std::string segment_id,
			std::string robot_joint_base)
  {
    pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorRobot<QUEUE_SIZE,NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
      std::make_shared<mer>(segment_id,robot_joint_base);
    pam_mujoco::Controllers::add(mirroring);
  }

  void add_share_robot_state(std::string segment_id,
			     std::string robot_joint_base)
  {
    pam_mujoco::ShareRobotState::clear(segment_id);
    typedef pam_mujoco::ShareRobotState srs;
    std::shared_ptr<srs> sharing =
      std::make_shared<srs>(segment_id,robot_joint_base);
    pam_mujoco::Controllers::add(sharing);
  }

  void add_mirror_free_joint(std::string segment_id,
			     std::string joint,
			     int index_qpos,
			     int index_qvel)
  {
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring =
      std::make_shared<mfj>(segment_id,joint,index_qpos,index_qvel);
    pam_mujoco::Controllers::add(mirroring);
  }

  
  void add_mirror_until_contact_free_joint(std::string segment_id,
					   std::string joint,
					   int index_qpos,
					   int index_qvel,
					   std::string contact_segment_id)
  {
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring =
      std::make_shared<mfj>(segment_id,joint,index_qpos,index_qvel,
			    contact_segment_id);
    pam_mujoco::Controllers::add(mirroring);
  }

  void add_4dofs_pressure_controller(std::string segment_id,
				     std::string robot_joint_base,
				     double scale_min_pressure, double scale_max_pressure,
				     double scale_min_activation, double scale_max_activation,
				     std::string muscle_json_config_path_ago,
				     std::string muscle_json_config_path_antago,
				     std::array<double,8> a_init,
				     std::array<double,8> l_MTC_change_init)
  {
    add_pressure_controller<4>(segment_id,
			       robot_joint_base,
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

  void add_contact_free_joint(std::string segment_id,
			      int index_qpos,
			      int index_qvel,
			      std::string geom,
			      std::string contactee_geom,
			      ContactItems contact_item)
  {
    std::shared_ptr<ContactBall> cb
      = std::make_shared<ContactBall>(segment_id,
				      index_qpos,
				      index_qvel,
				      geom,
				      contactee_geom,
				      contact_item);
    pam_mujoco::Controllers::add(cb);
  }

  void add_table_contact_free_joint(std::string segment_id,
				      int index_qpos,
				      int index_qvel,
				      std::string geom,
				      std::string contactee_geom)
  {
    add_contact_free_joint(segment_id,
			   index_qpos,
			   index_qvel,
			   geom,
			   contactee_geom,
			   ContactItems::Table);
  }

  void add_robot1_contact_free_joint(std::string segment_id,
				     int index_qpos,
				     int index_qvel,
				     std::string geom,
				     std::string contactee_geom)
  {
    add_contact_free_joint(segment_id,
			   index_qpos,
			   index_qvel,
			   geom,
			   contactee_geom,
			   ContactItems::Robot1);
  }

  void add_robot2_contact_free_joint(std::string segment_id,
				     int index_qpos,
				     int index_qvel,
				     std::string geom,
				     std::string contactee_geom)
  {
    add_contact_free_joint(segment_id,
			   index_qpos,
			   index_qvel,
			   geom,
			   contactee_geom,
			   ContactItems::Robot2);
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
    // (same principle for biases, also encapsulated by the Controllers static class)
    mjcb_control = pam_mujoco::Controllers::apply;
    mjcb_act_bias = pam_mujoco::Controllers::get_bias;
    
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
