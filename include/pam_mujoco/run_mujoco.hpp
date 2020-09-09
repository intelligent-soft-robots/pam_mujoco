#pragma once

#include <tuple>
#include <set>
#include <string>
#include "shared_memory/shared_memory.hpp"
#include "real_time_tools/thread.hpp"
#include "pam_mujoco/mujoco_base.hpp"
#include "pam_mujoco/mirror_robot.hpp"
#include "pam_mujoco/mirror_free_joint.hpp"
#include "pam_mujoco/contacts.hpp"
#include "pam_mujoco/burst_controller.hpp"
#include "pam_mujoco/pressure_controller.hpp"
#include "pam_mujoco/run_management.hpp"


namespace pam_mujoco
{

  static constexpr int NB_DOFS = 4;
  static constexpr long int QUEUE_SIZE = 500000;
  
  bool run_g = true;
  std::string error_message_g("no error");
  
  void exit(const char* text);
  
  void add_mirror_robot(std::string segment_id,
			std::string robot_joint_base);

  void add_mirror_free_joint(std::string segment_id,
			     std::string joint,
			     int index_qpos,
			     int index_qvel);
  
  void add_mirror_until_contact_free_joint(std::string segment_id,
					   std::string joint,
					   int index_qpos,
					   int index_qvel,
					   std::string contact_segment_id);

  void add_contact_free_joint(std::string segment_id_contact,
			      std::string segment_id_reset,
			      std::string joint,
			      int index_qpos,
			      int index_qvel,
			      std::string geom,
			      std::string contactee_geom,
			      const RecomputeStateConfig& config);

  void add_default_contact_free_joint(std::string segment_id_contact,
				std::string segment_id_reset,
				std::string joint,
				std::string geom,
				std::string contactee_geom);
  
  void add_bursting(std::string mujoco_id,std::string segment_id);

  template<int NB_DOFS>
  void add_pressure_controller(std::string segment_id,
			       double scale_min_pressure, double scale_max_pressure,
			       double scale_min_activation, double scale_max_activation,
			       std::string muscle_json_config_path_ago,
			       std::string muscle_json_config_path_antago,
			       std::array<double,NB_DOFS*2> a_init,
			       std::array<double,NB_DOFS*2> l_MTC_change_init);

  void add_4dofs_pressure_controller(std::string segment_id,
				     double scale_min_pressure, double scale_max_pressure,
				     double scale_min_activation, double scale_max_activation,
				     std::string muscle_json_config_path_ago,
				     std::string muscle_json_config_path_antago,
				     std::array<double,8> a_init,
				     std::array<double,8> l_MTC_change_init);
  
  void init_mujoco();
  void execute(std::string mujoco_id, std::string model_path);

  #include "run_mujoco.hxx"
  
}
