#include "pam_mujoco/run_mujoco.hpp"

namespace pam_mujoco
{

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
			     int index_qvel,
			     bool active_only)
  {
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring =
      std::make_shared<mfj>(segment_id,joint,index_qpos,index_qvel,active_only);
    pam_mujoco::Controllers::add(mirroring);
  }

  
  void add_mirror_until_contact_free_joint(std::string segment_id,
					   std::string joint,
					   int index_qpos,
					   int index_qvel,
					   std::string contact_segment_id,
					   bool active_only)
  {
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring =
      std::make_shared<mfj>(segment_id,joint,index_qpos,index_qvel,
			    contact_segment_id,active_only);
    pam_mujoco::Controllers::add(mirroring);
  }

  void add_4dofs_pressure_controller(std::string segment_id,
				     std::string robot_joint_base,
				     std::array<double,NB_DOFS*2> scale_min_pressure,
				     std::array<double,NB_DOFS*2> scale_max_pressure,
				     std::array<double,NB_DOFS*2> scale_min_activation,
				     std::array<double,NB_DOFS*2> scale_max_activation,
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
  
  /*void add_bursting(std::string mujoco_id,
		    std::string segment_id)
  {
    BurstingMode::activated = true;
    std::shared_ptr<BurstController> bc
      = std::make_shared<BurstController>(mujoco_id,segment_id);
    pam_mujoco::Controllers::add(bc);
    }*/

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


  void add_item_control(const MujocoConfig & config,
			MujocoItemControl mic)
  {

    if (type==MujocoItemTypes::ball || type==MujocoItemTypes::hitpoint ||
	type==MujocoItemTypes::goal)
      {
	if(mic.until_contact())
	  {
	    std::string contact_segment_id;
	    if(mic.contactee_type==ContacteeTypes::table)
	      {
		contact_segment_id = std::string(mic.segment_id)+std::string("_table");
		add_table_contact_free_joint(contact_segment_id,
					     mic.index_qpos,mic.index_qvel,
					     std::string(mic.geometry),
					     std::string(config.geometry_table));
	      }
	    if(mic.contactee_type==ContacteeTypes::racket1)
	      {
		contact_segment_id = std::string(mic.segment_id)+std::string("_racket1");
		add_robot1_contact_free_joint(contact_segment_id,
					      mic.index_qpos,mic.index_qvel,
					      std::string(mic.geometry),
					      std::string(mic.geometry_racket1));
	      }
	    if(mic.contactee_type==ContacteeTypes::racket2)
	      {
		contact_segment_id = std::string(mic.segment_id)+std::string("_racket2");
		add_robot1_contact_free_joint(contact_segment_id,
					      mic.index_qpos,mic.index_qvel,
					      std::string(mic.geometry),
					      std::string(mic.geometry_racket2));
	      }
	    add_mirror_until_contact_free_joint(std::string(mic.segment_id),
						std::string(mic.joint),
						mic.index_qpos,
						mic.index_qvel,
						contact_segment_id),
						mic.active_only);
	  }
	else
	  {
	    add_mirror_free_joint(std::string(mic.segment_id),
				  std::string(mic.joint),
				  mic.index_qpos,
				  mic.index_qvel,
				  mic.active_only);
	  }
      }
    if(type==MujocoItemType::joints)
      {
	add_mirror_robot(std::string(mic.segment_id),
			 std::string(mic.joint));
	return;
      }
    if(type==MujocoItemType::pressures)
      {
	pam_interface::JsonConfiguration pam_interface_config(std::string(mic.configuration_path));
	std::string robot_joint_base;
	std::array<double,8> scale_min_pressure;
	std::array<double,8> scale_max_pressure;
	std::array<double,8> scale_min_activation;
	std::array<double,8> scale_max_activation;
	std::array<double,8> a_init;
	std::array<double,8> l_MTC_change_init;
	a_init.fill(0.5);
	l_MTC_change_init.fill(0.0);
	scale_min_activation.fill(0.001);
	scale.max_activation.fill(1.0);
	for (int dof=0;dof<4;dof++)
	  {
	    scale_min_pressure[2*dof]=pam_interface_config.min_pressures_ago[dof];
	    scale_min_pressure[2*dof+1]=pam_interface_config.min_pressures_antago[dof];
	    scale_max_pressure[2*dof]=pam_interface_config.max_pressures_ago[dof];
	    scale_max_pressure[2*dof+1]=pam_interface_config.max_pressures_antago[dof];
	  }
	add_4dofs_pressure_controller(std::string(mic.segment_id,)
				      std::string(mic.joint),
				      scale_min_pressure,
				      scale_max_pressure,
				      scale_min_activation,
				      scale_max_activation,
				      std::string(mic.configuration_path),
				      std::string(mic.configuration_path),
				      a_init,
				      l_MTC_change_init);

      }
  }
  


  
}
