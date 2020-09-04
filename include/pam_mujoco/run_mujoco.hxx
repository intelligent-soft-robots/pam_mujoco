
template<int NB_BALLS>
void add_mirror_balls(std::string segment_id,std::string ball_obj_joint,
		      const std::map<int,std::string>& ball_index_segment_id)
{
  pam_mujoco::MirrorBalls<QUEUE_SIZE,NB_BALLS>::clear(segment_id);
  typedef pam_mujoco::MirrorBalls<QUEUE_SIZE,NB_BALLS> meb;
  std::shared_ptr<meb> mirroring =
    std::make_shared<meb>(segment_id,ball_obj_joint,
			  ball_index_segment_id);
  pam_mujoco::Controllers::add(mirroring);
}


template<int NB_DOFS>
void add_pressure_controller(std::string segment_id,
			     double scale_min_pressure, double scale_max_pressure,
			     double scale_min_activation, double scale_max_activation,
			     std::string muscle_json_config_path_ago,
			     std::string muscle_json_config_path_antago,
			     std::array<double,NB_DOFS*2> a_init,
			     std::array<double,NB_DOFS*2> l_MTC_change_init)
{
  pam_mujoco::PressureController<QUEUE_SIZE,NB_DOFS>::clear(segment_id);
  typedef pam_mujoco::PressureController<QUEUE_SIZE,NB_DOFS> pc;
  std::shared_ptr<pc> pressure_controller =
    std::make_shared<pc>(segment_id,
			 scale_min_pressure, scale_max_pressure,
			 scale_min_activation, scale_max_activation,
			 muscle_json_config_path_ago,
			 muscle_json_config_path_antago,
			 a_init,
			 l_MTC_change_init);
  pam_mujoco::Controllers::add(pressure_controller);
  pam_mujoco::Controllers::add_bias(pressure_controller);
}
