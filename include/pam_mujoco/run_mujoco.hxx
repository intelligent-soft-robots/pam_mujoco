
template<int NB_BALLS>
void add_mirror_balls(std::string segment_id,
			       const mjModel* m,
			       const mjData* d_init)
{
  pam_mujoco::MirrorExternalBalls<QUEUE_SIZE,NB_BALLS>::clear(segment_id);
  typedef pam_mujoco::MirrorExternalBalls<QUEUE_SIZE,NB_BALLS> meb;
  std::shared_ptr<meb> mirroring =
    std::make_shared<meb>(segment_id,
			  m,d);
    
  pam_mujoco::Controllers::add(mirroring);
}
