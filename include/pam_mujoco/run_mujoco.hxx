
template<int NB_BALLS>
void add_mirror_balls(std::string segment_id,std::string ball_obj_joint)
{
  pam_mujoco::MirrorBalls<QUEUE_SIZE,NB_BALLS>::clear(segment_id);
  typedef pam_mujoco::MirrorBalls<QUEUE_SIZE,NB_BALLS> meb;
  std::shared_ptr<meb> mirroring =
    std::make_shared<meb>(segment_id,ball_obj_joint);
  pam_mujoco::Controllers::add(mirroring);
}
