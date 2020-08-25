
template<int NB_BALLS>
void add_mirror_balls(std::string segment_id)
{
  pam_mujoco::MirrorExternalBalls<QUEUE_SIZE,NB_BALLS>::clear(segment_id);
  typedef pam_mujoco::MirrorExternalBalls<QUEUE_SIZE,NB_BALLS> meb;
  std::shared_ptr<meb> mirroring =
    std::make_shared<meb>(segment_id);
  pam_mujoco::Controllers::add(mirroring);
}
