
template <int NB_ITEMS>
void add_mirror_free_joints(std::string segment_id,
                            std::array<std::string, NB_ITEMS> joints,
                            std::array<int, NB_ITEMS> index_qpos,
                            std::array<int, NB_ITEMS> index_qvel,
                            bool active_only)
{
    pam_mujoco::MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS> mfj;
    std::shared_ptr<mfj> mirroring = std::make_shared<mfj>(
        segment_id, joints, index_qpos, index_qvel, active_only);
    pam_mujoco::Controllers::add(mirroring);
}

template <int NB_ITEMS>
void add_mirror_until_contact_free_joints(
    std::string segment_id,
    std::array<std::string, NB_ITEMS> joints,
    std::array<int, NB_ITEMS> index_qpos,
    std::array<int, NB_ITEMS> index_qvel,
    std::array<std::string, NB_ITEMS> contact_segment_ids,
    bool active_only)
{
    pam_mujoco::MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoints<QUEUE_SIZE, NB_ITEMS> mfj;
    std::shared_ptr<mfj> mirroring = std::make_shared<mfj>(segment_id,
                                                           joints,
                                                           index_qpos,
                                                           index_qvel,
                                                           contact_segment_ids,
                                                           active_only);
    pam_mujoco::Controllers::add(mirroring);
}

template <int NB_DOFS>
void add_pressure_controller(
    std::string segment_id,
    std::string robot_joint_base,
    std::array<double, NB_DOFS * 2> scale_min_pressure,
    std::array<double, NB_DOFS * 2> scale_max_pressure,
    std::array<double, NB_DOFS * 2> scale_min_activation,
    std::array<double, NB_DOFS * 2> scale_max_activation,
    std::string muscle_json_config_path_ago,
    std::string muscle_json_config_path_antago,
    std::array<double, NB_DOFS * 2> a_init,
    std::array<double, NB_DOFS * 2> l_MTC_change_init)
{
    pam_mujoco::PressureController<QUEUE_SIZE, NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::PressureController<QUEUE_SIZE, NB_DOFS> pc;
    std::shared_ptr<pc> pressure_controller =
        std::make_shared<pc>(segment_id,
                             robot_joint_base,
                             scale_min_pressure,
                             scale_max_pressure,
                             scale_min_activation,
                             scale_max_activation,
                             muscle_json_config_path_ago,
                             muscle_json_config_path_antago,
                             a_init,
                             l_MTC_change_init);
    pam_mujoco::Controllers::add(pressure_controller);
    pam_mujoco::Controllers::add_bias(pressure_controller);
}
