#include "pam_mujoco/add_controllers.hpp"

namespace pam_mujoco
{
void add_mirror_robot(std::string segment_id,
                      std::string robot_joint_base,
                      std::string robot_racket)
{
    pam_mujoco::MirrorRobot<QUEUE_SIZE, NB_DOFS>::clear(segment_id);
    typedef pam_mujoco::MirrorRobot<QUEUE_SIZE, NB_DOFS> mer;
    std::shared_ptr<mer> mirroring =
        std::make_shared<mer>(segment_id, robot_joint_base, robot_racket);
    pam_mujoco::Controllers::add(mirroring);
}

void add_mirror_free_joint(std::string mujoco_id,
                           std::string segment_id,
                           std::string joint,
                           bool active_only)
{
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring =
        std::make_shared<mfj>(mujoco_id, segment_id, joint, active_only);
    pam_mujoco::Controllers::add(mirroring);
}

void add_mirror_until_contact_free_joint(std::string mujoco_id,
                                         std::string segment_id,
                                         std::string joint,
                                         std::vector<std::string> contact_segment_ids,
                                         bool active_only)
{
    pam_mujoco::MirrorFreeJoint<QUEUE_SIZE>::clear(segment_id);
    typedef pam_mujoco::MirrorFreeJoint<QUEUE_SIZE> mfj;
    std::shared_ptr<mfj> mirroring = std::make_shared<mfj>(
        mujoco_id, segment_id, joint, contact_segment_ids, active_only);
    pam_mujoco::Controllers::add(mirroring);
}

void add_4dofs_pressure_controller(
    std::string segment_id,
    std::string robot_joint_base,
    std::array<double, NB_DOFS * 2> scale_min_pressure,
    std::array<double, NB_DOFS * 2> scale_max_pressure,
    std::array<double, NB_DOFS * 2> scale_min_activation,
    std::array<double, NB_DOFS * 2> scale_max_activation,
    std::string muscle_json_config_path_ago,
    std::string muscle_json_config_path_antago,
    std::array<double, 8> a_init,
    std::array<double, 8> l_MTC_change_init)
{
    add_pressure_controller<4>(segment_id,
                               robot_joint_base,
                               scale_min_pressure,
                               scale_max_pressure,
                               scale_min_activation,
                               scale_max_activation,
                               muscle_json_config_path_ago,
                               muscle_json_config_path_antago,
                               a_init,
                               l_MTC_change_init);
}

void add_contact_free_joint(std::string mujoco_id,
                            std::string segment_id,
                            std::string joint,
                            std::string geom,
                            std::string robot_base,			    
                            std::string contactee_geom,
                            ContactItems contact_item)
{
    std::shared_ptr<ContactBall> cb = std::make_shared<ContactBall>(mujoco_id, segment_id, joint,
                                                                    geom, robot_base,
                                                                    contactee_geom, contact_item);
    pam_mujoco::Controllers::add(cb);
}

void add_table_contact_free_joint(std::string mujoco_id,
                                  std::string segment_id,
                                  std::string joint,
                                  std::string geom,
                                  std::string robot_base,
                                  std::string contactee_geom)
{
    add_contact_free_joint(mujoco_id, segment_id, joint, geom,
                           robot_base, contactee_geom,
                           ContactItems::Table);
}

void add_robot1_contact_free_joint(std::string mujoco_id,
                                   std::string segment_id,
                                   std::string joint,
                                   std::string geom,
                                   std::string robot_base,				   
                                   std::string contactee_geom)
{
    add_contact_free_joint(mujoco_id, segment_id, joint, geom,
                           robot_base, contactee_geom, ContactItems::Robot1);
}

void add_robot2_contact_free_joint(std::string mujoco_id,
                                   std::string segment_id,
                                   std::string joint,
                                   std::string geom,
                                   std::string robot_base,				   
                                   std::string contactee_geom)
{
    add_contact_free_joint(mujoco_id, segment_id, joint, geom, robot_base,
                           contactee_geom,ContactItems::Robot2);
}

void add_joints_control(MujocoRobotJointControl mrc)
{
    add_mirror_robot(std::string(mrc.segment_id),
                     std::string(mrc.joint),
                     std::string(mrc.racket));
    return;
}

void add_pressures_control(MujocoRobotPressureControl mpc)
{
    std::string config_path(mpc.json_controller_path);
    pam_interface::JsonConfiguration<4> pam_interface_config(config_path);
    std::string robot_joint_base;
    std::array<double, 8> scale_min_pressure;
    std::array<double, 8> scale_max_pressure;
    std::array<double, 8> scale_min_activation;
    std::array<double, 8> scale_max_activation;
    std::array<double, 8> a_init;
    std::array<double, 8> l_MTC_change_init;
    a_init.fill(0.5);
    l_MTC_change_init.fill(0.0);
    scale_min_activation.fill(0.001);
    scale_max_activation.fill(1.0);
    for (int dof = 0; dof < 4; dof++)
    {
        scale_min_pressure[2 * dof] =
            pam_interface_config.min_pressures_ago[dof];
        scale_min_pressure[2 * dof + 1] =
            pam_interface_config.min_pressures_antago[dof];
        scale_max_pressure[2 * dof] =
            pam_interface_config.max_pressures_ago[dof];
        scale_max_pressure[2 * dof + 1] =
            pam_interface_config.max_pressures_antago[dof];
    }
    add_4dofs_pressure_controller(std::string(mpc.segment_id),
                                  std::string(mpc.joint),
                                  scale_min_pressure,
                                  scale_max_pressure,
                                  scale_min_activation,
                                  scale_max_activation,
                                  std::string(mpc.json_ago_hill_path),
                                  std::string(mpc.json_antago_hill_path),
                                  a_init,
                                  l_MTC_change_init);
}

template <int NB_ITEMS>
void add_items_control(const MujocoConfig& config,
                       MujocoItemsControl<NB_ITEMS> mic)
{
    std::array<std::string, NB_ITEMS> str_joints;
    for (int i = 0; i < NB_ITEMS; i++)
    {
        str_joints[i] = std::string(mic.joint[i]);
    }

    if (mic.contact_table || mic.contact_robot1 || mic.contact_robot2)
    {
        std::array<std::string, NB_ITEMS> contact_segment_ids;

        for (int item = 0; item < NB_ITEMS; item++)
        {
            if (mic.type[item] == MujocoItemTypes::ball ||
                mic.type[item] == MujocoItemTypes::hit_point ||
                mic.type[item] == MujocoItemTypes::goal)
            {
                std::string contact_segment_id;
                if (mic.contact_table)
                {
                    contact_segment_id = std::string(mic.segment_id) +
                                         std::string("_table_") +
                                         std::to_string(item);
                    add_table_contact_free_joint(
                        std::string(config.mujoco_id),
                        contact_segment_id,
                        std::string(mic.joint[item]),
                        std::string(mic.geometry[item]),
                        std::string(config.robot1_base),
                        std::string(config.table_geometry));
                }
                if (mic.contact_robot1)
                {
                    contact_segment_id = std::string(mic.segment_id) +
                                         std::string("_racket1_") +
                                         std::to_string(item);
                    add_robot1_contact_free_joint(
                        std::string(config.mujoco_id),
                        contact_segment_id,
                        std::string(mic.joint[item]),
                        std::string(mic.geometry[item]),
                        std::string(config.robot1_base),
                        std::string(config.racket1_geometry));
                }
                if (mic.contact_robot2)
                {
                    contact_segment_id = std::string(mic.segment_id) +
                                         std::string("_racket2") +
                                         std::to_string(item);
                    add_robot2_contact_free_joint(
                                                  std::string(config.mujoco_id),
                                                  contact_segment_id,
                                                  std::string(mic.joint[item]),
                                                  std::string(mic.geometry[item]),
                                                  std::string(config.robot2_base),
                                                  std::string(config.racket2_geometry));
                }

                contact_segment_ids[item] = contact_segment_id;
            }
        }
        add_mirror_until_contact_free_joints<NB_ITEMS>(
            std::string(config.mujoco_id),
            std::string(mic.segment_id),
            str_joints,
            mic.robot_geom,
            contact_segment_ids,
            mic.active_only);
    }

    else
    {
        add_mirror_free_joints<NB_ITEMS>(std::string(config.mujoco_id),
                                         std::string(mic.segment_id),
                                         str_joints,
                                         mic.robot_geom,
                                         mic.active_only);
    }
}

void add_3_items_control(const MujocoConfig& config, MujocoItemsControl<3> mic)
{
    add_items_control<3>(config, mic);
}

void add_10_items_control(const MujocoConfig& config,
                          MujocoItemsControl<10> mic)
{
    add_items_control<10>(config, mic);
}

void add_20_items_control(const MujocoConfig& config,
                          MujocoItemsControl<20> mic)
{
    add_items_control<20>(config, mic);
}

void add_50_items_control(const MujocoConfig& config,
                          MujocoItemsControl<50> mic)
{
    add_items_control<50>(config, mic);
}

void add_100_items_control(const MujocoConfig& config,
                           MujocoItemsControl<100> mic)
{
    add_items_control<100>(config, mic);
}

void add_item_control(const MujocoConfig& config, MujocoItemControl mic)
{
    if (mic.type == MujocoItemTypes::ball ||
        mic.type == MujocoItemTypes::hit_point ||
        mic.type == MujocoItemTypes::goal)
    {
        if (mic.contact_table || mic.contact_robot1 || mic.contact_robot2)
        {
            std::vector<std::string> contact_segment_ids;
            if (mic.contact_table)
            {
                std::string contact_segment_id(std::string(mic.segment_id) + std::string("_table"));
                contact_segment_ids.push_back(contact_segment_id);
                add_table_contact_free_joint(
                    std::string(config.mujoco_id),
                    contact_segment_id,
                    std::string(mic.joint),
                    std::string(mic.geometry),
                    std::string(config.robot1_base),
                    std::string(config.table_geometry));
            }
            if (mic.contact_robot1)
            {
                std::string contact_segment_id(std::string(mic.segment_id) + std::string("_racket1"));
                contact_segment_ids.push_back(contact_segment_id);
                add_robot1_contact_free_joint(
                    std::string(config.mujoco_id),
                    contact_segment_id,
                    std::string(mic.joint),
                    std::string(mic.geometry),
                    std::string(config.robot1_base),
                    std::string(config.racket1_geometry));
            }
            if (mic.contact_robot2)
            {
                std::string contact_segment_id(std::string(mic.segment_id) + std::string("_racket2"));
                contact_segment_ids.push_back(contact_segment_id);
                add_robot2_contact_free_joint(
                    std::string(config.mujoco_id),
                    contact_segment_id,
                    std::string(mic.joint),
                    std::string(mic.geometry),
                    std::string(config.robot2_base),
                    std::string(config.racket2_geometry));
            }
            add_mirror_until_contact_free_joint(std::string(config.mujoco_id),
                                                std::string(mic.segment_id),
                                                std::string(mic.joint),
                                                contact_segment_ids,
                                                mic.active_only);
        }
        else
        {
            add_mirror_free_joint(std::string(config.mujoco_id),
                                  std::string(mic.segment_id),
                                  std::string(mic.joint),
                                  mic.active_only);
        }
    }
}

}  // namespace pam_mujoco
