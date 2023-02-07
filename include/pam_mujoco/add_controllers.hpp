#pragma once

#include <array>
#include <set>
#include <string>
#include <tuple>
#include "pam_interface/configuration.hpp"
#include "pam_mujoco/contact_ball.hpp"
#include "pam_mujoco/mirror_free_joint.hpp"
#include "pam_mujoco/mirror_free_joints.hpp"
#include "pam_mujoco/mirror_robot.hpp"
#include "pam_mujoco/mujoco_config.hpp"
#include "pam_mujoco/pressure_controller.hpp"
#include "real_time_tools/thread.hpp"
#include "shared_memory/shared_memory.hpp"

namespace pam_mujoco
{
static constexpr int NB_DOFS = 4;
static constexpr long int QUEUE_SIZE = 500000;

void add_mirror_robot(std::string segment_id, std::string robot_joint_base);

template <int NB_ITEMS>
void add_mirror_free_joints(std::string mujoco_id,
                            std::string segment_id,
                            std::array<std::string, NB_ITEMS> joints,
                            std::string robot_joint_base,
                            bool active_only);

void add_mirror_free_joint(std::string segment_id,
                           std::string joint,
                           bool active_only);

template <int NB_ITEMS>
void add_mirror_until_contact_free_joints(
    std::string mujoco_id,
    std::string segment_id,
    std::array<std::string, NB_ITEMS> joints,
    std::string robot_joint_base,
    std::array<std::string, NB_ITEMS> contact_segment_ids,
    bool active_only);

void add_mirror_until_contact_free_joint(std::string segment_id,
                                         std::string joint,
                                         std::string contact_segment_id,
                                         bool active_only);

void add_contact_free_joint(std::string segment_id,
                            std::string geom,
                            std::string contactee_geom,
                            ContactItems contact_item);

void add_table_contact_free_joint(std::string segment_id,
                                  std::string geom,
                                  std::string contactee_geom);

void add_robot1_contact_free_joint(std::string segment_id,
                                   std::string geom,
                                   std::string contactee_geom);

void add_robot2_contact_free_joint(std::string segment_id,
                                   std::string geom,
                                   std::string contactee_geom);

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
    std::array<double, NB_DOFS * 2> l_MTC_change_init);

void add_4dofs_pressure_controller(std::string segment_id,
                                   std::string robot_joint_base,
                                   std::array<double, 8> scale_min_pressure,
                                   std::array<double, 8> scale_max_pressure,
                                   std::array<double, 8> scale_min_activation,
                                   std::array<double, 8> scale_max_activation,
                                   std::string muscle_json_config_path_ago,
                                   std::string muscle_json_config_path_antago,
                                   std::array<double, 8> a_init,
                                   std::array<double, 8> l_MTC_change_init);

void add_item_control(const MujocoConfig& config, MujocoItemControl mic);

void add_3_items_control(const MujocoConfig& config, MujocoItemsControl<3> mic);
void add_10_items_control(const MujocoConfig& config,
                          MujocoItemsControl<10> mic);
void add_20_items_control(const MujocoConfig& config,
                          MujocoItemsControl<20> mic);
void add_50_items_control(const MujocoConfig& config,
                          MujocoItemsControl<50> mic);
void add_100_items_control(const MujocoConfig& config,
                           MujocoItemsControl<100> mic);

void add_joints_control(MujocoRobotJointControl mrc);
void add_pressures_control(MujocoRobotPressureControl mpc);

#include "add_controllers.hxx"

}  // namespace pam_mujoco
