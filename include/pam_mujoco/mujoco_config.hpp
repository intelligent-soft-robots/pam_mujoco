#pragma once

#include <unistd.h>
#include <array>
#include <cstring>
#include "shared_memory/shared_memory.hpp"

namespace pam_mujoco
{
enum MujocoItemTypes
{
    ball,
    hit_point,
    goal
};

enum ContactTypes
{
    no_contact,
    table,
    racket1,
    racket2
};
    
class MujocoRobotJointControl
{
public:
    MujocoRobotJointControl();
    MujocoRobotJointControl(std::string _segment_id,
                            std::string _joint,
                            std::string _racket,
                            bool _active_only);
    std::string to_string() const;

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(segment_id, joint, racket, active_only);
    }

public:
    char segment_id[200];
    char joint[200];
    char racket[200];
    bool active_only;
};

class MujocoRobotPressureControl
{
public:
    MujocoRobotPressureControl();
    MujocoRobotPressureControl(std::string _segment_id,
                               std::string _joint,
                               bool _active_only,
                               std::string _json_controller_path,
                               std::string _json_ago_hill_path,
                               std::string _json_antago_hill_path);
    std::string to_string() const;

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(segment_id,
                joint,
                json_controller_path,
                json_ago_hill_path,
                json_antago_hill_path,
                active_only);
    }

public:
    char segment_id[200];
    char joint[200];
    char json_controller_path[200];
    char json_ago_hill_path[200];
    char json_antago_hill_path[200];
    bool active_only;
};

class MujocoItemControl
{
public:
    MujocoItemControl();
    MujocoItemControl(MujocoItemTypes _type,
                      std::string _segment_id,
                      std::string _joint,
                      std::string _geometry,
                      bool _active_only,
                      bool contact_robot1,
                      bool contact_robot2,
                      bool contact_table);
    

public:
    std::string to_string() const;

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(type, segment_id, joint,
                geometry, active_only,
                contact_robot1, contact_robot2,
                contact_table);
    }

public:
    MujocoItemTypes type;
    char segment_id[200];
    char joint[200];
    char geometry[100];
    bool active_only;
    bool contact_robot1;
    bool contact_robot2;
    bool contact_table;
};

template <int NB_ITEMS>
class MujocoItemsControl
{
public:
    MujocoItemsControl();
    MujocoItemsControl(std::array<MujocoItemTypes, NB_ITEMS> _type,
                       std::string _segment_id,
                       std::array<std::string, NB_ITEMS> _joint,
                       std::array<std::string, NB_ITEMS> _geometry,
                       std::string _robot_geom,
                       bool _active_only,
                       bool contact_robot1,
                       bool contact_robot2,
                       bool contact_table);

public:
    std::string to_string() const;

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(type,
                segment_id,
                joint,
                geometry,
                robot_geom,
                active_only,
                contact_robot1,
                contact_robot2,
                contact_table);
    }

public:
    std::array<MujocoItemTypes, NB_ITEMS> type;
    char segment_id[200];
    std::array<char[200], NB_ITEMS> joint;
    std::array<char[100], NB_ITEMS> geometry;
    char robot_geom[200];
    bool active_only;
    bool contact_robot1;
    bool contact_robot2;
    bool contact_table;
};

#include "mujoco_config.hxx"

class MujocoConfig
{
public:
    MujocoConfig();
    MujocoConfig(std::string _mujoco_id);
    void set_burst_mode(bool use_burst);
    void set_accelerated_time(bool use_accelerated);
    void set_model_path(std::string path);
    void set_graphics(bool use_graphics);
    std::string to_string() const;
    void set_robot1_base(std::string robot1_base);
    void set_robot2_base(std::string robot2_base);    
    void set_racket_robot1(std::string _racket1_geometry);
    void set_racket_robot2(std::string _racket2_geometry);
    void set_table(std::string _table_geometry);

    void add_3_control(MujocoItemsControl<3> misc);
    void add_10_control(MujocoItemsControl<10> misc);
    void add_20_control(MujocoItemsControl<20> misc);
    void add_50_control(MujocoItemsControl<50> misc);
    void add_100_control(MujocoItemsControl<100> misc);

    void add_control(MujocoItemControl mic);
    void add_control(MujocoRobotJointControl mrc);
    void add_control(MujocoRobotPressureControl mpc);

public:
    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(mujoco_id,
                model_path,
                burst_mode,
                accelerated_time,
                use_graphics,
                item_controls,
                item_3_controls,
                item_10_controls,
                item_20_controls,
                item_50_controls,
                item_100_controls,
                joint_controls,
                pressure_controls,
                table_geometry,
                robot1_base,
                robot2_base,
                racket1_geometry,
                racket2_geometry);
    }

public:
    char model_path[200];
    bool burst_mode;
    bool accelerated_time;
    bool use_graphics;
    char mujoco_id[200];
    std::vector<MujocoItemControl> item_controls;
    std::vector<MujocoItemsControl<3>> item_3_controls;
    std::vector<MujocoItemsControl<10>> item_10_controls;
    std::vector<MujocoItemsControl<20>> item_20_controls;
    std::vector<MujocoItemsControl<50>> item_50_controls;
    std::vector<MujocoItemsControl<100>> item_100_controls;
    std::vector<MujocoRobotJointControl> joint_controls;
    std::vector<MujocoRobotPressureControl> pressure_controls;
    char table_geometry[100];
    char robot1_base[100];
    char robot2_base[100];
    char racket1_geometry[100];
    char racket2_geometry[100];
};

void set_mujoco_config(const MujocoConfig& config);
bool get_mujoco_config(const std::string& mujoco_id, MujocoConfig& get);
bool wait_for_mujoco_config(const std::string& mujoco_id, MujocoConfig& get);

// if I call this function "wait_for_mujoco" instead of "_wait_for_mujoco",
// then pybind11 (in ../srcpy/wrappers) considers it as an override of another
// function (and fails to compile). I have no idea why, I could not find another
// "wait_for_mujoco" function in this package.
void _wait_for_mujoco(const std::string& mujoco_id);

}  // namespace pam_mujoco
