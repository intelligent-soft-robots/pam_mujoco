#include "pam_mujoco/mujoco_config.hpp"

namespace pam_mujoco
{
MujocoRobotJointControl::MujocoRobotJointControl()
{
}

MujocoRobotJointControl::MujocoRobotJointControl(std::string _segment_id,
                                                 std::string _joint,
                                                 std::string _racket,
                                                 bool _active_only)
    : active_only{_active_only}
{
    std::strcpy(segment_id, _segment_id.c_str());
    std::strcpy(joint, _joint.c_str());
    std::strcpy(racket, _racket.c_str());
}

std::string MujocoRobotJointControl::to_string() const
{
    std::stringstream ss;
    ss << "\trobot: ";
    ss << "joints control";
    ss << " |\tsegment_id: " << segment_id << " ";
    return ss.str();
}

MujocoRobotPressureControl::MujocoRobotPressureControl()
{
}

MujocoRobotPressureControl::MujocoRobotPressureControl(
    std::string _segment_id,
    std::string _joint,
    bool _active_only,
    std::string _json_controller_path,
    std::string _json_ago_hill_path,
    std::string _json_antago_hill_path)
    : active_only{_active_only}
{
    std::strcpy(segment_id, _segment_id.c_str());
    std::strcpy(joint, _joint.c_str());
    std::strcpy(json_controller_path, _json_controller_path.c_str());
    std::strcpy(json_ago_hill_path, _json_ago_hill_path.c_str());
    std::strcpy(json_antago_hill_path, _json_antago_hill_path.c_str());
}

std::string MujocoRobotPressureControl::to_string() const
{
    std::stringstream ss;
    ss << "\trobot: ";
    ss << "pressure control, ";
    ss << "segment_id: " << segment_id << ", ";
    ss << "configurations:\n";
    ss << "\t\tcontroller: " << json_controller_path << "\n";
    ss << "\t\tagnonist muscles: " << json_ago_hill_path << "\n";
    ss << "\t\tantagonist muscles: " << json_antago_hill_path << "\n";
    return ss.str();
}

MujocoItemControl::MujocoItemControl()
{
}
MujocoItemControl::MujocoItemControl(MujocoItemTypes _type,
                                     std::string _segment_id,
                                     std::string _joint,
                                     std::string _geometry,
                                     bool _active_only,
                                     bool contact_robot1,
                                     bool contact_robot2,
                                     bool contact_table)
    : type{_type},
      active_only{_active_only},
      contact_robot1{contact_robot1},
      contact_robot2{contact_robot2},
      contact_table{contact_table}
{
    strcpy(segment_id, _segment_id.c_str());
    strcpy(joint, _joint.c_str());
    strcpy(geometry, _geometry.c_str());
}

std::string MujocoItemControl::to_string() const
{
    std::stringstream ss;
    ss << "\t";
    if (type == MujocoItemTypes::ball) ss << "ball: ";
    if (type == MujocoItemTypes::hit_point) ss << "hit_point: ";
    if (type == MujocoItemTypes::goal) ss << "goal: ";
    ss << "segment_id: " << segment_id << " ";
    if (contact_table)
    {
        ss << "(interrupted on contact with table) ";
    }
    if (contact_robot1)
    {
        ss << "(interrupted on contact with racket 1) ";
    }
    if (contact_robot2)
    {
        ss << "(interrupted on contact with racket 2) ";
    }
    if (active_only)
    {
        ss << "(active control only) ";
    }
    return ss.str();
}

MujocoConfig::MujocoConfig()
    : burst_mode{false}, accelerated_time{false}, use_graphics{true}, save_data{true}
{
}

MujocoConfig::MujocoConfig(std::string _mujoco_id) : MujocoConfig()
{
    strcpy(mujoco_id, _mujoco_id.c_str());
}

void MujocoConfig::set_burst_mode(bool use_burst)
{
    burst_mode = use_burst;
}

void MujocoConfig::set_accelerated_time(bool use_accelerated)
{
    accelerated_time = use_accelerated;
}

void MujocoConfig::set_graphics(bool graphics)
{
    use_graphics = graphics;
}

void MujocoConfig::set_model_path(std::string path)
{
    strcpy(model_path, path.c_str());
}

std::string MujocoConfig::to_string() const
{
    std::stringstream ss;
    ss << "configuration for: " << mujoco_id << "\n"
       << "\tburst mode: " << burst_mode << "\n"
       << "\tmodel path: " << model_path << "\n"
       << "\taccelerated time: " << accelerated_time << "\n"
       << "\tgraphics: " << use_graphics << "\n";

    for (const MujocoItemControl& mic : item_controls)
    {
        ss << mic.to_string() << "\n";
    }
    for (const MujocoRobotJointControl& mrc : joint_controls)
    {
        ss << mrc.to_string() << "\n";
    }
    for (const MujocoRobotPressureControl& mpc : pressure_controls)
    {
        ss << mpc.to_string() << "\n";
    }
    return ss.str();
}

void MujocoConfig::set_robot1_base(std::string rb1_base)
{
    strcpy(robot1_base, rb1_base.c_str());
}
void MujocoConfig::set_robot2_base(std::string rb2_base)
{
    strcpy(robot2_base, rb2_base.c_str());
}
void MujocoConfig::set_racket_robot1(std::string _racket1_geometry)
{
    strcpy(racket1_geometry, _racket1_geometry.c_str());
}
void MujocoConfig::set_racket_robot2(std::string _racket2_geometry)
{
    strcpy(racket2_geometry, _racket2_geometry.c_str());
}
void MujocoConfig::set_table(std::string _table_geometry)
{
    strcpy(table_geometry, _table_geometry.c_str());
}

void MujocoConfig::add_control(MujocoItemControl mic)
{
    item_controls.push_back(mic);
}

void MujocoConfig::add_control(MujocoRobotJointControl mrc)
{
    joint_controls.push_back(mrc);
}

void MujocoConfig::add_control(MujocoRobotPressureControl mpc)
{
    pressure_controls.push_back(mpc);
}

void MujocoConfig::add_3_control(MujocoItemsControl<3> misc)
{
    item_3_controls.push_back(misc);
}

void MujocoConfig::add_10_control(MujocoItemsControl<10> misc)
{
    item_10_controls.push_back(misc);
}

void MujocoConfig::add_20_control(MujocoItemsControl<20> misc)
{
    item_20_controls.push_back(misc);
}

void MujocoConfig::add_50_control(MujocoItemsControl<50> misc)
{
    item_50_controls.push_back(misc);
}

void MujocoConfig::add_100_control(MujocoItemsControl<100> misc)
{
    item_100_controls.push_back(misc);
}

void set_mujoco_config(const MujocoConfig& config)
{
    shared_memory::serialize(std::string(config.mujoco_id), "config", config);
}

bool get_mujoco_config(const std::string& mujoco_id, MujocoConfig& get)
{
    try
    {
        shared_memory::deserialize(mujoco_id, "config", get);
    }
    catch (...)
    {
        return false;
    }
    return true;
}

bool wait_for_mujoco_config(const std::string& mujoco_id, MujocoConfig& get)
{
    shared_memory::set<bool>(mujoco_id, "exit", false);
    bool received = false;
    while (true)
    {
        received = get_mujoco_config(mujoco_id, get);
        if (received)
        {
            return false;
        }
        else
        {
            usleep(50000);
        }
        bool stop_requested;
        shared_memory::get<bool>(mujoco_id, "exit", stop_requested);
        if (stop_requested) return true;
    }
}

void _wait_for_mujoco(const std::string& mujoco_id)
{
    bool running = false;
    while (!running)
    {
        usleep(50000);
        shared_memory::get<bool>(mujoco_id, "running", running);
    }
}

}  // namespace pam_mujoco
