// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "o80/pybind11_helper.hpp"
#include "pam_mujoco/mujoco_config.hpp"
#include "pam_mujoco/read_robot_state.hpp"
#include "pam_mujoco/contact_ball.hpp"
#include "pam_mujoco/add_controllers.hpp"
#include "shared_memory/serializer.hpp"
#include "context/contact_information.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000


PYBIND11_MODULE(pam_mujoco_wrp, m)
{
    pybind11::class_<pam_mujoco::ReadRobotState>(m, "ReadRobotState")
        .def(pybind11::init<std::string>())
        .def("get_positions", &pam_mujoco::ReadRobotState::get_positions)
        .def("get_velocities", &pam_mujoco::ReadRobotState::get_velocities);

    pybind11::enum_<pam_mujoco::MujocoItemTypes>(m, "MujocoItemTypes")
        .value("ball", pam_mujoco::ball)
        .value("hit_point", pam_mujoco::hit_point)
      .value("goal", pam_mujoco::goal);

    pybind11::enum_<pam_mujoco::ContactTypes>(m, "ContactTypes")
        .value("no_contact", pam_mujoco::no_contact)
        .value("table", pam_mujoco::table)
        .value("racket1", pam_mujoco::racket1)
        .value("racket2", pam_mujoco::racket2);
    
    pybind11::class_<pam_mujoco::MujocoRobotJointControl>(m, "MujocoRobotJointControl")
      .def(pybind11::init<std::string,std::string,bool>())
      .def("__str__",&pam_mujoco::MujocoRobotJointControl::to_string);

    pybind11::class_<pam_mujoco::MujocoRobotPressureControl>(m, "MujocoRobotPressureControl")
      .def(pybind11::init<std::string,std::string,bool,std::string>())
      .def("__str__",&pam_mujoco::MujocoRobotPressureControl::to_string);

    pybind11::class_<pam_mujoco::MujocoItemControl>(m, "MujocoItemControl")
      .def(pybind11::init<pam_mujoco::MujocoItemTypes,std::string,std::string,int,int,std::string,bool,pam_mujoco::ContactTypes>())
      .def("__str__",&pam_mujoco::MujocoItemControl::to_string);
    
    pybind11::class_<pam_mujoco::MujocoConfig>(m, "MujocoConfig")
        .def(pybind11::init<std::string>())
        .def("set_burst_mode", &pam_mujoco::MujocoConfig::set_burst_mode)
        .def("set_model_path", &pam_mujoco::MujocoConfig::set_model_path)
        .def("set_accelerated_time",
             &pam_mujoco::MujocoConfig::set_accelerated_time)
      .def("set_racket_robot1",&pam_mujoco::MujocoConfig::set_racket_robot1)
      .def("set_racket_robot2",&pam_mujoco::MujocoConfig::set_racket_robot2)
      .def("set_table",&pam_mujoco::MujocoConfig::set_table)
      .def("add_control", pybind11::overload_cast<pam_mujoco::MujocoItemControl>(&pam_mujoco::MujocoConfig::add_control))
      .def("add_control", pybind11::overload_cast<pam_mujoco::MujocoRobotJointControl>(&pam_mujoco::MujocoConfig::add_control))
      .def("add_control", pybind11::overload_cast<pam_mujoco::MujocoRobotPressureControl>(&pam_mujoco::MujocoConfig::add_control))
        .def("__str__", &pam_mujoco::MujocoConfig::to_string);

    m.def("set_mujoco_config", &pam_mujoco::set_mujoco_config);
    m.def("wait_for_mujoco", &pam_mujoco::_wait_for_mujoco);
    
    m.def("get_contact", [](std::string segment_id) {
        context::ContactInformation ci;
        shared_memory::deserialize(segment_id, segment_id, ci);
        return ci;
    });

    m.def("reset_contact", &pam_mujoco::reset_contact);
    m.def("activate_contact", &pam_mujoco::activate_contact);
    m.def("deactivate_contact", &pam_mujoco::deactivate_contact);



    

    
}
