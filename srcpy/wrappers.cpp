// Copyright (c) 2020 Max Planck Gesellschaft
// Author : Vincent Berenz

#include "context/contact_information.hpp"
#include "o80/pybind11_helper.hpp"
#include "pam_mujoco/add_controllers.hpp"
#include "pam_mujoco/contact_ball.hpp"
#include "pam_mujoco/extra_balls_extended_state.hpp"
#include "pam_mujoco/mujoco_config.hpp"
#include "pam_mujoco/read_robot_state.hpp"
#include "pam_mujoco/mj_state_tools.hpp"
#include "shared_memory/serializer.hpp"

#define NB_DOFS 4
#define QUEUE_SIZE 500000

template <int NB_BALLS>
void add_extra_balls_extended_state(pybind11::module& m)

{
    typedef pam_mujoco::ExtraBallsExtendedState<NB_BALLS> EES;
    pybind11::class_<EES>(
        m, ("ExtraBallsExtendedState" + std::to_string(NB_BALLS)).c_str())
        .def(pybind11::init<>())
        .def_readonly("contacts", &EES::contacts)
        .def_readonly("episode", &EES::episode)
        .def_readonly("robot_position", &EES::robot_position);
}

PYBIND11_MODULE(pam_mujoco_wrp, m)
{
    add_extra_balls_extended_state<3>(m);
    add_extra_balls_extended_state<10>(m);
    add_extra_balls_extended_state<20>(m);
    add_extra_balls_extended_state<50>(m);
    add_extra_balls_extended_state<100>(m);

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

    pybind11::class_<pam_mujoco::MujocoRobotJointControl>(
        m, "MujocoRobotJointControl")
        .def(pybind11::init<std::string, std::string, std::string, bool>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoRobotJointControl::segment_id)
        .def("__str__", &pam_mujoco::MujocoRobotJointControl::to_string);

    pybind11::class_<pam_mujoco::MujocoRobotPressureControl>(
        m, "MujocoRobotPressureControl")
        .def(pybind11::init<std::string,
                            std::string,
                            bool,
                            std::string,
                            std::string,
                            std::string>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoRobotPressureControl::segment_id)
        .def("__str__", &pam_mujoco::MujocoRobotPressureControl::to_string);

    pybind11::class_<pam_mujoco::MujocoItemControl>(m, "MujocoItemControl")
        .def(pybind11::init<pam_mujoco::MujocoItemTypes,
                            std::string,
                            std::string,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id", &pam_mujoco::MujocoItemControl::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemControl::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemControl::active_only)
        .def("__str__", &pam_mujoco::MujocoItemControl::to_string);

    pybind11::class_<pam_mujoco::MujocoItemsControl<3>>(m,
                                                        "Mujoco3ItemsControl")
        .def(pybind11::init<std::array<pam_mujoco::MujocoItemTypes, 3>,
                            std::string,
                            std::array<std::string, 3>,
                            std::array<std::string, 3>,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoItemsControl<3>::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemsControl<3>::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemsControl<3>::active_only)
        .def("__str__", &pam_mujoco::MujocoItemsControl<3>::to_string);

    pybind11::class_<pam_mujoco::MujocoItemsControl<10>>(m,
                                                         "Mujoco10ItemsControl")
        .def(pybind11::init<std::array<pam_mujoco::MujocoItemTypes, 10>,
                            std::string,
                            std::array<std::string, 10>,
                            std::array<std::string, 10>,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoItemsControl<10>::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemsControl<10>::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemsControl<10>::active_only)
        .def("__str__", &pam_mujoco::MujocoItemsControl<10>::to_string);

    pybind11::class_<pam_mujoco::MujocoItemsControl<20>>(m,
                                                         "Mujoco20ItemsControl")
        .def(pybind11::init<std::array<pam_mujoco::MujocoItemTypes, 20>,
                            std::string,
                            std::array<std::string, 20>,
                            std::array<std::string, 20>,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoItemsControl<20>::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemsControl<20>::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemsControl<20>::active_only)
        .def("__str__", &pam_mujoco::MujocoItemsControl<20>::to_string);

    pybind11::class_<pam_mujoco::MujocoItemsControl<50>>(m,
                                                         "Mujoco50ItemsControl")
        .def(pybind11::init<std::array<pam_mujoco::MujocoItemTypes, 50>,
                            std::string,
                            std::array<std::string, 50>,
                            std::array<std::string, 50>,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoItemsControl<50>::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemsControl<50>::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemsControl<50>::active_only)
        .def("__str__", &pam_mujoco::MujocoItemsControl<50>::to_string);

    pybind11::class_<pam_mujoco::MujocoItemsControl<100>>(
        m, "Mujoco100ItemsControl")
        .def(pybind11::init<std::array<pam_mujoco::MujocoItemTypes, 100>,
                            std::string,
                            std::array<std::string, 100>,
                            std::array<std::string, 100>,
                            std::string,
                            bool,
                            pam_mujoco::ContactTypes>())
        .def_readonly("segment_id",
                      &pam_mujoco::MujocoItemsControl<100>::segment_id)
        .def_readonly("type", &pam_mujoco::MujocoItemsControl<100>::type)
        .def_readonly("active_only",
                      &pam_mujoco::MujocoItemsControl<100>::active_only)
        .def("__str__", &pam_mujoco::MujocoItemsControl<100>::to_string);

    pybind11::class_<pam_mujoco::MujocoConfig>(m, "MujocoConfig")
        .def(pybind11::init<std::string>())
        .def("set_burst_mode", &pam_mujoco::MujocoConfig::set_burst_mode)
        .def("set_model_path", &pam_mujoco::MujocoConfig::set_model_path)
        .def("set_accelerated_time",
             &pam_mujoco::MujocoConfig::set_accelerated_time)
        .def("set_graphics", &pam_mujoco::MujocoConfig::set_graphics)
        .def("set_save_data", &pam_mujoco::MujocoConfig::set_save_data)
        .def("set_save_folder", &pam_mujoco::MujocoConfig::set_save_folder)
        .def("set_robot1_base", &pam_mujoco::MujocoConfig::set_robot1_base)
        .def("set_robot2_base", &pam_mujoco::MujocoConfig::set_robot2_base)
        .def("set_racket_robot1", &pam_mujoco::MujocoConfig::set_racket_robot1)
        .def("set_racket_robot2", &pam_mujoco::MujocoConfig::set_racket_robot2)
        .def("set_table", &pam_mujoco::MujocoConfig::set_table)
        .def("add_control",
             pybind11::overload_cast<pam_mujoco::MujocoItemControl>(
                 &pam_mujoco::MujocoConfig::add_control))
        .def("add_control",
             pybind11::overload_cast<pam_mujoco::MujocoRobotJointControl>(
                 &pam_mujoco::MujocoConfig::add_control))
        .def("add_control",
             pybind11::overload_cast<pam_mujoco::MujocoRobotPressureControl>(
                 &pam_mujoco::MujocoConfig::add_control))
        .def("add_3_control", &pam_mujoco::MujocoConfig::add_3_control)
        .def("add_10_control", &pam_mujoco::MujocoConfig::add_10_control)
        .def("add_20_control", &pam_mujoco::MujocoConfig::add_20_control)
        .def("add_50_control", &pam_mujoco::MujocoConfig::add_50_control)
        .def("add_100_control", &pam_mujoco::MujocoConfig::add_100_control)
        .def_readonly("item_controls", &pam_mujoco::MujocoConfig::item_controls)
        .def_readonly("item_3_controls",
                      &pam_mujoco::MujocoConfig::item_3_controls)
        .def_readonly("item_10_controls",
                      &pam_mujoco::MujocoConfig::item_10_controls)
        .def_readonly("item_20_controls",
                      &pam_mujoco::MujocoConfig::item_20_controls)
        .def_readonly("item_50_controls",
                      &pam_mujoco::MujocoConfig::item_50_controls)
        .def_readonly("item_100_controls",
                      &pam_mujoco::MujocoConfig::item_100_controls)
        .def_readonly("joint_controls",
                      &pam_mujoco::MujocoConfig::joint_controls)
        .def_readonly("pressure_controls",
                      &pam_mujoco::MujocoConfig::pressure_controls)
        .def("__str__", &pam_mujoco::MujocoConfig::to_string);

    m.def("set_mujoco_config", &pam_mujoco::set_mujoco_config);
    m.def("wait_for_mujoco", &pam_mujoco::_wait_for_mujoco);

    m.def("get_mujoco_config",
          [](std::string mujoco_id)
          {
              pam_mujoco::MujocoConfig config;
              pam_mujoco::get_mujoco_config(mujoco_id, config);
              return config;
          });

    m.def("get_contact",
          [](std::string segment_id)
          {
              context::ContactInformation ci;
              shared_memory::deserialize(segment_id, segment_id, ci);
              return ci;
          });

    m.def("reset_contact", &pam_mujoco::reset_contact);
    m.def("activate_contact", &pam_mujoco::activate_contact);
    m.def("deactivate_contact", &pam_mujoco::deactivate_contact);

    o80::create_python_bindings<QUEUE_SIZE,
                                3,
                                o80::Item3dState,
                                pam_mujoco::ExtraBallsExtendedState<3>,
                                o80::NO_EXTENDED_STATE,
                                o80::NO_STATE,
                                o80::NO_INTROSPECTOR>(m, std::string("Balls3"));

    o80::create_python_bindings<QUEUE_SIZE,
                                10,
                                o80::Item3dState,
                                pam_mujoco::ExtraBallsExtendedState<10>,
                                o80::NO_EXTENDED_STATE,
                                o80::NO_STATE,
                                o80::NO_INTROSPECTOR>(m,
                                                      std::string("Balls10"));

    o80::create_python_bindings<QUEUE_SIZE,
                                20,
                                o80::Item3dState,
                                pam_mujoco::ExtraBallsExtendedState<20>,
                                o80::NO_EXTENDED_STATE,
                                o80::NO_STATE,
                                o80::NO_INTROSPECTOR>(m,
                                                      std::string("Balls20"));

    o80::create_python_bindings<QUEUE_SIZE,
                                50,
                                o80::Item3dState,
                                pam_mujoco::ExtraBallsExtendedState<50>,
                                o80::NO_EXTENDED_STATE,
                                o80::NO_STATE,
                                o80::NO_INTROSPECTOR>(m,
                                                      std::string("Balls50"));

    o80::create_python_bindings<QUEUE_SIZE,
                                100,
                                o80::Item3dState,
                                pam_mujoco::ExtraBallsExtendedState<100>,
                                o80::NO_EXTENDED_STATE,
                                o80::NO_STATE,
                                o80::NO_INTROSPECTOR>(m,
                                                      std::string("Balls100"));
}
