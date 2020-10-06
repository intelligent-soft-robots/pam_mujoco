#pragma once

#include "pam_interface/state/robot.hpp"
#include "pam_models/hill/factory.hpp"
#include "o80_pam/actuator_state.hpp"
#include "pam_mujoco/controllers.hpp"

namespace pam_mujoco
{

  template<int QUEUE_SIZE, int NB_DOFS>
  class PressureController : public ActuatorBiasBase, // get_bias function
                             public ControllerBase // apply function
    
  {
  public:
    typedef pam_models::hill::Muscle Muscle;
    typedef o80::BackEnd<QUEUE_SIZE,
			 NB_DOFS*2,
			 o80_pam::ActuatorState,
			 pam_interface::RobotState<NB_DOFS>> Backend;
    typedef o80::States<2*NB_DOFS,o80_pam::ActuatorState> States;
  public:
    PressureController(std::string segment_id,
		       std::string robot_joint_base,
		       int scale_min_pressure, int scale_max_pressure,
		       int scale_min_activation, int scale_max_activation,
		       std::string muscle_json_config_path_ago,
		       std::string muscle_json_config_path_antago,
		       std::array<double,NB_DOFS*2> a_init,
		       std::array<double,NB_DOFS*2> l_MTC_change_init);
    void apply(const mjModel* m,
	       mjData* d);
    mjtNum get_bias(const mjModel* m, const mjData* d, int id);
  public:
    static void clear(std::string segment_id);
  private:
    double pressure2activation(double pressure);
    double activation2pressure(double activation);
  private:
    Backend backend_;
    std::string robot_joint_base_;
    int index_q_robot_;
    int index_qvel_robot_;
    double scale_min_pressure_;
    double scale_min_activation_;
    double scale_ratio_;
    std::vector<Muscle> muscles_;
    std::array<double,NB_DOFS*2> bias_forces_;
    int iteration_;
    States states_;
  };

#include "pressure_controller.hxx"
  
}
