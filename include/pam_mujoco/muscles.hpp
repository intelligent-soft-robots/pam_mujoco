#pragma once

#include "pam_mujoco/controllers.hpp"
#include "pam_mujoco/force_compute.hpp"


namespace pam_mujoco
{


  typedef std::vector<std::tuple<double,double>> FDotLCE;
  
  class MusclesController : public ControllerBase,
			    public ForceComputeBase
  {
  public:
    template<int CONTROLLER_INDEX, int FORCE_INDEX>
    MusclesController(int nb_dofs,
		      std::string &json_params1,
		      std::String &json_params2,
		      std::vector<double>& a_init)
      : ControllerBase(CONTROLLER_INDEX),
	ForceComputeBase(FORCE_INDEX),
	nb_dofs_{nb_dofs}
    {
      for(size_t i=0;i<nb_dofs;i++)
	{
	  muscles_.push_back(pam_models::hill:from_json(json_params1,
							a_init[i],0));
	}
      for(size_t i=0;i<nb_dofs;i++)
	{
	  muscles_.push_back(pam_models::hill:from_json(json_params2,
							a_init[nb_dofs+i],0));
	}
      muscles.resize(nb_dofs*2);
      f_dot_lce_.resize(nb_dofs*2)
    }
    void getForce(const mjModel* m,
		  const mjData* d,
		  int id)
    {
      return f_dot_lce_[id];
    }
    void control(const mjModel* m,
		 mjData* d)
    {
      for(int i=0;i<nb_dofs_*2;i++)
	{
	  int id_activation = i;
	  int id_pam = i + N_MUSCLES;
	  double l_MTC = d->actuator_length[id_pam];
	  double dot_l_MTC = d->actuator_velocity[id_pam];
	  double a = d->act[id_activation];
	  double l_CE = d->act[id_pam];
	  f_dot_lce_[id_pam] = muscle.get(l_MTC,dot_l_MTC,
				 a,l_CE);
	  d->ctrl[id_pam]=std::get<1>(f_dot_lce_[id_pam]);
	}
    }
   
  private:
    int nb_dofs_;
    std::vector<pam_models::hill::Muscle> muscles_;
    FDotLCE f_dot_lce_;
  };


}
