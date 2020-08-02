#pragma once

#include <map>

namespace pam_mujoco
{

  class ControllerBase;
  
  typedef std::map<int,ControllerBase*> ControllerInstances;
  ControllerInstances controller_instances_g;

  typedef std::map<int,ForceComputeBase*> ForceComputeInstances;
  ForceComputeInstances force_compute_instances_g;
  
  
  class ControllerBase
  {
  public:
    ControllerBase(int controller_index)
    {
      controller_instances_g.insert(
				    std::pair<int,
				    ControllerBase*>(controller_index,
						     this) );
    }
    virtual void control(const mjModel* m,
			 mjData* d)=0;
  };

  class ForceComputeBase
  {
  public:
    ForceComputeBase(int force_index)
    {
      force_compute_instances_g.insert(
				    std::pair<int,
				    ForceComputeBase*>(force_index,
						       this) );
    }
    virtual void get_force(const mjModel* m,
			   mjData* d,
			   int id)=0;
  };

  
  template<int CONTROLLER_INDEX>
  virtual void control(const mjModel* m,
		       mjData* d)
  {
    static ControllerBase* controller =
      controller_instances_g[CONTROLLER_INDEX];
    controller->control(m,d);
  }

  
  template<int FORCE_INDEX>
  virtual void get_force(const mjModel* m,
		       mjData* d)
  {
    static ForceComputeBase* force_compute =
      force_compute_instances_g[FORCE_INDEX];
    force_compute->get_force(m,d);
  }

  

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
