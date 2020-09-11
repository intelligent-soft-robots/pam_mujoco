#pragma once

#include "mujoco.h"

namespace pam_mujoco
{

  namespace internal
  {
  
    class ContactAction
    {
    public:
      bool muted;
      bool in_contact;
    };
  
    class ContactLogic
    {
    public:
      ContactLogic();
      ContactAction apply(const mjModel* m, mjData* d,
			  int index_geo, int index_geom_contactee);
    private:
      bool is_muted(bool contact_detected);
      bool is_in_contact(const mjModel* m, mjData* d,
			 int index_geo, int index_geom_contactee);
    private:
      int contact_count_;
      bool contact_mode_;
    };

  }

}
