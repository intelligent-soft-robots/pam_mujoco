#include "pam_mujoco/internal/is_in_contact.hpp"

namespace pam_mujoco
{
  namespace internal
  {
    bool is_in_contact(const mjModel* m,
		       mjData* d,
		       int index_geom,
		       int index_geom_contactee)
    {
      for (int i = 0; i < d->ncon; i++)
	{
	  if (d->contact[i].geom1 == index_geom &&
	      d->contact[i].geom2 == index_geom_contactee)
	    {
	      return true;
	    }
	  if (d->contact[i].geom2 == index_geom &&
	      d->contact[i].geom1 == index_geom_contactee)
	    {
	      return true;
	    }
	}
      return false;
    }
  }
}
