#include "pam_mujoco/internal/contact_logic.hpp"

namespace pam_mujoco
{
namespace internal
{
ContactAction::ContactAction() : muted(false), in_contact(false)
{
}

ContactLogic::ContactLogic() : contact_count_(-1), contact_mode_(false)
{
}

void ContactLogic::reset()
{
    contact_count_ = -1;
    contact_mode_ = false;
}

bool ContactLogic::is_in_contact(const mjModel* m,
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

ContactAction ContactLogic::apply(const mjModel* m,
                                  mjData* d,
                                  int index_geom,
                                  int index_geom_contactee)
{
    ContactAction contact_action;

    // a contact has been detected by mujoco during
    // one of the previous iteration and is already
    // handled. Turning off contact detection for
    // 100 iterations
    if(contact_mode_)
      {
	contact_action.in_contact = false;
	// muted means: maybe mujoco
	// is detecting a contact, but
	// we ignore this
	contact_action.muted=true;
	contact_count_++;
	if(contact_count_>=100)
	  {
	    // being ready for a new
	    // contact
	    contact_mode_=false;
	  }
	return contact_action;
      }

    // did mujoco detect a contact ?
    bool contact_detected =
        is_in_contact(m, d, index_geom, index_geom_contactee);

    if(!contact_detected)
      {
	// no new contact 
	contact_action.in_contact=false;
	contact_action.muted=false;
	return contact_action;
      }

    // new contact !
    contact_mode_=true;
    contact_count_=0;    
    contact_action.in_contact = true;
    contact_action.muted = false;

    return contact_action;
}

}  // namespace internal

}  // namespace pam_mujoco
