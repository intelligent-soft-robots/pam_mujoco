#include "pam_mujoco/internal/contact_logic.hpp"

namespace pam_mujoco
{

  namespace internal
  {

    ContactLogic::ContactLogic()
      : contact_count_(-1),
	contact_mode_(false)
    {}


    bool ContactLogic::is_in_contact(const mjModel* m, mjData* d,
				     int index_geom,int index_geom_contactee)
    {
      for(int i=0;i<d->ncon;i++)
	{
	  if(d->contact[i].geom1 == index_geom
	     && d->contact[i].geom2 == index_geom_contactee)
	    {
	      return true;
	    }
	  if(d->contact[i].geom2 == index_geom
	     && d->contact[i].geom1 == index_geom_contactee)
	    {
	      return true;
	    }
	}
      return false;
    }


    ContactAction ContactLogic::apply(const mjModel* m, mjData* d,
				      int index_geom,int index_geom_contactee)
    {
      ContactAction contact_action;

      // a contact has been detected by mujoco during one
      // of the previous iteration, triggering the contact_mode_
      // to be true for 4 iterations
      if(contact_mode_)
	{
	  contact_count_++;
	  if(contact_count_>=4)
	    {
	      // contact_mode_ was active for
	      // 4 iteration, leaving it
	      contact_mode_=false;
	      // (not returning)
	    }
	  else
	    {
	      // contact_mode_ still active,
	      // returning that we are in contact
	      contact_action.in_contact = true;
	      return contact_action;
	    }
	}

      // a contact has been detected, but more
      // than 4 iterations ago. We are no longer
      // in contact mode, but maybe we are muted
      // (i.e. new contacts are ignored for 20
      // itertions)
      if(contact_count_>=4 )
	{
	  contact_count_++;
	  if(contact_count_<20)
	    {
	      // still less than 20 iterations
	      // since the last contact, returning
	      // that new contacts are ignored
	      contact_action.muted = true;
	      return contact_action;
	    }
	  else
	    {
	      // more than 20 iterations, resetting
	      // and getting ready for new contacts
	      contact_count_=-1;
	      // (not returning)
	    }
	}
  
      bool contact_detected = is_in_contact(m,d,
					    index_geom,index_geom_contactee);

      // new contact detected
      // entering contact mode
      contact_mode_=true;

      // returning info that we are in contact
      contact_action.in_contact = true;
      return contact_action;

    }

  }

}
