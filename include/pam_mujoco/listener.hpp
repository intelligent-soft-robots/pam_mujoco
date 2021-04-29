#pragma once

#include "shared_memory/shared_memory.hpp"

namespace pam_mujoco
{

  class Listener
  {

  public:
    Listener(std::string segment_id,
	     std::string object_id)
      : segment_id_{segment_id},
	object_id_{object_id}
    {
      shared_memory::set<bool>(segment_id,
			       object_id,
			       false);
    }
    bool do_once()
    {
      bool v;
      shared_memory::get<bool>(segment_id_,
			       object_id_,
			       v);
      if(v)
	{
	  shared_memory::set<bool>(segment_id_,
				   object_id_,
				   false);
	}
      return v;
      
    }

    bool is_on()
    {
      bool v;
      shared_memory::get<bool>(segment_id_,
			       object_id_,
			       v);
      return v;
    }

  private:
    std::string segment_id_;
    std::string object_id_;
  };
  

}
