#pragma once

#include "shared_memory/serializer.hpp"

namespace pam_mujoco
{

  /**
   * Encapsulates the information about a contact
   * (or the abscence of contact). 
   * position and time_stamp
   * have meaning only if contact_occured is true.
   */
  class ContactInformation
  {
  public:
    ContactInformation();
    void register_distance(double d);
    void register_contact(std::array<double,3> position,
			  double time_stamp);
    template <class Archive>
    void serialize(Archive &archive)
    {
      archive(position,
	      contact_occured,
	      time_stamp,
	      minimal_distance);
    }
  public:
    std::array<double,3> position;
    bool contact_occured;
    double time_stamp;
    double minimal_distance;
  };
  
}
