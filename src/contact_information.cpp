#include "pam_mujoco/contact_information.hpp"

namespace pam_mujoco
{
  ContactInformation::ContactInformation()
    : position{0,0,0},
      contact_occured(false),
      minimal_distance(-1),
      time_stamp(-1)
  {}

  void ContactInformation::register_distance(double d)
  {
    if(minimal_distance<0)
      {
	minimal_distance=d;
	return;
      }
    minimal_distance = std::min(minimal_distance,
				d);
  }

  void ContactInformation::register_contact(std::array<double,3> _position,
					    double _time_stamp)
  {
    contact_occured=true;
    minimal_distance = 0;
    position = _position;
    time_stamp = _time_stamp;
  }

}
