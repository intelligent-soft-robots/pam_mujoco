#pragma once

#include <iostream>
#include "mujoco.h"

namespace pam_mujoco
{
namespace internal
{
class ContactAction
{
public:
    ContactAction();
    bool muted;
    bool in_contact;
};

class ContactLogic
{
public:
    ContactLogic();
    void reset();
    ContactAction apply(const mjModel* m,
                        mjData* d,
                        int index_geo,
                        int index_geom_contactee);

private:
    bool is_muted(bool contact_detected);
    bool is_in_contact(const mjModel* m,
                       mjData* d,
                       int index_geo,
                       int index_geom_contactee);

private:
    int contact_count_;
    bool contact_mode_;
};


class ContactLogic
{
public:
  ContactLogic();
  void reset();
  bool is_in_contact(const mjModel* m,
		     mjData* d,
		     int index_geo,
		     int index_geom_contactee) const;
  bool is_muted() const;
  void get_position(double get_position[3]) const;
  void get_velocity(double get_velocity[3]) const;
private:
  int nb_iters_since_contact_;
  bool muted_;
  double ball_position_[3];
  double ball_velocity_[3];
};
  
}  // namespace internal

}  // namespace pam_mujoco
