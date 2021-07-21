#include "gtest/gtest.h"
#include "pam_mujoco/internal/recompute_state_after_contact.hpp"

using namespace pam_mujoco;

template<class T>
void print3d(T& t)
{
  for(int dim=0; dim<3; dim++)
    {
      std::cout << t[dim];
      if(dim!=2)
	{
	  std::cout << ",";
	}
    }
}

void print_results(const internal::ContactStates& previous,
		   const internal::ContactStates& current,
		   double* out_position,
		   double* out_velocity)
{
  std::cout << "\n\n";
  std::cout << "previous:\t" << "position: ";
  print3d(previous.ball_position);
  std::cout << "\tvelocity: ";
  print3d(previous.ball_velocity);
  std::cout << std::endl;
  std::cout << "current:\t" << "position: ";
  print3d(current.ball_position);
  std::cout << "\tvelocity: ";
  print3d(current.ball_velocity);
  std::cout << std::endl;
  std::cout << "out:\t" << "position: ";
  print3d(out_position);
  std::cout << "\tvelocity: ";
  print3d(out_velocity);
  std::cout << std::endl;
  std::cout << "\n\n";
}


class ContactCase
{
public:
  ContactCase(){}
public:
  internal::ContactStates previous;
  internal::ContactStates current;
};

void set_horizontal_immobile_contactee(ContactCase& contact_case)
{
  std::array<double,9> orientation;
  orientation.fill(0);
  orientation[0]=1;
  orientation[4]=1;
  orientation[8]=1;
  std::array<double,3> position;
  position[0]=0.8;
  position[1]=1.7;
  position[2]=-0.475;
  std::array<double,3> velocity;
  velocity.fill(0);
  contact_case.previous.contactee_orientation=orientation;
  contact_case.previous.contactee_position=position;
  contact_case.previous.contactee_velocity=velocity;
}

void set_values(std::array<double,3>& a, double v1, double v2, double v3)
{
  a[0]=v1;
  a[1]=v2;
  a[2]=v3;
}

// note: these contact cases reproduce the values
//       of pam_demos/contacts/table_contacts.py

ContactCase get_contact_case1()
{
  ContactCase ccase;
  set_horizontal_immobile_contactee(ccase);
  set_values(ccase.previous.ball_position,1.06232,0.876756,-0.434885);
  set_values(ccase.previous.ball_velocity,0.0621983,-0.621835,-0.952241);
  set_values(ccase.current.ball_position,1.06252,0.874752,-0.437891);
  set_values(ccase.current.ball_velocity,0.0623983,-0.623834,-0.955238);
  return ccase;
}

ContactCase get_contact_case2()
{
  ContactCase ccase;
  set_horizontal_immobile_contactee(ccase);
  set_values(ccase.previous.ball_position,1.25095,0.877378,-0.433933);
  set_values(ccase.previous.ball_velocity,-0.248782,-0.621907,-0.942636);
  set_values(ccase.current.ball_position,1.25015,0.875376,-0.436936);
  set_values(ccase.current.ball_velocity,-0.249583,-0.623902,-0.945639);
  return ccase;
}

ContactCase get_contact_case3()
{
  ContactCase ccase;
  set_horizontal_immobile_contactee(ccase);
  set_values(ccase.previous.ball_position,0.873573,0.877378,-0.433933);
  set_values(ccase.previous.ball_velocity,0.373131,-0.621939,-0.942591);
  set_values(ccase.current.ball_position,0.874774,0.875376,-0.436936);
  set_values(ccase.current.ball_velocity,0.374331,-0.62394,-0.945596);
  return ccase;
}

ContactCase get_contact_case4()
{
  ContactCase ccase;
  set_horizontal_immobile_contactee(ccase);
  set_values(ccase.previous.ball_position,1.1,0.5,-0.433933);
  set_values(ccase.previous.ball_velocity,-1.25239e-05,3.60857e-05,-0.942676);
  set_values(ccase.current.ball_position,1.1,0.5,-0.436936);
  set_values(ccase.current.ball_velocity,1.1,0.5,-0.436936);
  return ccase;
}

ContactCase get_contact_case5()
{
  ContactCase ccase;
  set_horizontal_immobile_contactee(ccase);
  set_values(ccase.previous.ball_position,0.684884,0.424524,-0.433933);
  set_values(ccase.previous.ball_velocity,0.684014,0.124406,-0.942633);
  set_values(ccase.current.ball_position,0.687086,0.424925,-0.436936);
  set_values(ccase.current.ball_velocity,0.686212,0.124805,-0.945631);
  return ccase;
}

std::array<ContactCase,5> get_contact_cases()
{
  std::array<ContactCase,5> cases;
  cases[0]=get_contact_case1();
  cases[1]=get_contact_case2();
  cases[2]=get_contact_case3();
  cases[3]=get_contact_case4();
  cases[4]=get_contact_case5();
  return cases;
}



class ContactsTests: public ::testing::Test
{
};


TEST_F(ContactsTests, table_bouncing_up)
{

  internal::RecomputeStateConfig config = internal::get_table_recompute_config();
    
  std::array<ContactCase,5> cases = get_contact_cases();

  double get_position[3];
  double get_velocity[3];
  
  for(const ContactCase& c: cases)
    {
      recompute_state_after_contact(config,
				    c.previous,
				    c.current,
				    get_position,
				    get_velocity);

      print_results(c.previous,c.current,get_position,get_velocity);
      
      // bouncing up, not down
      ASSERT_GT(get_position[2],c.current.ball_position[2]);
      ASSERT_GT(get_velocity[2],0);
    }

}


  
