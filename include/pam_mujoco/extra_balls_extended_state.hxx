
template<int NB_BALLS>
ExtraBallsExtendedState<NB_BALLS>::ExtraBallsExtendedState()
    : episode{-1}
  {
    contacts.fill(false);
    robot_position.fill(0);
  }
  
template<int NB_BALLS>
ExtraBallsExtendedState<NB_BALLS>::ExtraBallsExtendedState(const std::array<bool,NB_BALLS>& _contacts,
							   long int _episode,
							   const std::array<double,3>& _robot_position)
    : contacts{_contacts},
      episode{_episode},
      robot_position{_robot_position}
  {}
