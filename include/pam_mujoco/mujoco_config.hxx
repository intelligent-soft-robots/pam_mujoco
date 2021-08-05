

template <int NB_ITEMS>
MujocoItemsControl<NB_ITEMS>::MujocoItemsControl()
{
}

template <int NB_ITEMS>
MujocoItemsControl<NB_ITEMS>::MujocoItemsControl(
    std::array<MujocoItemTypes, NB_ITEMS> _type,
    std::string _segment_id,
    std::array<std::string, NB_ITEMS> _joint,
    std::array<int, NB_ITEMS> _index_qpos,
    std::array<int, NB_ITEMS> _index_qvel,
    std::array<std::string, NB_ITEMS> _geometry,
    std::string _robot_geom,
    bool _active_only,
    ContactTypes _contact_type)
    : type{_type},
      index_qpos{_index_qpos},
      index_qvel{_index_qvel},
      active_only{_active_only},
      contact_type{_contact_type}
{
    strcpy(segment_id, _segment_id.c_str());
    strcpy(robot_geom, _robot_geom.c_str());
    for (int i = 0; i < NB_ITEMS; i++)
    {
        strcpy(joint[i], _joint[i].c_str());
        strcpy(geometry[i], _geometry[i].c_str());
    }
}

template <int NB_ITEMS>
std::string MujocoItemsControl<NB_ITEMS>::to_string() const
{
    std::stringstream ss;
    ss << "\t";
    ss << "collection of " << NB_ITEMS << " items:";
    ss << "segment_id: " << segment_id << " ";
    if (contact_type == ContactTypes::table)
    {
        ss << "(interrupted on contact with table) ";
    }
    if (contact_type == ContactTypes::racket1)
    {
        ss << "(interrupted on contact with racket 1) ";
    }
    if (contact_type == ContactTypes::racket2)
    {
        ss << "(interrupted on contact with racket 2) ";
    }
    if (active_only)
    {
        ss << "(active control only) ";
    }
    return ss.str();
}
