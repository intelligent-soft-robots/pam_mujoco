

class MujocoItem:

    NO_CONTROL=0
    CONSTANT_CONTROL=1
    COMMAND_ACTIVE_CONTROL=2
    
    def __init__(self,
                 segment_id,
                 color=None,
                 control=False,
                 contact_type=None):
        self.segment_id=segment_id
        self.color=color
        self.control=control
        self.contact_type=contact_type


class MujocoItems:

    # only these templates exists
    # for the moment
    # in /include/mujoco_config.hpp
    # (and other files)
    accepted_sizes = (3,10,20,50,100)
    
    def __init__(self):

        self.items = {"balls":[],
                      "goals":[],
                      "hit_points":[]}

    def all_items(self):
        r = []
        for k in self.items.keys():
            r+=self.items[k]
        return r

    def _add(self, item: MujocoItem, category):

        for other in self.all_items():
            if other.control != item.control:
                raise ValueError("MujocoItems: all added items should be of "
                                 "same control type")
            if other.contact_type != item.contact_type:
                raise ValueError("MujocoItems: all added items should be of "
                                 "same contact type")

        self.items[category].append(item)
    
    def add_ball(self,item: MujocoItem):
        self._add(item,"balls")

    def add_goal(self,item: MujocoItem):
        self._add(item,"goals")

    def add_hit_point(self,item: MujocoItem):
        self._add(item,"hit_points")


        
    
