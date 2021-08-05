class MujocoItem:

    NO_CONTROL = 0
    CONSTANT_CONTROL = 1
    COMMAND_ACTIVE_CONTROL = 2

    def __init__(self, segment_id, color=None, control=False, contact_type=None):
        self.segment_id = segment_id
        self.color = color
        self.control = control
        self.contact_type = contact_type


class MujocoItems:

    # only these templates exists
    # for the moment
    # in /include/mujoco_config.hpp
    # (and other files)
    accepted_sizes = (3, 10, 20, 50, 100)

    def __init__(self, segment_id):

        self.items = {"balls": [], "goals": [], "hit_points": []}
        self.segment_id = segment_id
        self.contact_type = None
        self.control = None
        self.size = 0

    def iterate(self):
        for ball in self.items["balls"]:
            yield ball
        for goal in self.items["goals"]:
            yield goal
        for hit_points in self.items["hit_points"]:
            yield hit_points
        return

    def all_items(self):
        r = []
        for k in self.items.keys():
            r += self.items[k]
        return r

    def _add(self, item: MujocoItem, category):

        for other in self.all_items():
            if other.control != item.control:
                raise ValueError(
                    "MujocoItems: all added items should be of " "same control type"
                )
            if other.contact_type != item.contact_type:
                raise ValueError(
                    "MujocoItems: all added items should be of " "same contact type"
                )

        if self.contact_type is None:
            self.contact_type = item.contact_type
        if self.control is None:
            self.control = item.control

        self.items[category].append(item)

        self.size += 1

    def add_ball(self, item: MujocoItem):
        self._add(item, "balls")

    def add_goal(self, item: MujocoItem):
        self._add(item, "goals")

    def add_hit_point(self, item: MujocoItem):
        self._add(item, "hit_points")
