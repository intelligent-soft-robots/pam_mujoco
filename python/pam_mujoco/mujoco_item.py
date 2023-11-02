import pam_mujoco
from typing import Optional


def _contact_from_contact_type(item: "MujocoItem", contact_type: Optional[int]) -> None:
    # for retro-compatibility
    if contact_type is None:
        return
    if contact_type == pam_mujoco.ContactTypes.racket1:
        item.contact_robot1 = True
    elif contact_type == pam_mujoco.ContactTypes.racket2:
        item.contact_robot2 = True
    elif contact_type == pam_mujoco.ContactTypes.table:
        item.contact_table = True


class MujocoItem:
    NO_CONTROL = 0
    CONSTANT_CONTROL = 1
    COMMAND_ACTIVE_CONTROL = 2

    def __init__(
        self,
        segment_id,
        color=None,
        control=False,
        contact_robot1=False,
        contact_robot2=False,
        contact_table=False,
        contact_type=None,
    ):
        self.segment_id = segment_id
        self.color = color
        self.control = control
        _contact_from_contact_type(self, contact_type)
        self.contact_robot1 = contact_robot1
        self.contact_robot2 = contact_robot2
        self.contact_table = contact_table


class MujocoItems:
    # only these templates exists
    # for the moment
    # in /include/mujoco_config.hpp
    # (and other files)
    accepted_sizes = (3, 10, 20, 50, 100)

    def __init__(
        self,
        segment_id,
        contact_robot1=False,
        contact_robot2=False,
        contact_table=False,
        contact_type=None,
        control=False,
    ):
        self.items = {"balls": [], "goals": [], "hit_points": []}
        self.segment_id = segment_id
        # for retro-compatibility
        _contact_from_contact_type(self, contact_type)
        self.contact_robot1 = contact_robot1
        self.contact_robot2 = contact_robot2
        self.contact_table = contact_table
        self.control = control
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

    def _same_contact_type(self, other: MujocoItem) -> bool:
        for attr in ("contact_robot1", "contact_robot2", "contact_table"):
            if getattr(self, attr) != getattr(other, attr):
                return False
        return True

    def _add(self, item: MujocoItem, category):
        if not self.control == item.control:
            raise ValueError(
                "MujocoItems: all added items should be of " "same control type"
            )
        if not self._same_contact_type(item):
            raise ValueError(
                "MujocoItems: all added items should be of " "same contact type"
            )
        self.items[category].append(item)
        self.size += 1

    def add_ball(self, item: MujocoItem):
        self._add(item, "balls")

    def add_goal(self, item: MujocoItem):
        self._add(item, "goals")

    def add_hit_point(self, item: MujocoItem):
        self._add(item, "hit_points")
