class MujocoTable:
    def __init__(
        self,
        segment_id,
        position=[+0.4, +1.57, 0.755],
        orientation="0 0 0",
        size=[0.7625, 1.37, 0.02],
    ):

        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
        self.size = size
