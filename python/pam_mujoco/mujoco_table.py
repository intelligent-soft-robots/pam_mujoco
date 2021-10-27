class MujocoTable:
    def __init__(
        self,
        segment_id,
        position=[0.1, 0.0, -0.44],
        orientation="-1 0 0 0 -1 0",
        size=[0.7625, 1.37, 0.02],
    ):

        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
        self.size = size
