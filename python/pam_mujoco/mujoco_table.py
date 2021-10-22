class MujocoTable:

    def __init__(
        self,
        segment_id,
        position=[0.1, 0.0, -0.44],
        orientation=None,
    ):

        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
