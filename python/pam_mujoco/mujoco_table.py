import typing as t

from scipy.spatial.transform import Rotation


class MujocoTable:
    def __init__(
        self,
        segment_id: str,
        position: t.Sequence[float] = (0.4, 1.57, 0.755),
        orientation: t.Optional[Rotation] = None,
        size: t.Sequence[float] = (0.7625, 1.37, 0.02),
    ) -> None:
        if orientation is None:
            orientation = Rotation.identity()

        self.segment_id = segment_id
        self.position = position
        self.orientation = orientation
        self.size = size
