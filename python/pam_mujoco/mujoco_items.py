

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

