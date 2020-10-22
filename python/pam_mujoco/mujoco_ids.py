import time
import pam_mujoco

class mujoco_ids:

    robot = "pam_robot_mujoco"

    @classmethod
    def wait_for_mujoco(cls,mujoco_id):
        try :
            started = False 
            while not started:
                time.sleep(0.01)
                try :
                    started = pam_mujoco.has_mujoco_started(mujoco_id)
                except Exception as e :
                    started = False
        except KeyboardInterrupt:
            return False
        return True
    

    
            
