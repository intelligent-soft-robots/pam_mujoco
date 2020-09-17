import time
import o80
import o80_pam
import pam_mujoco
import pam_models
import numpy as np
import multiprocessing

segment_id = "pressure_control"
mujoco_id = "mj"

model_name = "pressure"
items = pam_mujoco.model_factory(model_name,robot1=True)
robot = items["robot"]

# pam model configuration
pam_model_config_path= pam_models.get_default_config_path()
a_init = [0.5]*8
l_MTC_change_init = [0.0]*8
scale_max_activation = 1.0
scale_max_pressure = 24000
scale_min_activation = 0.001
scale_min_pressure = 6000
pam_model_config = [ segment_id,
                     robot.joint,
                     scale_min_pressure,scale_max_pressure,
                     scale_min_activation, scale_max_activation,
                     pam_model_config_path,pam_model_config_path,
                     a_init,l_MTC_change_init ]

# running the mujoco thread
def execute_mujoco(pam_model_config,
                   robot,
                   mujoco_id,model_name):
    # init mujoco
    pam_mujoco.init_mujoco()
    # adding pressure controller
    pam_mujoco.add_pressure_controller(*pam_model_config)
    # starting the mujoco thread
    pam_mujoco.execute(mujoco_id,model_name)
    # runnign it until requested to stop
    while not pam_mujoco.is_stop_requested(mujoco_id):
        time.sleep(0.01)

# starting the mujoco thread
process  = multiprocessing.Process(target=execute_mujoco,
                                   args=(pam_model_config,robot,
                                         mujoco_id,model_name,))
pam_mujoco.clear(mujoco_id)
process.start()
pam_mujoco.wait_for_mujoco(mujoco_id)

# initializing the o80 frontend for sending
# pressure commands
frontend = o80_pam.FrontEnd(segment_id)

# sending some commands and printing robot state

def go_to(ago_pressure,antago_pressure,duration,dofs=[0,1,2,3]):
    for dof in dofs:
        frontend.add_command(dof,
                             ago_pressure,antago_pressure,
                             o80.Duration_us.milliseconds(duration),
                             o80.Mode.QUEUE)
    frontend.pulse_and_wait()

go_to(20000,20000,1000)
for dof in range(4):
    print("moving dof:",dof)
    go_to(18000,22000,1000,dofs=[dof])
    go_to(22000,18000,1000,dofs=[dof])
    go_to(20000,20000,1500,dofs=[dof])

print("releasing pressure")
go_to(0,0,3000)
time.sleep(3)
    

# ending the mujoco thread
pam_mujoco.request_stop("mj")
frontend.final_burst()
process.join()



