import pam_mujoco
import time

model = "test"
mujoco_id = "test"

pam_mujoco.generate_model(model,{"robot":([0,0,0],None)})
pam_mujoco.init_mujoco()
pam_mujoco.execute(mujoco_id,model)

time_start = time.time()
while time.time()-time_start < 5:
    time.sleep(0.01)

pam_mujoco.request_stop(mujoco_id)
