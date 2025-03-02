# %%
import mujoco
import mujoco
import mujoco.viewer 
import time
import numpy as np

#sys.path.append("")
xml_path = '/home/dan/source/sim-assets/sim-robots/custom/rainbow/VR_teleop/rby1a/mujoco/model_act.xml'

# %%
model = mujoco.MjModel.from_xml_path(str(xml_path))
body_names = [model.names[model.name_bodyadr[i]:].split(b'\x00')[0].decode('utf-8') for i in range(model.nbody)]    
body_name="cylinder"
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
joint_qpos_index = model.jnt_qposadr[model.body_jntadr[body_id]]

#%%


# model.geom_size[body_id][1]
#%%

# output_path = "modified_model.xml"
# mujoco.mj_saveLastXML(output_path, model)
# %%
timestep=model.opt.timestep
model.opt.solver=0

data = mujoco.MjData(model)
oldpos=data.qpos[joint_qpos_index:joint_qpos_index + 3]
#newpos=np.array(oldpos[0],[oldpos[1]+1],oldpos[2]) 
#data.qpos[joint_qpos_index:joint_qpos_index + 3] = newpos

simulation_time = 5.0  # Total simulation time in seconds
frame_skip = 10  # Number of steps between rendering frames
data.actuator('left_arm_2_act').ctrl =np.pi/6
data.actuator('left_arm_1_act').ctrl =-np.pi/6
#%

# %%
with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  q=0
  i=0 

  while viewer.is_running():
    step_start = time.time()
    
    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    maxangle=-np.pi/6 
    data.actuator('left_arm_1_act').ctrl =-np.pi/6+i
    data.actuator('left_arm_3_act').ctrl =1.2*i
    data.actuator('left_arm_4_act').ctrl =2*i
    data.actuator('right_arm_1_act').ctrl =-np.pi/6+i
    data.actuator('right_arm_2_act').ctrl= maxangle
    data.actuator('right_arm_3_act').ctrl =-1.2*i
    data.actuator('right_arm_4_act').ctrl =2*i
    for _ in range(frame_skip):
        mujoco.mj_step(model, data)
    
    # Example modification of a viewer option: toggle contact points every two seconds.
    #with viewer.lock():
      #viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
    # print("working")
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
   # viewer.render()
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)

    i=i-0.001
   
    if i< maxangle:
      i=maxangle
# %%
