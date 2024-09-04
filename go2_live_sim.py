import time

import mujoco
import mujoco.viewer
from control import *

current_thetas = []
target_thetas = []

def key_listener(keycode):
    #print(keycode)
    if chr(keycode) == ' ':
        global paused
        paused = not paused
    if chr(keycode) == '256':
       endSim = True

def init_controller(model, data):
  for joint in range(0,len(data.qpos)-1):
    current_thetas.append(data.qpos[joint])
    target_thetas.append(data.qpos[joint])

def controller(model, data):
  #pass
  for joint in range(0,len(data.qpos)-1):
    if joint in range(0,len(data.qvel)-1):
      current_thetas[joint] = data.qpos[joint]
      data.qvel[joint] = bot.PID(current_thetas[joint], target_thetas[joint])

#m = mujoco.MjModel.from_xml_path('resources/unitree_go2/scene.xml')
#m = mujoco.MjModel.from_xml_path('resources/trossen_wx250s/scene.xml')
m = mujoco.MjModel.from_xml_path('resources/agility_cassie/scene.xml')
d = mujoco.MjData(m)

bot = Bot(current_thetas)

init_controller(m, d)
mujoco.set_mjcb_control(controller)
paused = False

base_refresh_rate = 100 #Hz
start_time = time.time()
last_refresh = start_time

with mujoco.viewer.launch_passive(m, d, key_callback=key_listener) as viewer:
  start = time.time()
  while viewer.is_running():
    while not paused:
        #runtime limit 15s
        if time.time()-start_time>15:
            stopped = True

        bot.update_time(curr_time)
        
        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
          viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
        
        #time control 
        curr_time = time.time()
        while (curr_time) < (last_refresh + 1.0/base_refresh_rate):
            time.sleep(0.00001)
        time_elapsed = curr_time - last_refresh
        last_refresh = curr_time

        print(f'refresh rate: {1.0/time_elapsed} Hz')
