import time

import mujoco
import mujoco.viewer

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
   pass

def controller(model, data):
   data.ctrl[0] = 15
   data.ctrl[1] = 15
   data.ctrl[2] = 15


   #print output
   print(f"{data.qpos[0]} | {data.qpos[1]} | {data.qpos[2]} | {data.qpos[3]}")

def init_controller(model, data):
  for joint in range(0,len(data.qpos)-1):
    current_thetas.append(data.qpos[joint])
    target_thetas.append(data.qpos[joint])

def controller(model, data):
  pass
"""
  kp = 0.01
  for joint in range(0,len(data.qpos)-1):
    current_thetas[joint] = data.qpos[joint]
    error = target_thetas[joint] - current_thetas[joint]
    vel = kp * error
    data.qvel[joint] = vel
"""

#m = mujoco.MjModel.from_xml_path('resources/unitree_go2/scene.xml')
m = mujoco.MjModel.from_xml_path('resources/trossen_wx250s/scene.xml')
d = mujoco.MjData(m)

init_controller(m, d)
mujoco.set_mjcb_control(controller)
paused = False

"""
with mujoco.viewer.launch_passive(m, d, key_callback=key_listener) as viewer:
  start = time.time()
  while viewer.is_running():
    step_start = time.time()
    if not paused:

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
"""

base_refresh_rate = 1000 #Hz
start_time = time.time()
last_refresh = start_time

with mujoco.viewer.launch_passive(m, d, key_callback=key_listener) as viewer:
  start = time.time()
  while viewer.is_running():
    while not paused:
        #runtime limit 15s
        if time.time()-start_time>15:
            stopped = True

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
          viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
        
        #time control 
        while (time.time()) < (last_refresh + 1.0/base_refresh_rate):
            time.sleep(0.00001)
        curr_time = time.time()
        time_elapsed = curr_time - last_refresh
        last_refresh = curr_time
        print(f'refresh rate: {1.0/time_elapsed} Hz')
