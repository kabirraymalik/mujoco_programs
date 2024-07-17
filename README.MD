note: if GLFW shader/render issue, make sure to edit BASHRC to include:

export LD_LIBRARY_PATH=/home/kabir/.mujoco/mujoco210/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export PATH="$LD_LIBRARY_PATH:$PATH"
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

Note: had to rename mujoco-3.6.1 to mujoco210 for this to work?
Useful video here: https://www.youtube.com/watch?v=Wnb_fiStFb8