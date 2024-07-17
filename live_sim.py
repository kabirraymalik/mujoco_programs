with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

  # Enable wireframe rendering of the entire scene.
  viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
  viewer.sync()

  while viewer.is_running():
    ...
    # Step the physics.
    mujoco.mj_step(m, d)

    # Add a 3x3x3 grid of variously colored spheres to the middle of the scene.
    viewer.user_scn.ngeom = 0
    i = 0
    for x, y, z in itertools.product(*((range(-1, 2),) * 3)):
      mujoco.mjv_initGeom(
          viewer.user_scn.geoms[i],
          type=mujoco.mjtGeom.mjGEOM_SPHERE,
          size=[0.02, 0, 0],
          pos=0.1*np.array([x, y, z]),
          mat=np.eye(3).flatten(),
          rgba=0.5*np.array([x + 1, y + 1, z + 1, 2])
      )
      i += 1
    viewer.user_scn.ngeom = i
    viewer.sync()
    ...