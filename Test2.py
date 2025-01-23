from dm_control import mjcf
from mujoco import MjSim, MjViewer

# Create a new MJCF model
model = mjcf.RootElement()

# Add a ground plane
model.worldbody.add('geom', type='plane', size=[10, 10, 0.1], rgba=[0.8, 0.9, 0.8, 1])

# Add a body with a sphere geom and a free joint
dynamic_body = model.worldbody.add('body', name='dynamic_body', pos=[0, 0, 1])
dynamic_body.add('joint', name='free_joint', type='free')
dynamic_body.add('geom', type='sphere', size=[0.1], rgba=[0.2, 0.6, 0.8, 1])

# Compile the MJCF model into a simulation-ready model
physics = mjcf.Physics.from_mjcf_model(model)

# Create a simulation instance
sim = MjSim(physics.model)

# Visualize the simulation
viewer = MjViewer(sim)

# Run the simulation
while True:
    sim.step()
    viewer.render()