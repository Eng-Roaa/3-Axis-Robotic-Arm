import pyvista as pv
import os

plotter = pv.Plotter(off_screen=True)
colors = ['white', 'yellow', 'red', 'green', 'blue', 'cyan']
for i, name in enumerate(['base_link', 'Link_1', 'Link_2', 'Link_3', 'Link_4', 'Link_5']):
    mesh = pv.read(os.path.abspath(f'Robotics _Arm_URDF/meshes/{name}.STL'))
    plotter.add_mesh(mesh, color=colors[i])

plotter.camera_position = 'iso'
plotter.screenshot('raw_meshes.png')
print("Saved raw_meshes.png")
