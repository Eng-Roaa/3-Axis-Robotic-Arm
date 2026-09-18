import os
import pyvista as pv
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

urdf_path = os.path.abspath('Robotics _Arm_URDF/urdf/Robotics _Arm_URDF.urdf')
robot = rtb.Robot.URDF(urdf_path)
q = np.array([0., 0., 1.57, 0., 0.])

plotter = pv.Plotter(off_screen=True)

import xml.etree.ElementTree as ET
tree = ET.parse(urdf_path)
root = tree.getroot()

colors = {'base_link': 'white', 'Link_1': 'yellow', 'Link_2': 'red', 'Link_3': 'green', 'Link_4': 'blue', 'Link_5': 'cyan'}
base_dir = os.path.dirname(urdf_path)

for link in root.findall('link'):
    name = link.get('name')
    visual = link.find('visual')
    if visual is not None:
        geom = visual.find('geometry')
        if geom is not None:
            mesh_tag = geom.find('mesh')
            if mesh_tag is not None:
                filename = mesh_tag.get('filename')
                if filename.startswith('package://'):
                    pkg_root = os.path.dirname(base_dir) 
                    rel_path = filename.split('package://')[1].split('/', 1)[1]
                    mesh_file = os.path.join(pkg_root, rel_path)
                else:
                    mesh_file = os.path.join(base_dir, filename)

                mesh_file = os.path.abspath(mesh_file)
                
                try:
                    mesh = pv.read(mesh_file)
                    
                    T = robot.fkine(q, end=name)
                    
                    actor = plotter.add_mesh(mesh, color=colors.get(name, 'white'))
                    actor.user_matrix = T.A
                except Exception as e:
                    print(e)

plotter.camera_position = 'iso'
plotter.screenshot('colored_arm.png')
print("Saved colored_arm.png")
