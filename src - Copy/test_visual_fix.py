import pyvista as pv
import roboticstoolbox as rtb
import numpy as np
import os
import xml.etree.ElementTree as ET

urdf_path = os.path.abspath('Robotics _Arm_URDF/urdf/Robotics _Arm_URDF.urdf')

# Modify URDF temporarily
tree = ET.parse(urdf_path)
root = tree.getroot()

for link in root.findall('link'):
    if link.get('name') == 'Link_3':
        visual = link.find('visual')
        origin = visual.find('origin')
        origin.set('xyz', '0.25316092 -0.24960879 -0.03800183')
        origin.set('rpy', '3.14159265 0 1.5708')

temp_urdf = os.path.abspath('temp_fix.urdf')
tree.write(temp_urdf)

robot = rtb.Robot.URDF(temp_urdf)
q = np.array([0, 0, 1.57, 0, 0])

plotter = pv.Plotter(off_screen=True)
colors = {'base_link': 'white', 'Link_1': 'yellow', 'Link_2': 'red', 'Link_3': 'green', 'Link_4': 'blue', 'Link_5': 'cyan'}
base_dir = os.path.dirname(urdf_path)

from spatialmath import SE3

for link in root.findall('link'):
    name = link.get('name')
    visual = link.find('visual')
    if visual is not None:
        geom = visual.find('geometry')
        mesh_tag = geom.find('mesh')
        filename = mesh_tag.get('filename')
        rel_path = filename.split('package://')[1].split('/', 1)[1]
        mesh_file = os.path.join(os.path.dirname(base_dir), rel_path)
        
        origin_tag = visual.find('origin')
        xyz = [float(x) for x in origin_tag.get('xyz', '0 0 0').split()]
        rpy = [float(x) for x in origin_tag.get('rpy', '0 0 0').split()]
        visual_tf = SE3(xyz) * SE3.RPY(rpy)
        
        mesh = pv.read(os.path.abspath(mesh_file))
        T = robot.fkine(q, end=name)
        global_tf = T.A @ visual_tf.A
        
        actor = plotter.add_mesh(mesh, color=colors.get(name, 'white'))
        actor.user_matrix = global_tf

plotter.camera_position = 'iso'
plotter.screenshot('fixed_arm.png')
print("Saved fixed_arm.png")
os.remove(temp_urdf)
