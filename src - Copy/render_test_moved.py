import os
import pyvista as pv
import roboticstoolbox as rtb
import numpy as np

urdf_path = os.path.abspath('Robotics _Arm_URDF/urdf/Robotics _Arm_URDF.urdf')
robot = rtb.Robot.URDF(urdf_path)
q = np.array([1.0, 1.0, 1.0, 1.0, 1.0])

plotter = pv.Plotter(off_screen=True)

import xml.etree.ElementTree as ET
tree = ET.parse(urdf_path)
root = tree.getroot()

base_dir = os.path.dirname(urdf_path)
visual_origins = {}
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
                
                origin_tag = visual.find('origin')
                if origin_tag is not None:
                    xyz = [float(x) for x in origin_tag.get('xyz', '0 0 0').split()]
                    rpy = [float(x) for x in origin_tag.get('rpy', '0 0 0').split()]
                else:
                    xyz = [0, 0, 0]
                    rpy = [0, 0, 0]
                
                from spatialmath import SE3
                visual_tf = SE3(xyz) * SE3.RPY(rpy)
                visual_origins[name] = visual_tf.A

                try:
                    mesh = pv.read(mesh_file)
                    
                    T = robot.fkine(q, end=name)
                    global_tf = T.A @ visual_tf.A
                    
                    actor = plotter.add_mesh(mesh, color='white')
                    actor.user_matrix = global_tf
                except Exception as e:
                    print(e)

plotter.camera_position = 'iso'
plotter.screenshot('moved_arm.png')
print("Saved moved_arm.png")
