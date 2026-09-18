import pyvista as pv
import roboticstoolbox as rtb
import numpy as np
import os
import xml.etree.ElementTree as ET

urdf_path = os.path.abspath('Robotics _Arm_URDF/urdf/Robotics _Arm_URDF.urdf')

# Temporarily modify URDF
tree = ET.parse(urdf_path)
root = tree.getroot()
for joint in root.findall('joint'):
    if joint.get('name') == 'Joint_3':
        axis = joint.find('axis')
        axis.set('xyz', '0 -1 0')  # Try -Y axis

temp_urdf = os.path.abspath('temp.urdf')
tree.write(temp_urdf)

robot = rtb.Robot.URDF(temp_urdf)

mesh2 = pv.read(os.path.abspath('Robotics _Arm_URDF/meshes/Link_2.STL'))
mesh3 = pv.read(os.path.abspath('Robotics _Arm_URDF/meshes/Link_3.STL'))

T2 = robot.fkine([0,0,1.57,0,0], end='Link_2').A
T3 = robot.fkine([0,0,1.57,0,0], end='Link_3').A

m2 = mesh2.copy()
m2.transform(T2, inplace=True)
m3 = mesh3.copy()
m3.transform(T3, inplace=True)

print('L2 bounds:', m2.bounds)
print('L3 bounds:', m3.bounds)
os.remove(temp_urdf)
