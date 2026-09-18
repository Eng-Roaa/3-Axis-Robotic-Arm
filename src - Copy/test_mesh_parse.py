import xml.etree.ElementTree as ET
import os

urdf_path = os.path.abspath('../Robotics _Arm_URDF/urdf/Robotics _Arm_URDF.urdf')
tree = ET.parse(urdf_path)
root = tree.getroot()

meshes = {}
for link in root.findall('link'):
    name = link.get('name')
    visual = link.find('visual')
    if visual is not None:
        geom = visual.find('geometry')
        if geom is not None:
            mesh = geom.find('mesh')
            if mesh is not None:
                filename = mesh.get('filename')
                # Resolve package://
                filename = filename.replace('package://Robotics _Arm_URDF', '../Robotics _Arm_URDF')
                filename = os.path.abspath(filename)
                
                # Get origin
                origin = visual.find('origin')
                if origin is not None:
                    xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
                    rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
                else:
                    xyz = [0,0,0]
                    rpy = [0,0,0]
                    
                # Get color
                material = visual.find('material')
                color_rgba = [1, 1, 1, 1]
                if material is not None:
                    color = material.find('color')
                    if color is not None:
                        color_rgba = [float(x) for x in color.get('rgba', '1 1 1 1').split()]
                        
                meshes[name] = {
                    'file': filename,
                    'xyz': xyz,
                    'rpy': rpy,
                    'color': color_rgba
                }

print("Found meshes:")
for k, v in meshes.items():
    print(k, v['file'], "exists:", os.path.exists(v['file']), "color:", v['color'])
