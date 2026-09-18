import os
import xml.etree.ElementTree as ET
import numpy as np
import pyvista as pv
from spatialmath import SE3

class URDFVisualizer:
    def __init__(self, plotter, urdf_path, color_override=None, opacity=1.0):
        """
        plotter: pyvistaqt.QtInteractor instance
        urdf_path: Absolute path to the URDF file
        color_override: If provided, all links will use this color (e.g., 'blue', 'green').
        opacity: Transparency of the arm (0.0 to 1.0).
        """
        self.plotter = plotter
        self.urdf_path = urdf_path
        self.color_override = color_override
        self.opacity = opacity
        self.base_dir = os.path.dirname(urdf_path)
        self.actors = {}
        self.visual_origins = {}

        self._parse_and_load()

    def _parse_and_load(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        for link in root.findall('link'):
            name = link.get('name')
            visual = link.find('visual')
            if visual is not None:
                geom = visual.find('geometry')
                if geom is not None:
                    mesh_tag = geom.find('mesh')
                    if mesh_tag is not None:
                        filename = mesh_tag.get('filename')
                        # Resolve ROS package:// syntax
                        if filename.startswith('package://'):
                            # Assume package root is the directory containing the 'urdf' folder
                            pkg_root = os.path.dirname(self.base_dir) 
                            rel_path = filename.split('package://')[1].split('/', 1)[1]
                            mesh_file = os.path.join(pkg_root, rel_path)
                        else:
                            mesh_file = os.path.join(self.base_dir, filename)

                        mesh_file = os.path.abspath(mesh_file)

                        # Parse visual origin (offset relative to link frame)
                        origin_tag = visual.find('origin')
                        if origin_tag is not None:
                            xyz = [float(x) for x in origin_tag.get('xyz', '0 0 0').split()]
                            rpy = [float(x) for x in origin_tag.get('rpy', '0 0 0').split()]
                        else:
                            xyz = [0, 0, 0]
                            rpy = [0, 0, 0]
                        
                        visual_tf = SE3(xyz) * SE3.RPY(rpy)
                        self.visual_origins[name] = visual_tf.A # Save 4x4 matrix

                        # Parse color
                        material_tag = visual.find('material')
                        color_rgb = [0.8, 0.8, 0.8] # Default gray
                        if material_tag is not None:
                            color_tag = material_tag.find('color')
                            if color_tag is not None:
                                rgba = [float(x) for x in color_tag.get('rgba', '0.8 0.8 0.8 1').split()]
                                color_rgb = rgba[:3]

                        if os.path.exists(mesh_file):
                            try:
                                mesh = pv.read(mesh_file)
                                # Override color if requested (e.g., for ghost arms)
                                final_color = self.color_override if self.color_override else color_rgb
                                # Add to plotter
                                actor = self.plotter.add_mesh(mesh, color=final_color, opacity=self.opacity, smooth_shading=True)
                                self.actors[name] = actor
                            except Exception as e:
                                print(f"Error loading mesh {mesh_file}: {e}")

        # Add a base floor grid
        self.plotter.add_axes()
        self.plotter.view_isometric()

    def update_pose(self, robot, q):
        """
        Updates the 3D meshes given the robot model and joint angles q.
        """
        for link in robot.links:
            if link.name in self.actors:
                # Calculate precise global SE3 transform for this specific link's frame
                # This naturally includes all cumulative <joint> origin offsets and rotations
                T = robot.fkine(q, end=link.name)
                
                # Global Transform = Link Frame * Visual Offset
                visual_offset = self.visual_origins[link.name]
                global_tf = T.A @ visual_offset
                
                # Apply 4x4 transformation matrix to the PyVista actor
                self.actors[link.name].user_matrix = global_tf

        self.plotter.render()
