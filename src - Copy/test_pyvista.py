import pyvista as pv
import xml.etree.ElementTree as ET
import os
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton
from pyvistaqt import QtInteractor
import numpy as np

class TestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyVista + PyQt5 Test")
        
        self.frame = QWidget()
        self.layout = QVBoxLayout()
        self.frame.setLayout(self.layout)
        self.setCentralWidget(self.frame)
        
        # PyVista widget
        self.plotter = QtInteractor(self.frame)
        self.layout.addWidget(self.plotter.interactor)
        
        self.btn = QPushButton("Test")
        self.layout.addWidget(self.btn)
        
        # Load mesh
        mesh_file = os.path.abspath('../Robotics _Arm_URDF/meshes/base_link.STL')
        mesh = pv.read(mesh_file)
        self.actor = self.plotter.add_mesh(mesh, color='white')
        self.plotter.reset_camera()
        
app = QApplication(sys.argv)
window = TestWindow()
window.show()
sys.exit(app.exec_())
