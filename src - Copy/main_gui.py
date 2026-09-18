import sys
import os
import math
import numpy as np

import serial.tools.list_ports

import roboticstoolbox as rtb
from spatialmath import SE3

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QComboBox,
    QTextEdit,
    QMessageBox,
    QTabWidget,
    QGroupBox,
    QFormLayout,
    QFrame
)

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPalette, QColor

from pyvistaqt import QtInteractor

from urdf_loader import load_solidworks_urdf
from fuzzy_ik_solver import FuzzyIKSolver
from serial_comm import SerialManager
from urdf_visualizer import URDFVisualizer


# ============================================================
# SETTINGS
# ============================================================

BAUDRATE = 9600

ARDUINO_JOINTS = 3

# Arduino angles are negative
SEND_NEGATIVE_ANGLES = True

# Arduino limits
J1_MIN = -120
J1_MAX = 0

J2_MIN = -180
J2_MAX = 0

J3_MIN = -120
J3_MAX = 0


# ============================================================
# MAIN APPLICATION
# ============================================================

class ArmControllerApp(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "FieldHawk Robotics Arm"
        )

        self.resize(
            1200,
            800
        )

        # ====================================================
        # LOAD ROBOT
        # WORKING SOLIDWORKS URDF LOADER
        # ====================================================

        try:

            # IMPORTANT:
            # Store path as self.urdf_path
            # because it is also needed inside setup_ui()

            self.urdf_path = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "Robotics Arm URDF",
                    "urdf",
                    "Robotics ARm.SLDASM.urdf"
                )
            )

            print("=" * 60)
            print("URDF PATH:")
            print(self.urdf_path)
            print("=" * 60)

            # IMPORTANT:
            # Use the working SolidWorks URDF loader
            self.robot = load_solidworks_urdf(
                self.urdf_path
            )

            print(
                "Robot loaded successfully."
            )

            print(
                "Number of joints:",
                self.robot.n
            )

        except Exception as e:

            QMessageBox.critical(
                self,
                "URDF Error",
                f"Failed to load URDF:\n\n{e}"
            )

            raise

        # ====================================================
        # IK SOLVER
        # ====================================================

        self.ik_solver = FuzzyIKSolver(
            self.robot
        )

        # ====================================================
        # SERIAL
        # ====================================================

        self.serial_mgr = SerialManager(
            port=None,
            baudrate=BAUDRATE
        )

        # ====================================================
        # ROBOT STATES
        # ====================================================

        self.max_reach = (
            self._calculate_max_reach()
        )

        self.q_fk = np.zeros(
            self.robot.n
        )

        self.q_ik = np.zeros(
            self.robot.n
        )

        self.q_current = np.zeros(
            self.robot.n
        )

        self.prev_fk_target = None

        self.prev_ik_target = None

        # ====================================================
        # UI
        # ====================================================

        self.setup_ui()

        # ====================================================
        # SERIAL TIMER
        # ====================================================

        self.serial_timer = QTimer()

        self.serial_timer.timeout.connect(
            self.read_serial
        )

        self.serial_timer.start(
            100
        )

        # ====================================================
        # REFRESH PORTS
        # ====================================================

        self.refresh_ports()

    # ============================================================
    # SETUP UI
    # ============================================================

    def setup_ui(self):

        central_widget = QWidget()

        self.setCentralWidget(
            central_widget
        )

        main_vbox = QVBoxLayout(
            central_widget
        )

        # ====================================================
        # SERIAL CONNECTION BAR
        # ====================================================

        serial_bar = QFrame()

        serial_bar.setStyleSheet(
            "background-color: #3b3b3b;"
            "border-radius: 5px;"
        )

        serial_layout = QHBoxLayout(
            serial_bar
        )

        serial_layout.addWidget(
            QLabel("Serial port:")
        )

        self.port_combo = QComboBox()

        self.port_combo.setMinimumWidth(
            80
        )

        serial_layout.addWidget(
            self.port_combo
        )

        update_btn = QPushButton(
            "Update"
        )

        update_btn.setStyleSheet(
            "padding: 5px 15px;"
            "font-weight: bold;"
            "background-color: #555555;"
            "color: white;"
            "border-radius: 3px;"
        )

        update_btn.clicked.connect(
            self.refresh_ports
        )

        serial_layout.addWidget(
            update_btn
        )

        serial_layout.addStretch()

        self.connect_btn = QPushButton(
            "Connect Serial"
        )

        self.connect_btn.setMinimumWidth(
            150
        )

        self.connect_btn.setStyleSheet(
            "padding: 5px 15px;"
            "font-weight: bold;"
            "background-color: #555555;"
            "color: white;"
            "border-radius: 3px;"
        )

        self.connect_btn.clicked.connect(
            self.toggle_serial
        )

        serial_layout.addWidget(
            self.connect_btn
        )

        serial_layout.addStretch()

        serial_layout.addWidget(
            QLabel("Status:")
        )

        self.status_circle = QLabel()

        self.status_circle.setFixedSize(
            16,
            16
        )

        self.status_circle.setStyleSheet(
            "background-color: red;"
            "border-radius: 8px;"
        )

        serial_layout.addWidget(
            self.status_circle
        )

        self.status_text = QLabel(
            "Disconnected"
        )

        serial_layout.addWidget(
            self.status_text
        )

        main_vbox.addWidget(
            serial_bar
        )

        # ====================================================
        # MAIN HORIZONTAL AREA
        # ====================================================

        main_layout = QHBoxLayout()

        main_vbox.addLayout(
            main_layout
        )

        # ====================================================
        # LEFT CONTROL PANEL
        # ====================================================

        control_panel = QFrame()

        control_panel.setFixedWidth(
            350
        )

        control_layout = QVBoxLayout(
            control_panel
        )

        self.tabs = QTabWidget()

        # ====================================================
        # HOME TAB
        # ====================================================

        self.home_tab = QWidget()

        home_layout = QVBoxLayout(
            self.home_tab
        )

        welcome_lbl = QLabel(
            "Welcome to the 5-DoF Robotic Arm Controller!"
        )

        welcome_lbl.setWordWrap(
            True
        )

        welcome_lbl.setStyleSheet(
            "font-size: 16px;"
            "font-weight: bold;"
            "margin-bottom: 20px;"
        )

        welcome_lbl.setAlignment(
            Qt.AlignCenter
        )

        home_layout.addWidget(
            welcome_lbl
        )

        info_lbl = QLabel(
            "The 3D viewer on the right displays TWO independent models:\n\n"
            "- The BLUE Ghost Arm represents the Forward Kinematics (FK) model.\n"
            "- The SOLID Arm represents the Inverse Kinematics (IK) model.\n\n"
            "You can use this to compare if the IK solver perfectly "
            "matches the exact joint configurations from the FK model."
        )

        info_lbl.setWordWrap(
            True
        )

        home_layout.addWidget(
            info_lbl
        )

        nav_fk_btn = QPushButton(
            "Go to Forward Kinematics"
        )

        nav_fk_btn.setStyleSheet(
            "padding: 15px;"
            "font-weight: bold;"
            "background-color: #555555;"
            "color: white;"
        )

        nav_fk_btn.clicked.connect(
            lambda:
            self.tabs.setCurrentIndex(1)
        )

        home_layout.addWidget(
            nav_fk_btn
        )

        nav_ik_btn = QPushButton(
            "Go to Inverse Kinematics"
        )

        nav_ik_btn.setStyleSheet(
            "padding: 15px;"
            "font-weight: bold;"
            "background-color: #555555;"
            "color: white;"
        )

        nav_ik_btn.clicked.connect(
            lambda:
            self.tabs.setCurrentIndex(2)
        )

        home_layout.addWidget(
            nav_ik_btn
        )

        # ====================================================
        # HOME ARM
        # ====================================================

        self.home_arm_btn = QPushButton(
            "Home Arm"
        )

        self.home_arm_btn.setStyleSheet(
            "padding: 15px;"
            "font-weight: bold;"
            "background-color: #555555;"
            "color: white;"
        )

        self.home_arm_btn.clicked.connect(
            self.home_arm
        )

        home_layout.addWidget(
            self.home_arm_btn
        )

        home_layout.addStretch()

        self.tabs.addTab(
            self.home_tab,
            "Home"
        )

        # ====================================================
        # FK TAB
        # ====================================================

        self.fk_tab = QWidget()

        fk_layout = QVBoxLayout(
            self.fk_tab
        )

        fk_input_group = QGroupBox(
            "Target Joint Angles (deg)"
        )

        fk_input_layout = QFormLayout()

        self.fk_inputs = []

        for i in range(5):

            ent = QLineEdit(
                "0.0"
            )

            fk_input_layout.addRow(
                f"J{i + 1}:",
                ent
            )

            self.fk_inputs.append(
                ent
            )

        fk_input_group.setLayout(
            fk_input_layout
        )

        fk_layout.addWidget(
            fk_input_group
        )

        self.fk_solve_btn = QPushButton(
            "Calculate FK && Simulate"
        )

        self.fk_solve_btn.setStyleSheet(
            "background-color: #555555;"
            "color: white;"
            "font-weight: bold;"
            "font-size: 16px;"
            "padding: 10px;"
        )

        self.fk_solve_btn.clicked.connect(
            self.solve_fk_and_move
        )

        fk_layout.addWidget(
            self.fk_solve_btn
        )

        fk_results_group = QGroupBox(
            "Results (End-Effector Pose)"
        )

        fk_results_layout = QFormLayout()

        self.fk_result_labels = {}

        for lbl in [
            "X (m)",
            "Y (m)",
            "Z (m)",
            "Roll (deg)",
            "Pitch (deg)",
            "Yaw (deg)"
        ]:

            val_lbl = QLabel(
                "0.00"
            )

            val_lbl.setStyleSheet(
                "font-weight: bold;"
                "color: blue;"
            )

            fk_results_layout.addRow(
                lbl + ":",
                val_lbl
            )

            self.fk_result_labels[
                lbl
            ] = val_lbl

        fk_results_group.setLayout(
            fk_results_layout
        )

        fk_layout.addWidget(
            fk_results_group
        )

        fk_layout.addStretch()

        self.tabs.addTab(
            self.fk_tab,
            "Forward Kinematics"
        )

        # ====================================================
        # IK TAB
        # ====================================================

        self.ik_tab = QWidget()

        ik_layout = QVBoxLayout(
            self.ik_tab
        )

        ik_input_group = QGroupBox(
            "Target End-Effector Pose"
        )

        ik_input_layout = QFormLayout()

        self.inputs = {}

        defaults = {
            "X (m)": "0.2",
            "Y (m)": "0.0",
            "Z (m)": "0.2",
            "Roll (deg)": "0.0",
            "Pitch (deg)": "180.0",
            "Yaw (deg)": "0.0"
        }

        for lbl, dflt in defaults.items():

            ent = QLineEdit(
                dflt
            )

            ik_input_layout.addRow(
                lbl,
                ent
            )

            self.inputs[
                lbl
            ] = ent

        ik_input_group.setLayout(
            ik_input_layout
        )

        ik_layout.addWidget(
            ik_input_group
        )

        # ====================================================
        # BUTTONS
        # ====================================================

        button_layout = QHBoxLayout()

        self.solve_btn = QPushButton(
            "Solve IK && Move"
        )

        self.solve_btn.setStyleSheet(
            "background-color: #555555;"
            "color: white;"
            "font-weight: bold;"
            "font-size: 16px;"
            "padding: 10px;"
        )

        self.solve_btn.clicked.connect(
            self.solve_and_move
        )

        button_layout.addWidget(
            self.solve_btn
        )

        calculate_send_btn = QPushButton(
            "IK + SEND"
        )

        calculate_send_btn.setStyleSheet(
            "background-color: #555555;"
            "color: white;"
            "font-weight: bold;"
            "padding: 10px;"
        )

        calculate_send_btn.clicked.connect(
            self.calculate_ik_and_send
        )

        button_layout.addWidget(
            calculate_send_btn
        )

        ik_layout.addLayout(
            button_layout
        )

        # ====================================================
        # IK RESULTS
        # ====================================================

        ik_results_group = QGroupBox(
            "Results (Joint Angles)"
        )

        ik_results_layout = QFormLayout()

        self.joint_labels = []

        for i in range(5):

            lbl = QLabel(
                "0.00°"
            )

            lbl.setStyleSheet(
                "font-weight: bold;"
                "color: blue;"
            )

            ik_results_layout.addRow(
                f"J{i + 1}:",
                lbl
            )

            self.joint_labels.append(
                lbl
            )

        ik_results_group.setLayout(
            ik_results_layout
        )

        ik_layout.addWidget(
            ik_results_group
        )

        ik_layout.addStretch()

        self.tabs.addTab(
            self.ik_tab,
            "Inverse Kinematics"
        )

        # ====================================================
        # ARDUINO TEST TAB
        # ====================================================

        self.create_serial_test_tab()

        # ====================================================
        # ADD TABS
        # ====================================================

        control_layout.addWidget(
            self.tabs
        )

        # ====================================================
        # STATUS
        # ====================================================

        self.status_lbl = QLabel(
            "Ready"
        )

        self.status_lbl.setWordWrap(
            True
        )

        control_layout.addWidget(
            self.status_lbl
        )

        control_layout.addStretch()

        main_layout.addWidget(
            control_panel
        )

        # ====================================================
        # RIGHT 3D VISUALIZER
        # ====================================================

        self.plotter = QtInteractor(
            central_widget
        )

        self.plotter.set_background(
            "#2b2b2b"
        )

        main_layout.addWidget(
            self.plotter.interactor
        )

        # ====================================================
        # BLUE GHOST FK ARM
        # ====================================================

        # FIX:
        # Use self.urdf_path instead of urdf_path

        self.visualizer_fk = URDFVisualizer(
            self.plotter,
            self.urdf_path,
            color_override=[
                0.0,
                0.5,
                1.0
            ],
            opacity=0.4
        )

        # ====================================================
        # SOLID IK ARM
        # ====================================================

        self.visualizer_ik = URDFVisualizer(
            self.plotter,
            self.urdf_path,
            opacity=1.0
        )

        # ====================================================
        # INITIAL POSE
        # ====================================================

        self.visualizer_fk.update_pose(
            self.robot,
            self.q_fk
        )

        self.visualizer_ik.update_pose(
            self.robot,
            self.q_ik
        )

        # ====================================================
        # CAMERA
        # ====================================================

        self.plotter.camera_position = [
            (
                1.5,
                -2.5,
                1.0
            ),
            (
                0.0,
                0.0,
                0.2
            ),
            (
                0.0,
                0.0,
                -1.0
            )
        ]

        # ====================================================
        # SYSTEM LOG
        # ====================================================

        log_group = QGroupBox(
            "System Log"
        )

        log_layout = QVBoxLayout()

        self.log_output = QTextEdit()

        self.log_output.setReadOnly(
            True
        )

        self.log_output.setMaximumHeight(
            120
        )

        log_layout.addWidget(
            self.log_output
        )

        log_group.setLayout(
            log_layout
        )

        main_vbox.addWidget(
            log_group
        )

    # ============================================================
    # ARDUINO TEST TAB
    # ============================================================

    def create_serial_test_tab(self):

        tab = QWidget()

        layout = QVBoxLayout()

        group = QGroupBox(
            "Direct Arduino Joint Test"
        )

        grid = QGridLayout()

        self.test_inputs = []

        defaults = [
            "-10",
            "-10",
            "-10"
        ]

        for i in range(3):

            grid.addWidget(
                QLabel(
                    f"J{i + 1} (deg):"
                ),
                i,
                0
            )

            edit = QLineEdit(
                defaults[i]
            )

            self.test_inputs.append(
                edit
            )

            grid.addWidget(
                edit,
                i,
                1
            )

        send_button = QPushButton(
            "SEND ANGLES TO ARDUINO"
        )

        send_button.clicked.connect(
            self.send_test_angles
        )

        grid.addWidget(
            send_button,
            3,
            0,
            1,
            2
        )

        home_button = QPushButton(
            "SEND HOME"
        )

        home_button.clicked.connect(
            self.home_arm
        )

        grid.addWidget(
            home_button,
            4,
            0,
            1,
            2
        )

        group.setLayout(
            grid
        )

        layout.addWidget(
            group
        )

        info = QLabel(
            "Example:\n"
            "J1 = -10°\n"
            "J2 = -10°\n"
            "J3 = -10°\n\n"
            "Command:\n"
            "A,-10.00,-10.00,-10.00"
        )

        info.setWordWrap(
            True
        )

        layout.addWidget(
            info
        )

        layout.addStretch()

        tab.setLayout(
            layout
        )

        self.tabs.addTab(
            tab,
            "Arduino Test"
        )

    # ============================================================
    # MAX REACH
    # ============================================================

    def _calculate_max_reach(self):

        max_reach = 0.0

        for link in self.robot.links:

            if link.parent is not None:

                max_reach += np.linalg.norm(
                    link.A(0).t
                )

        return max_reach

    # ============================================================
    # REFRESH PORTS
    # ============================================================

    def refresh_ports(self):

        current_port = (
            self.port_combo.currentText()
        )

        self.port_combo.clear()

        ports = (
            serial.tools.list_ports
            .comports()
        )

        for port in ports:

            self.port_combo.addItem(
                port.device
            )

        if self.port_combo.count() == 0:

            self.port_combo.addItem(
                "COM?"
            )

        if current_port:

            index = (
                self.port_combo
                .findText(
                    current_port
                )
            )

            if index >= 0:

                self.port_combo.setCurrentIndex(
                    index
                )

        self.log(
            "Available COM ports refreshed."
        )

    # ============================================================
    # TOGGLE SERIAL
    # ============================================================

    def toggle_serial(self):

        # --------------------------------------------------------
        # DISCONNECT
        # --------------------------------------------------------

        if (
            self.serial_mgr.serial_conn
            and
            self.serial_mgr.serial_conn.is_open
        ):

            self.serial_mgr.disconnect()

            self.connect_btn.setText(
                "Connect Serial"
            )

            self.connect_btn.setStyleSheet(
                "padding: 5px 15px;"
                "font-weight: bold;"
                "background-color: #555555;"
                "color: white;"
                "border-radius: 3px;"
            )

            self.status_circle.setStyleSheet(
                "background-color: red;"
                "border-radius: 8px;"
            )

            self.status_text.setText(
                "Disconnected"
            )

            self.status_lbl.setText(
                "Serial Disconnected."
            )

            self.log(
                "Arduino disconnected."
            )

            return

        # --------------------------------------------------------
        # CONNECT
        # --------------------------------------------------------

        port = (
            self.port_combo
            .currentText()
        )

        if (
            not port
            or
            port == "COM?"
        ):

            QMessageBox.warning(
                self,
                "Port Error",
                "No valid COM port selected."
            )

            return

        self.serial_mgr.port = port

        self.status_lbl.setText(
            f"Connecting to {port}..."
        )

        self.log(
            f"Connecting to {port}..."
        )

        QApplication.processEvents()

        success = (
            self.serial_mgr
            .connect()
        )

        if success:

            self.connect_btn.setText(
                "Disconnect Serial"
            )

            self.connect_btn.setStyleSheet(
                "padding: 5px 15px;"
                "font-weight: bold;"
                "background-color: #e74c3c;"
                "color: white;"
                "border-radius: 3px;"
            )

            self.status_circle.setStyleSheet(
                "background-color: #2ecc71;"
                "border-radius: 8px;"
            )

            self.status_text.setText(
                "Arduino READY"
            )

            self.status_lbl.setText(
                f"Connected to {port}. "
                f"Arduino is READY."
            )

            self.log(
                "Arduino connected."
            )

        else:

            self.status_circle.setStyleSheet(
                "background-color: red;"
                "border-radius: 8px;"
            )

            self.status_text.setText(
                "Not Ready"
            )

            self.status_lbl.setText(
                "Arduino is not READY."
            )

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino connected, but did not "
                "reach READY."
            )

    # ============================================================
    # HOME ARM
    # ============================================================

    def home_arm(self):

        # --------------------------------------------------------
        # Reset simulation
        # --------------------------------------------------------

        self.q_fk = np.zeros(
            self.robot.n
        )

        self.q_ik = np.zeros(
            self.robot.n
        )

        self.q_current = np.zeros(
            self.robot.n
        )

        self.prev_fk_target = None

        self.prev_ik_target = None

        self.visualizer_fk.update_pose(
            self.robot,
            self.q_fk
        )

        self.visualizer_ik.update_pose(
            self.robot,
            self.q_ik
        )

        for label in self.joint_labels:

            label.setText(
                "0.00°"
            )

        # --------------------------------------------------------
        # Send HOME to Arduino
        # --------------------------------------------------------

        if not self.serial_mgr.is_connected():

            self.status_lbl.setText(
                "Simulation reset to HOME. "
                "(Arduino not connected)"
            )

            self.log(
                "Visual arm reset to HOME."
            )

            return

        self.log(
            "Sending HOME command to Arduino..."
        )

        success = (
            self.serial_mgr
            .send_home()
        )

        if success:

            self.status_circle.setStyleSheet(
                "background-color: #f1c40f;"
                "border-radius: 8px;"
            )

            self.status_text.setText(
                "Homing..."
            )

            self.status_lbl.setText(
                "HOME command sent. "
                "Arduino is homing..."
            )

            self.log(
                "HOME command sent successfully."
            )

        else:

            self.status_lbl.setText(
                "Failed to send HOME command."
            )

    # ============================================================
    # FK
    # ============================================================

    def solve_fk_and_move(self):

        try:

            q_target = []

            for i in range(5):

                q_target.append(
                    math.radians(
                        float(
                            self.fk_inputs[
                                i
                            ].text()
                        )
                    )
                )

            q_target = np.array(
                q_target
            )

        except ValueError:

            QMessageBox.critical(
                self,
                "Input Error",
                "Please enter valid numerical "
                "values for joints."
            )

            return

        # --------------------------------------------------------
        # Optimization
        # --------------------------------------------------------

        if (
            self.prev_fk_target is not None
            and
            np.allclose(
                self.prev_fk_target,
                q_target
            )
        ):

            self.status_lbl.setText(
                "Joint angles unchanged. "
                "Skipping calculation."
            )

            return

        self.prev_fk_target = (
            q_target.copy()
        )

        self.q_fk = (
            q_target.copy()
        )

        self.q_current = (
            q_target.copy()
        )

        # --------------------------------------------------------
        # FK
        # --------------------------------------------------------

        T = self.robot.fkine(
            self.q_fk
        )

        rpy = T.rpy()

        # --------------------------------------------------------
        # Results
        # --------------------------------------------------------

        self.fk_result_labels[
            "X (m)"
        ].setText(
            f"{T.t[0]:.3f}"
        )

        self.fk_result_labels[
            "Y (m)"
        ].setText(
            f"{T.t[1]:.3f}"
        )

        self.fk_result_labels[
            "Z (m)"
        ].setText(
            f"{T.t[2]:.3f}"
        )

        self.fk_result_labels[
            "Roll (deg)"
        ].setText(
            f"{math.degrees(rpy[0]):.2f}"
        )

        self.fk_result_labels[
            "Pitch (deg)"
        ].setText(
            f"{math.degrees(rpy[1]):.2f}"
        )

        self.fk_result_labels[
            "Yaw (deg)"
        ].setText(
            f"{math.degrees(rpy[2]):.2f}"
        )

        # --------------------------------------------------------
        # Move BLUE Ghost Arm
        # --------------------------------------------------------

        self.visualizer_fk.update_pose(
            self.robot,
            self.q_fk
        )

        # --------------------------------------------------------
        # FK is visual only
        # --------------------------------------------------------

        self.status_lbl.setText(
            "FK Calculated and Ghost Arm "
            "moved visually. "
            "(No data sent to Arduino)"
        )

        self.log(
            "FK calculated successfully."
        )

    # ============================================================
    # GET TARGET POSE
    # ============================================================

    def get_target_pose(self):

        x = float(
            self.inputs[
                "X (m)"
            ].text()
        )

        y = float(
            self.inputs[
                "Y (m)"
            ].text()
        )

        z = float(
            self.inputs[
                "Z (m)"
            ].text()
        )

        roll = float(
            self.inputs[
                "Roll (deg)"
            ].text()
        )

        pitch = float(
            self.inputs[
                "Pitch (deg)"
            ].text()
        )

        yaw = float(
            self.inputs[
                "Yaw (deg)"
            ].text()
        )

        roll = np.radians(
            roll
        )

        pitch = np.radians(
            pitch
        )

        yaw = np.radians(
            yaw
        )

        T = (
            SE3(x, y, z)
            *
            SE3.RPY(
                [
                    roll,
                    pitch,
                    yaw
                ],
                order="xyz"
            )
        )

        return T

    # ============================================================
    # CALCULATE IK
    # ============================================================

    def calculate_ik(self):

        try:

            target_pose = (
                self.get_target_pose()
            )

            # Start from current IK position
            q0 = (
                self.q_ik.copy()
            )

            self.log(
                "Calculating IK..."
            )

            q_solution, success = (
                self.ik_solver.solve_ik(
                    target_pose,
                    q0
                )
            )

            if not success:

                self.log(
                    "WARNING: IK did not fully converge."
                )

            self.q_ik = (
                q_solution.copy()
            )

            self.q_current = (
                q_solution.copy()
            )

            degrees = np.degrees(
                q_solution
            )

            # ----------------------------------------------------
            # DISPLAY J1-J5
            # ----------------------------------------------------

            for i in range(
                min(
                    5,
                    len(degrees)
                )
            ):

                self.joint_labels[
                    i
                ].setText(
                    f"{degrees[i]:.2f}°"
                )

            # ----------------------------------------------------
            # MOVE SOLID IK ARM
            # ----------------------------------------------------

            self.visualizer_ik.update_pose(
                self.robot,
                self.q_ik
            )

            self.log(
                "IK result:\n"
                +
                ", ".join(
                    [
                        f"J{i + 1}="
                        f"{degrees[i]:.2f}°"
                        for i in range(
                            len(degrees)
                        )
                    ]
                )
            )

            return degrees, success

        except Exception as e:

            QMessageBox.critical(
                self,
                "IK Error",
                str(e)
            )

            self.log(
                f"IK Error: {e}"
            )

            return None, False

    # ============================================================
    # SOLVE IK + MOVE + SEND
    # ============================================================

    def solve_and_move(self):

        try:

            x = float(
                self.inputs[
                    "X (m)"
                ].text()
            )

            y = float(
                self.inputs[
                    "Y (m)"
                ].text()
            )

            z = float(
                self.inputs[
                    "Z (m)"
                ].text()
            )

            r = math.radians(
                float(
                    self.inputs[
                        "Roll (deg)"
                    ].text()
                )
            )

            p = math.radians(
                float(
                    self.inputs[
                        "Pitch (deg)"
                    ].text()
                )
            )

            yaw = math.radians(
                float(
                    self.inputs[
                        "Yaw (deg)"
                    ].text()
                )
            )

        except ValueError:

            QMessageBox.critical(
                self,
                "Input Error",
                "Please enter valid numerical values."
            )

            return

        target_tuple = (
            x,
            y,
            z,
            r,
            p,
            yaw
        )

        if (
            self.prev_ik_target is not None
            and
            self.prev_ik_target
            ==
            target_tuple
        ):

            self.status_lbl.setText(
                "Target pose unchanged. "
                "Skipping IK calculation."
            )

            return

        self.prev_ik_target = (
            target_tuple
        )

        # --------------------------------------------------------
        # Reachability
        # --------------------------------------------------------

        target_dist = math.sqrt(
            x**2 +
            y**2 +
            z**2
        )

        if target_dist > self.max_reach:

            QMessageBox.critical(
                self,
                "Reachability Error",
                f"Target is unreachable!\n"
                f"Target distance "
                f"({target_dist:.2f}m) exceeds "
                f"max arm reach "
                f"(~{self.max_reach:.2f}m)."
            )

            return

        self.status_lbl.setText(
            "Solving IK using Fuzzy Logic..."
        )

        QApplication.processEvents()

        # --------------------------------------------------------
        # Target pose
        # --------------------------------------------------------

        target_pose = (
            SE3(x, y, z)
            *
            SE3.RPY(
                [
                    r,
                    p,
                    yaw
                ],
                order="xyz"
            )
        )

        # --------------------------------------------------------
        # Solve IK
        # --------------------------------------------------------

        q_sol, converged = (
            self.ik_solver.solve_ik(
                target_pose,
                self.q_ik
            )
        )

        # --------------------------------------------------------
        # Actual position
        # --------------------------------------------------------

        T_actual = (
            self.robot.fkine(
                q_sol
            )
        )

        pos_err = np.linalg.norm(
            target_pose.t -
            T_actual.t
        )

        if pos_err > 0.01:

            QMessageBox.warning(
                self,
                "Reachability Warning",
                f"Could not perfectly reach target.\n"
                f"The arm stopped "
                f"{pos_err * 100:.1f} cm "
                f"away from target because "
                f"it is mechanically impossible."
            )

            self.status_lbl.setText(
                f"Stopped "
                f"{pos_err * 100:.1f} cm from target."
            )

        elif not converged:

            self.status_lbl.setText(
                "IK Solved. "
                "XYZ reached; Roll/Pitch "
                "slightly approximated."
            )

        else:

            self.status_lbl.setText(
                "IK Solved and Perfectly Converged."
            )

        # --------------------------------------------------------
        # Update state
        # --------------------------------------------------------

        self.q_ik = (
            q_sol.copy()
        )

        self.q_current = (
            q_sol.copy()
        )

        # --------------------------------------------------------
        # Update SOLID IK ARM
        # --------------------------------------------------------

        self.visualizer_ik.update_pose(
            self.robot,
            self.q_ik
        )

        # --------------------------------------------------------
        # Display J1-J5
        # --------------------------------------------------------

        j_degs = [
            math.degrees(
                theta
            )
            for theta in self.q_ik[:5]
        ]

        for i in range(5):

            self.joint_labels[
                i
            ].setText(
                f"{j_degs[i]:.2f}°"
            )

        # --------------------------------------------------------
        # Send first 3 joints to Arduino
        # --------------------------------------------------------

        self.send_ik_to_arduino(
            j_degs
        )

    # ============================================================
    # CALCULATE IK + SEND
    # ============================================================

    def calculate_ik_and_send(self):

        result = (
            self.calculate_ik()
        )

        if result[0] is None:

            return

        degrees, success = result

        self.send_ik_to_arduino(
            degrees
        )

    # ============================================================
    # SEND IK TO ARDUINO
    # ============================================================

    def send_ik_to_arduino(
        self,
        degrees
    ):

        # --------------------------------------------------------
        # Check connection
        # --------------------------------------------------------

        if not self.serial_mgr.is_connected():

            self.status_lbl.setText(
                "IK calculated and "
                "simulation moved. "
                "(Arduino not connected)"
            )

            self.log(
                "IK calculated but Arduino "
                "is not connected."
            )

            return

        # --------------------------------------------------------
        # Check READY
        # --------------------------------------------------------

        if hasattr(
            self.serial_mgr,
            "is_ready"
        ):

            if not self.serial_mgr.is_ready():

                QMessageBox.warning(
                    self,
                    "Arduino",
                    "Arduino is not READY.\n\n"
                    "Wait until HOMING is completed."
                )

                self.log(
                    "Arduino is not READY."
                )

                return

        # --------------------------------------------------------
        # Make sure we have 3 joints
        # --------------------------------------------------------

        if len(degrees) < ARDUINO_JOINTS:

            QMessageBox.warning(
                self,
                "Arduino",
                "IK solution does not contain "
                "enough joints."
            )

            return

        # --------------------------------------------------------
        # First 3 joints
        # --------------------------------------------------------

        j1 = float(
            degrees[0]
        )

        j2 = float(
            degrees[1]
        )

        j3 = float(
            degrees[2]
        )

        # --------------------------------------------------------
        # Convert to negative convention
        # --------------------------------------------------------

        if SEND_NEGATIVE_ANGLES:

            j1 = -abs(j1)

            j2 = -abs(j2)

            j3 = -abs(j3)

        # --------------------------------------------------------
        # Limits
        # --------------------------------------------------------

        if not (
            J1_MIN <= j1 <= J1_MAX
        ):

            QMessageBox.warning(
                self,
                "J1 Limit",
                f"J1 = {j1:.2f}°\n\n"
                "Allowed range:\n"
                "-120° to 0°"
            )

            return

        if not (
            J2_MIN <= j2 <= J2_MAX
        ):

            QMessageBox.warning(
                self,
                "J2 Limit",
                f"J2 = {j2:.2f}°\n\n"
                "Allowed range:\n"
                "-180° to 0°"
            )

            return

        if not (
            J3_MIN <= j3 <= J3_MAX
        ):

            QMessageBox.warning(
                self,
                "J3 Limit",
                f"J3 = {j3:.2f}°\n\n"
                "Allowed range:\n"
                "-120° to 0°"
            )

            return

        # --------------------------------------------------------
        # Send
        # --------------------------------------------------------

        command = (
            f"A,{j1:.2f},"
            f"{j2:.2f},"
            f"{j3:.2f}"
        )

        self.log(
            "----------------------------------------"
        )

        self.log(
            "IK -> Arduino:"
        )

        self.log(
            f"J1 = {j1:.2f}°"
        )

        self.log(
            f"J2 = {j2:.2f}°"
        )

        self.log(
            f"J3 = {j3:.2f}°"
        )

        self.log(
            f"Command: {command}"
        )

        success = (
            self.serial_mgr
            .send_angles(
                j1,
                j2,
                j3
            )
        )

        if success:

            self.status_lbl.setText(
                "IK calculated, "
                "3D arm moved, "
                "and J1-J3 sent to Arduino."
            )

            self.log(
                "Command sent successfully."
            )

        else:

            self.status_lbl.setText(
                "IK calculated and "
                "3D arm moved, "
                "but serial transmission failed."
            )

            self.log(
                "ERROR: Serial transmission failed."
            )

    # ============================================================
    # DIRECT ARDUINO TEST
    # ============================================================

    def send_test_angles(self):

        try:

            j1 = float(
                self.test_inputs[
                    0
                ].text()
            )

            j2 = float(
                self.test_inputs[
                    1
                ].text()
            )

            j3 = float(
                self.test_inputs[
                    2
                ].text()
            )

        except ValueError:

            QMessageBox.warning(
                self,
                "Input Error",
                "Please enter valid numbers."
            )

            return

        # --------------------------------------------------------
        # Limits
        # --------------------------------------------------------

        if not (
            J1_MIN <= j1 <= J1_MAX
        ):

            QMessageBox.warning(
                self,
                "J1 Limit",
                "J1 must be between "
                "-120° and 0°."
            )

            return

        if not (
            J2_MIN <= j2 <= J2_MAX
        ):

            QMessageBox.warning(
                self,
                "J2 Limit",
                "J2 must be between "
                "-180° and 0°."
            )

            return

        if not (
            J3_MIN <= j3 <= J3_MAX
        ):

            QMessageBox.warning(
                self,
                "J3 Limit",
                "J3 must be between "
                "-120° and 0°."
            )

            return

        # --------------------------------------------------------
        # READY
        # --------------------------------------------------------

        if not self.serial_mgr.is_ready():

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino is not READY yet.\n\n"
                "Connect and wait for HOMING "
                "to finish."
            )

            return

        # --------------------------------------------------------
        # Send
        # --------------------------------------------------------

        success = (
            self.serial_mgr
            .send_angles(
                j1,
                j2,
                j3
            )
        )

        if success:

            command = (
                f"A,{j1:.2f},"
                f"{j2:.2f},"
                f"{j3:.2f}"
            )

            self.log(
                "----------------------------------------"
            )

            self.log(
                f"TEST -> {command}"
            )

            self.status_lbl.setText(
                "Test angles sent to Arduino."
            )

        else:

            self.log(
                "Failed to send test angles."
            )

            self.status_lbl.setText(
                "Failed to send test angles."
            )

    # ============================================================
    # READ SERIAL
    # ============================================================

    def read_serial(self):

        if not self.serial_mgr.is_connected():

            return

        try:

            messages = (
                self.serial_mgr
                .read_available()
            )

            for message in messages:

                self.log(
                    f"Arduino: {message}"
                )

                # ----------------------------------------------
                # HOMING DONE
                # ----------------------------------------------

                if message == "HOMING_DONE":

                    self.status_circle.setStyleSheet(
                        "background-color: #2ecc71;"
                        "border-radius: 8px;"
                    )

                    self.status_text.setText(
                        "Homing Done"
                    )

                    self.status_lbl.setText(
                        "Arduino Homing completed."
                    )

                # ----------------------------------------------
                # READY
                # ----------------------------------------------

                elif message == "READY":

                    self.status_circle.setStyleSheet(
                        "background-color: #2ecc71;"
                        "border-radius: 8px;"
                    )

                    self.status_text.setText(
                        "Arduino READY"
                    )

                    self.status_lbl.setText(
                        "Arduino is READY."
                    )

                # ----------------------------------------------
                # DONE
                # ----------------------------------------------

                elif message == "DONE":

                    self.status_lbl.setText(
                        "Arduino movement completed."
                    )

        except Exception as e:

            self.log(
                f"Serial read error: {e}"
            )

    # ============================================================
    # LOG
    # ============================================================

    def log(
        self,
        message
    ):

        if hasattr(
            self,
            "log_output"
        ):

            self.log_output.append(
                str(message)
            )

    # ============================================================
    # CLOSE
    # ============================================================

    def closeEvent(
        self,
        event
    ):

        try:

            if hasattr(
                self,
                "serial_timer"
            ):

                self.serial_timer.stop()

        except Exception:

            pass

        try:

            self.serial_mgr.disconnect()

        except Exception:

            pass

        try:

            self.plotter.close()

        except Exception:

            pass

        event.accept()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    app = QApplication(
        sys.argv
    )

    # ========================================================
    # DARK THEME FROM FIRST CODE
    # ========================================================

    app.setStyle(
        "Fusion"
    )

    dark_palette = QPalette()

    dark_palette.setColor(
        QPalette.Window,
        QColor(53, 53, 53)
    )

    dark_palette.setColor(
        QPalette.WindowText,
        Qt.white
    )

    dark_palette.setColor(
        QPalette.Base,
        QColor(25, 25, 25)
    )

    dark_palette.setColor(
        QPalette.AlternateBase,
        QColor(53, 53, 53)
    )

    dark_palette.setColor(
        QPalette.ToolTipBase,
        Qt.white
    )

    dark_palette.setColor(
        QPalette.ToolTipText,
        Qt.white
    )

    dark_palette.setColor(
        QPalette.Text,
        Qt.white
    )

    dark_palette.setColor(
        QPalette.Button,
        QColor(53, 53, 53)
    )

    dark_palette.setColor(
        QPalette.ButtonText,
        Qt.white
    )

    dark_palette.setColor(
        QPalette.BrightText,
        Qt.red
    )

    dark_palette.setColor(
        QPalette.Link,
        QColor(42, 130, 218)
    )

    dark_palette.setColor(
        QPalette.Highlight,
        QColor(42, 130, 218)
    )

    dark_palette.setColor(
        QPalette.HighlightedText,
        Qt.black
    )

    app.setPalette(
        dark_palette
    )

    # ========================================================
    # START
    # ========================================================

    window = ArmControllerApp()

    window.show()

    sys.exit(
        app.exec_()
    )