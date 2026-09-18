import sys
import os
import numpy as np
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QComboBox,
    QTextEdit,
    QMessageBox,
    QTabWidget,
    QGroupBox
)

from PyQt5.QtCore import QTimer

import roboticstoolbox as rtb
from spatialmath import SE3

from urdf_loader import load_solidworks_urdf
from fuzzy_ik_solver import FuzzyIKSolver
from serial_comm import SerialManager

from urdf_visualizer import URDFVisualizer


# ============================================================
# SETTINGS
# ============================================================

BAUDRATE = 9600

ARDUINO_JOINTS = 3

# Arduino angles are negative.
# URDF may return positive angles, so we convert them.
SEND_NEGATIVE_ANGLES = True


# ============================================================
# MAIN WINDOW
# ============================================================

class MainWindow(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "5-DOF Robotic Arm - IK + Arduino"
        )

        self.resize(
            1300,
            850
        )

        # ======================================================
        # LOAD URDF
        # ======================================================

        try:

            urdf_path = os.path.abspath(
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
            print(urdf_path)
            print("=" * 60)

            self.robot = load_solidworks_urdf(
                urdf_path
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

        # ======================================================
        # IK SOLVER
        # ======================================================

        self.ik_solver = FuzzyIKSolver(
            self.robot
        )

        # Current Python joint configuration
        self.q_current = np.zeros(
            self.robot.n
        )

        # ======================================================
        # SERIAL
        # ======================================================

        self.serial_manager = SerialManager(
            port=None,
            baudrate=BAUDRATE
        )

        # ======================================================
        # UI
        # ======================================================

        self.init_ui()

        # ======================================================
        # SERIAL TIMER
        # ======================================================

        self.serial_timer = QTimer()

        self.serial_timer.timeout.connect(
            self.read_serial
        )

        self.serial_timer.start(
            100
        )

        # ======================================================
        # REFRESH PORTS
        # ======================================================

        self.refresh_ports()

    # ============================================================
    # UI
    # ============================================================

    def init_ui(self):

        central = QWidget()

        self.setCentralWidget(
            central
        )

        main_layout = QVBoxLayout()

        central.setLayout(
            main_layout
        )

        # ======================================================
        # SERIAL CONNECTION
        # ======================================================

        serial_group = QGroupBox(
            "Arduino Serial Connection"
        )

        serial_layout = QHBoxLayout()

        self.port_combo = QComboBox()

        self.refresh_button = QPushButton(
            "Refresh Ports"
        )

        self.refresh_button.clicked.connect(
            self.refresh_ports
        )

        self.connect_button = QPushButton(
            "Connect"
        )

        self.connect_button.clicked.connect(
            self.connect_serial
        )

        self.disconnect_button = QPushButton(
            "Disconnect"
        )

        self.disconnect_button.clicked.connect(
            self.disconnect_serial
        )

        self.home_button = QPushButton(
            "HOME"
        )

        self.home_button.clicked.connect(
            self.home_arm
        )

        self.status_label = QLabel(
            "Disconnected"
        )

        serial_layout.addWidget(
            QLabel("COM Port:")
        )

        serial_layout.addWidget(
            self.port_combo
        )

        serial_layout.addWidget(
            self.refresh_button
        )

        serial_layout.addWidget(
            self.connect_button
        )

        serial_layout.addWidget(
            self.disconnect_button
        )

        serial_layout.addWidget(
            self.home_button
        )

        serial_layout.addWidget(
            self.status_label
        )

        serial_group.setLayout(
            serial_layout
        )

        main_layout.addWidget(
            serial_group
        )

        # ======================================================
        # TABS
        # ======================================================

        self.tabs = QTabWidget()

        self.create_fk_tab()

        self.create_ik_tab()

        self.create_serial_test_tab()

        main_layout.addWidget(
            self.tabs
        )

        # ======================================================
        # LOG
        # ======================================================

        log_group = QGroupBox(
            "System Log"
        )

        log_layout = QVBoxLayout()

        self.log_output = QTextEdit()

        self.log_output.setReadOnly(
            True
        )

        log_layout.addWidget(
            self.log_output
        )

        log_group.setLayout(
            log_layout
        )

        main_layout.addWidget(
            log_group
        )

    # ============================================================
    # FK TAB
    # ============================================================

    def create_fk_tab(self):

        tab = QWidget()

        layout = QVBoxLayout()

        group = QGroupBox(
            "Forward Kinematics - Joint Angles"
        )

        grid = QGridLayout()

        self.fk_inputs = []

        for i in range(5):

            label = QLabel(
                f"J{i + 1} (deg):"
            )

            edit = QLineEdit(
                "0"
            )

            self.fk_inputs.append(
                edit
            )

            grid.addWidget(
                label,
                i,
                0
            )

            grid.addWidget(
                edit,
                i,
                1
            )

        fk_button = QPushButton(
            "Apply FK"
        )

        fk_button.clicked.connect(
            self.apply_fk
        )

        grid.addWidget(
            fk_button,
            5,
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

        tab.setLayout(
            layout
        )

        self.tabs.addTab(
            tab,
            "FK"
        )

    # ============================================================
    # IK TAB
    # ============================================================

    def create_ik_tab(self):

        tab = QWidget()

        layout = QVBoxLayout()

        # ======================================================
        # TARGET POSE
        # ======================================================

        target_group = QGroupBox(
            "Target Pose"
        )

        grid = QGridLayout()

        self.ik_inputs = {}

        fields = [
            ("X (m)", "x", "0.30"),
            ("Y (m)", "y", "0.00"),
            ("Z (m)", "z", "0.20"),
            ("Roll (deg)", "roll", "0"),
            ("Pitch (deg)", "pitch", "0"),
            ("Yaw (deg)", "yaw", "0"),
        ]

        for row, (
            label_text,
            key,
            default
        ) in enumerate(fields):

            label = QLabel(
                label_text
            )

            edit = QLineEdit(
                default
            )

            self.ik_inputs[key] = edit

            grid.addWidget(
                label,
                row,
                0
            )

            grid.addWidget(
                edit,
                row,
                1
            )

        target_group.setLayout(
            grid
        )

        layout.addWidget(
            target_group
        )

        # ======================================================
        # BUTTONS
        # ======================================================

        button_layout = QHBoxLayout()

        solve_button = QPushButton(
            "Calculate IK"
        )

        solve_button.clicked.connect(
            self.calculate_ik
        )

        calculate_send_button = QPushButton(
            "Calculate IK + SEND TO ARDUINO"
        )

        calculate_send_button.clicked.connect(
            self.calculate_ik_and_send
        )

        home_button = QPushButton(
            "HOME"
        )

        home_button.clicked.connect(
            self.home_arm
        )

        button_layout.addWidget(
            solve_button
        )

        button_layout.addWidget(
            calculate_send_button
        )

        button_layout.addWidget(
            home_button
        )

        layout.addLayout(
            button_layout
        )

        # ======================================================
        # IK RESULT
        # ======================================================

        result_group = QGroupBox(
            "IK Result"
        )

        result_layout = QGridLayout()

        self.joint_result_labels = []

        for i in range(5):

            result_layout.addWidget(
                QLabel(
                    f"J{i + 1}:"
                ),
                i,
                0
            )

            label = QLabel(
                "0.00°"
            )

            self.joint_result_labels.append(
                label
            )

            result_layout.addWidget(
                label,
                i,
                1
            )

        result_group.setLayout(
            result_layout
        )

        layout.addWidget(
            result_group
        )

        tab.setLayout(
            layout
        )

        self.tabs.addTab(
            tab,
            "IK"
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

        home_button = QPushButton(
            "SEND HOME"
        )

        home_button.clicked.connect(
            self.home_arm
        )

        grid.addWidget(
            send_button,
            3,
            0,
            1,
            2
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
            "J1 = -10\n"
            "J2 = -10\n"
            "J3 = -10\n\n"
            "Command sent to Arduino:\n"
            "A,-10.00,-10.00,-10.00"
        )

        layout.addWidget(
            info
        )

        tab.setLayout(
            layout
        )

        self.tabs.addTab(
            tab,
            "Arduino Test"
        )

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

        if current_port:

            index = (
                self.port_combo
                .findText(current_port)
            )

            if index >= 0:

                self.port_combo.setCurrentIndex(
                    index
                )

        self.log(
            "Available COM ports refreshed."
        )

    # ============================================================
    # CONNECT SERIAL
    # ============================================================

    def connect_serial(self):

        port = (
            self.port_combo
            .currentText()
        )

        if not port:

            QMessageBox.warning(
                self,
                "Serial",
                "Please select a COM port."
            )

            return

        self.serial_manager.port = port

        self.log(
            f"Connecting to {port}..."
        )

        success = (
            self.serial_manager
            .connect()
        )

        if success:

            self.status_label.setText(
                f"Arduino READY: {port}"
            )

            self.log(
                "Arduino connected and "
                "HOMING completed."
            )

        else:

            self.status_label.setText(
                "Arduino not ready"
            )

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino connected, but it "
                "did not reach READY."
            )

    # ============================================================
    # DISCONNECT
    # ============================================================

    def disconnect_serial(self):

        self.serial_manager.disconnect()

        self.status_label.setText(
            "Disconnected"
        )

        self.log(
            "Arduino disconnected."
        )

    # ============================================================
    # HOME
    # ============================================================

    def home_arm(self):

        if not self.serial_manager.is_connected():

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino is not connected."
            )

            return

        self.log(
            "Sending HOME command to Arduino..."
        )

        success = (
            self.serial_manager
            .send_home()
        )

        if success:

            self.q_current = np.zeros(
                self.robot.n
            )

            for label in (
                self.joint_result_labels
            ):

                label.setText(
                    "0.00°"
                )

            self.status_label.setText(
                "Homing..."
            )

            self.log(
                "HOME command sent successfully."
            )

        else:

            self.log(
                "Failed to send HOME command."
            )

    # ============================================================
    # FK
    # ============================================================

    def apply_fk(self):

        try:

            degrees = []

            for edit in self.fk_inputs:

                value = float(
                    edit.text()
                )

                degrees.append(
                    value
                )

            q = np.radians(
                degrees
            )

            T = self.robot.fkine(
                q
            )

            self.q_current = q.copy()

            self.log(
                f"FK calculated.\n"
                f"Position: {T.t}"
            )

            QMessageBox.information(
                self,
                "FK Result",
                f"End Effector Position:\n\n"
                f"X = {T.t[0]:.4f} m\n"
                f"Y = {T.t[1]:.4f} m\n"
                f"Z = {T.t[2]:.4f} m"
            )

        except Exception as e:

            QMessageBox.critical(
                self,
                "FK Error",
                str(e)
            )

    # ============================================================
    # GET TARGET POSE
    # ============================================================

    def get_target_pose(self):

        x = float(
            self.ik_inputs["x"].text()
        )

        y = float(
            self.ik_inputs["y"].text()
        )

        z = float(
            self.ik_inputs["z"].text()
        )

        roll = float(
            self.ik_inputs["roll"].text()
        )

        pitch = float(
            self.ik_inputs["pitch"].text()
        )

        yaw = float(
            self.ik_inputs["yaw"].text()
        )

        # Degrees -> radians

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
                [roll, pitch, yaw],
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

            q0 = (
                self.q_current.copy()
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

            self.q_current = (
                q_solution.copy()
            )

            degrees = np.degrees(
                q_solution
            )

            # --------------------------------------------------
            # DISPLAY J1-J5
            # --------------------------------------------------

            for i in range(
                min(
                    5,
                    len(degrees)
                )
            ):

                self.joint_result_labels[
                    i
                ].setText(
                    f"{degrees[i]:.2f}°"
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
    # CALCULATE IK + SEND TO ARDUINO
    # ============================================================

    def calculate_ik_and_send(self):

        # --------------------------------------------------------
        # 1. Calculate IK
        # --------------------------------------------------------

        result = (
            self.calculate_ik()
        )

        if result[0] is None:

            return

        degrees, success = result

        # --------------------------------------------------------
        # 2. Check Serial
        # --------------------------------------------------------

        if not self.serial_manager.is_ready():

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino is not READY.\n\n"
                "Connect and wait until "
                "HOMING is completed."
            )

            self.log(
                "IK calculated but Arduino "
                "is not READY."
            )

            return

        # --------------------------------------------------------
        # 3. First 3 joints only
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
        # 4. Convert to Arduino negative convention
        # --------------------------------------------------------

        if SEND_NEGATIVE_ANGLES:

            j1 = -abs(j1)

            j2 = -abs(j2)

            j3 = -abs(j3)

        # --------------------------------------------------------
        # 5. Arduino software limits
        # --------------------------------------------------------

        if not (
            -120 <= j1 <= 0
        ):

            QMessageBox.warning(
                self,
                "J1 Limit",
                f"J1 = {j1:.2f}°\n\n"
                "Allowed range: "
                "-120° to 0°"
            )

            return

        if not (
            -180 <= j2 <= 0
        ):

            QMessageBox.warning(
                self,
                "J2 Limit",
                f"J2 = {j2:.2f}°\n\n"
                "Allowed range: "
                "-180° to 0°"
            )

            return

        if not (
            -120 <= j3 <= 0
        ):

            QMessageBox.warning(
                self,
                "J3 Limit",
                f"J3 = {j3:.2f}°\n\n"
                "Allowed range: "
                "-120° to 0°"
            )

            return

        # --------------------------------------------------------
        # 6. SEND
        # --------------------------------------------------------

        self.log(
            "================================================"
        )

        self.log(
            "IK calculated successfully."
        )

        self.log(
            f"Arduino J1 = {j1:.2f}°"
        )

        self.log(
            f"Arduino J2 = {j2:.2f}°"
        )

        self.log(
            f"Arduino J3 = {j3:.2f}°"
        )

        command_text = (
            f"A,{j1:.2f},"
            f"{j2:.2f},"
            f"{j3:.2f}"
        )

        self.log(
            f"Sending -> {command_text}"
        )

        success_send = (
            self.serial_manager
            .send_angles(
                j1,
                j2,
                j3
            )
        )

        if success_send:

            self.log(
                f"SENT -> {command_text}"
            )

            self.status_label.setText(
                "Angles sent"
            )

        else:

            self.log(
                "ERROR: Failed to send angles."
            )

            QMessageBox.warning(
                self,
                "Serial",
                "Failed to send joint angles."
            )

    # ============================================================
    # DIRECT ARDUINO TEST
    # ============================================================

    def send_test_angles(self):

        try:

            j1 = float(
                self.test_inputs[0].text()
            )

            j2 = float(
                self.test_inputs[1].text()
            )

            j3 = float(
                self.test_inputs[2].text()
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
            -120 <= j1 <= 0
        ):

            QMessageBox.warning(
                self,
                "J1 Limit",
                "J1 must be between "
                "-120 and 0 degrees."
            )

            return

        if not (
            -180 <= j2 <= 0
        ):

            QMessageBox.warning(
                self,
                "J2 Limit",
                "J2 must be between "
                "-180 and 0 degrees."
            )

            return

        if not (
            -120 <= j3 <= 0
        ):

            QMessageBox.warning(
                self,
                "J3 Limit",
                "J3 must be between "
                "-120 and 0 degrees."
            )

            return

        # --------------------------------------------------------
        # READY CHECK
        # --------------------------------------------------------

        if not self.serial_manager.is_ready():

            QMessageBox.warning(
                self,
                "Arduino",
                "Arduino is not READY yet.\n\n"
                "Connect and wait for HOMING "
                "to finish."
            )

            return

        # --------------------------------------------------------
        # SEND
        # --------------------------------------------------------

        success = (
            self.serial_manager
            .send_angles(
                j1,
                j2,
                j3
            )
        )

        if success:

            self.log(
                "================================================"
            )

            self.log(
                f"SENT TEST -> "
                f"A,{j1:.2f},"
                f"{j2:.2f},"
                f"{j3:.2f}"
            )

            self.status_label.setText(
                "Test angles sent"
            )

        else:

            self.log(
                "Failed to send test angles."
            )

    # ============================================================
    # READ SERIAL
    # ============================================================

    def read_serial(self):

        messages = (
            self.serial_manager
            .read_available()
        )

        for message in messages:

            self.log(
                f"Arduino: {message}"
            )

            # ----------------------------------------------------
            # Update status after Arduino finishes HOME
            # ----------------------------------------------------

            if (
                message == "HOMING_DONE"
                or
                message == "READY"
            ):

                self.status_label.setText(
                    "Arduino READY"
                )

    # ============================================================
    # LOG
    # ============================================================

    def log(self, message):

        self.log_output.append(
            str(message)
        )

    # ============================================================
    # CLOSE
    # ============================================================

    def closeEvent(self, event):

        try:

            self.serial_manager.disconnect()

        except Exception:

            pass

        event.accept()


# ============================================================
# RUN APPLICATION
# ============================================================

if __name__ == "__main__":

    app = QApplication(
        sys.argv
    )

    window = MainWindow()

    window.show()

    sys.exit(
        app.exec_()
    )