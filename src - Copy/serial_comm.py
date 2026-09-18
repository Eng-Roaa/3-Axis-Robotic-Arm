import serial
import time
import threading


class SerialManager:

    def __init__(self, port=None, baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.lock = threading.Lock()

        # True only after Arduino says READY
        self.arduino_ready = False

    # =========================================================
    # CONNECT
    # =========================================================

    def connect(self):

        if not self.port:
            print("No serial port selected.")
            return False

        try:

            print(f"Opening {self.port}...")

            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                write_timeout=2
            )

            # Opening Serial resets Arduino
            time.sleep(2)

            # Clear old Python-side data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            self.arduino_ready = False

            print(
                f"Connected to {self.port} "
                f"at {self.baudrate} baud."
            )

            print("Waiting for Arduino HOMING...")

            # -------------------------------------------------
            # Wait for READY
            # -------------------------------------------------

            start_time = time.time()

            # Maximum wait for Arduino homing
            max_wait = 180

            while time.time() - start_time < max_wait:

                if self.serial_conn.in_waiting > 0:

                    line = (
                        self.serial_conn
                        .readline()
                        .decode(
                            "utf-8",
                            errors="ignore"
                        )
                        .strip()
                    )

                    if line:

                        print(
                            f"Arduino: {line}"
                        )

                        if line == "READY":

                            self.arduino_ready = True

                            print(
                                "Arduino is READY."
                            )

                            return True

                time.sleep(0.05)

            # -------------------------------------------------
            # Timeout
            # -------------------------------------------------

            print(
                "Timeout: Arduino did not send READY."
            )

            self.arduino_ready = False

            return False

        except Exception as e:

            print(
                f"Failed to connect to {self.port}: {e}"
            )

            self.serial_conn = None
            self.arduino_ready = False

            return False

    # =========================================================
    # DISCONNECT
    # =========================================================

    def disconnect(self):

        try:

            if self.serial_conn and self.serial_conn.is_open:

                self.serial_conn.close()

            self.arduino_ready = False

            print("Disconnected.")

        except Exception as e:

            print(
                f"Disconnect error: {e}"
            )

    # =========================================================
    # CONNECTION STATUS
    # =========================================================

    def is_connected(self):

        return (
            self.serial_conn is not None
            and self.serial_conn.is_open
        )

    # =========================================================
    # ARDUINO READY
    # =========================================================

    def is_ready(self):

        return (
            self.is_connected()
            and self.arduino_ready
        )

    # =========================================================
    # SEND HOME
    # =========================================================

    def send_home(self):

        if not self.is_connected():

            print(
                "Serial connection is not open."
            )

            return False

        command = "HOME\n"

        with self.lock:

            try:

                self.serial_conn.write(
                    command.encode("utf-8")
                )

                self.serial_conn.flush()

                print(
                    f"Sent: {command.strip()}"
                )

                # Arduino will be homing again
                self.arduino_ready = False

                return True

            except Exception as e:

                print(
                    f"Failed to send HOME: {e}"
                )

                return False

    # =========================================================
    # SEND 3 JOINT ANGLES
    # =========================================================

    def send_angles(self, j1, j2, j3):

        if not self.is_connected():

            print(
                "Serial connection is not open."
            )

            return False

        # -----------------------------------------------------
        # IMPORTANT:
        # Don't send movement while Arduino is still homing
        # -----------------------------------------------------

        if not self.arduino_ready:

            print(
                "Arduino is not READY yet."
            )

            return False

        payload = (
            f"A,"
            f"{float(j1):.2f},"
            f"{float(j2):.2f},"
            f"{float(j3):.2f}"
            f"\n"
        )

        with self.lock:

            try:

                print(
                    f"Sending: {payload.strip()}"
                )

                self.serial_conn.write(
                    payload.encode("utf-8")
                )

                self.serial_conn.flush()

                print(
                    "Command sent successfully."
                )

                return True

            except Exception as e:

                print(
                    f"Failed to send angles: {e}"
                )

                return False

    # =========================================================
    # READ AVAILABLE ARDUINO MESSAGES
    # =========================================================

    def read_available(self):

        if not self.is_connected():

            return []

        messages = []

        try:

            while self.serial_conn.in_waiting > 0:

                line = (
                    self.serial_conn
                    .readline()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                    .strip()
                )

                if line:

                    messages.append(line)

                    print(
                        f"Arduino: {line}"
                    )

                    # If Arduino finished a HOME command
                    if line == "HOMING_DONE":
                        self.arduino_ready = True

                    if line == "READY":
                        self.arduino_ready = True

            return messages

        except Exception as e:

            print(
                f"Serial read error: {e}"
            )

            return []