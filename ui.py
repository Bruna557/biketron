import os
import time
import json
import openvr
import vgamepad as vg
from PyQt6 import QtCore
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QGroupBox,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLineEdit,
    QDoubleSpinBox,
    QLabel,
    QCheckBox,
    QComboBox,
    QInputDialog,
)

from curve_editor import CurveEditor, MAX_CURVE_VALUE
from open_vr_wrapper import find_controllers, get_controller_state, make_state
from hardware import PedalSensor
from controller import update_virtual_gamepad, calculate_steering
from calibration import build_calibration

steeringSensitivity = 1.00
steeringSmoothing = 0.25

usePedalTracking = True
pedalSensitivity = 1.00
pedalSmoothing = 0.1
pedalDeadzone = 0.02

serialPort = "COM3"
baudRate = 115200
loopInterval = 0.01

CONFIG_DIR = "./configs"
os.makedirs(CONFIG_DIR, exist_ok=True)


class JoystickWorker(QtCore.QThread):
    update_graph_input_display = QtCore.pyqtSignal(int)
    update_input_display = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = False

    def start_loop(self):
        self.running = True
        self.start()

    def stop_loop(self):
        self.running = False
        self.mouseDeltaHistory = []

    def run(self):
        global window, pedalSensitivity, pedalSmoothing, pedalDeadzone, steeringSensitivity, steeringSmoothing, loopInterval

        print("running worker")

        try:
            openvr.init(openvr.VRApplication_Other)
            self.vr = openvr.VRSystem()
        except:
            self.vr = None
            print("Unable to connect to openvr. Won't use steering.")

        self.gamepad = vg.VX360Gamepad()

        pedal = None
        if usePedalTracking:
            pedal = PedalSensor(serialPort, baudRate)
            pedal.start()
        self.pedal = pedal

        axis = None
        s_left = None
        s_right = None

        filtered_speed = 0.0

        calibration_complete = False

        while True:
            current_pps = self.pedal.get_pps()

            speed = (
                current_pps / window.max_pulse_per_sec
                if window.max_pulse_per_sec
                else 0.0
            )
            speed_with_gain = speed * pedalSensitivity
            speed_with_gain = max(0.0, min(speed_with_gain, 1.0))

            # filtered_speed += (speed_with_gain - filtered_speed) * pedalSmoothing
            # filtered_speed = (
            #     filtered_speed * (1.0 - pedalSmoothing) + speed_with_gain * pedalSmoothing
            # )
            # if filtered_speed < pedalDeadzone:
            #     filtered_speed = 0.0

            pedal_curve_points = window.pedalCurve.get_or_build_curve_mapping()
            pedal_magnitude = window.interpolate_curve(
                speed_with_gain * MAX_CURVE_VALUE, pedal_curve_points
            )
            window.update_pedal_curve_input(pedal_magnitude)
            pedal_trigger = pedal_magnitude / MAX_CURVE_VALUE

            if self.vr:
                poses = self.vr.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
                )

                left_pos, right_pos, left_index, right_index = find_controllers(
                    self.vr, poses
                )

                left_controller_state = get_controller_state(self.vr, left_index)

                right_controller_state = get_controller_state(self.vr, right_index)

                # =================================================
                # ESTADO DO GUIDÃO
                # =================================================

                current_state = None

                if left_pos is not None and right_pos is not None:

                    current_state = make_state(left_pos, right_pos)
                    window.current_state = current_state

                # =================================================
                # STEERING
                # =================================================

                steering_stick_x = 0.0
                steering_stick_y = 0.0

                if (
                    current_state is not None
                    and window.state_center is not None
                    and window.state_right is not None
                    and window.state_left is not None
                    and window.max_pulse_per_sec is not None
                ):
                    if not calibration_complete:
                        calibration_complete = True
                        axis, s_left, s_right = build_calibration(
                            window.state_left, window.state_center, window.state_right
                        )

                    raw_steering, side = calculate_steering(
                        current_state, window.state_center, axis, s_left, s_right
                    )

                    steering_scaled_input = abs(raw_steering) * steeringSensitivity
                    steering_curve_points = (
                        window.steeringCurve.get_or_build_curve_mapping()
                    )
                    steering_magnitude = window.interpolate_curve(
                        steering_scaled_input * MAX_CURVE_VALUE, steering_curve_points
                    )
                    window.update_steering_curve_input(abs(steering_magnitude))
                    steering_stick_x = (
                        steering_magnitude
                        * (-1 if side == "LEFT" else 1)
                        / MAX_CURVE_VALUE
                    )

                # =================================================
                # XBOX
                # =================================================

                update_virtual_gamepad(
                    self.gamepad,
                    steering_stick_x,
                    steering_stick_y,
                    pedal_trigger,
                    left_controller_state,
                    right_controller_state,
                )

                print(f"stickX: {steering_stick_x}  pedal: {pedal_trigger}")

            time.sleep(loopInterval)


class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setWindowTitle("Biketron")
        self.setWindowIcon(QIcon("./resources/icon.webp"))

        self.worker = JoystickWorker()

        self.current_state = None
        self.state_center = None
        self.state_left = None
        self.state_right = None
        self.max_pulse_per_sec = None

        # Group: Internal Settings
        internalSettingsGroup = QGroupBox("Internal Settings")

        internalSettingsLayout = QFormLayout()
        self.serialPortLine = QLineEdit(str(serialPort))
        self.serialPortLine.setToolTip("Adjust he serial port to connect to the ESP32.")
        self.serialPortLine.textChanged.connect(self.setSerialPort)

        self.baudRateLine = QLineEdit(str(baudRate))
        self.baudRateLine.setToolTip("Adjust the baud rate for serial connection.")
        self.baudRateLine.textChanged.connect(self.setBaudRate)

        self.loopIntervalLine = QLineEdit(str(loopInterval))
        self.loopIntervalLine.setToolTip("Adjust the sleep time for the main loop.")
        self.loopIntervalLine.textChanged.connect(self.setLoopInterval)

        internalSettingsLayout.addRow("Serial Port:", self.serialPortLine)
        internalSettingsLayout.addRow("Baudrate:", self.baudRateLine)
        internalSettingsLayout.addRow("Loop Interval:", self.loopIntervalLine)

        internalSettingsGroup.setLayout(internalSettingsLayout)

        # Group: Config Management
        configGroup = QGroupBox("Configuration")
        configGroup.setToolTip("Save and load user configurations for reuse.")
        configLayout = QVBoxLayout()
        self.configDropdown = QComboBox()
        self.configDropdown.setToolTip("Select from saved configurations.")
        self.loadConfigButton = QPushButton("Load Config")
        self.loadConfigButton.setToolTip("Load the selected configuration.")
        self.saveConfigButton = QPushButton("Save Config")
        self.saveConfigButton.setToolTip(
            "Save the current settings under a custom name."
        )

        self.loadConfigButton.clicked.connect(self.load_config)
        self.saveConfigButton.clicked.connect(lambda: self.save_config())

        configLayout.addWidget(self.configDropdown)
        configLayout.addWidget(self.loadConfigButton)
        configLayout.addWidget(self.saveConfigButton)
        configGroup.setLayout(configLayout)

        self.update_config_dropdown()

        # Group: Sterring Settings
        steeringGroup = QGroupBox("Steering Settings")

        steeringLayout = QFormLayout()
        self.steeringSensitivityLine = QDoubleSpinBox()
        self.steeringSensitivityLine.setRange(0.0, 100.0)
        self.steeringSensitivityLine.setSingleStep(0.5)
        self.steeringSensitivityLine.setDecimals(1)
        self.steeringSensitivityLine.setLocale(
            QtCore.QLocale(QtCore.QLocale.Language.C)
        )
        self.steeringSensitivityLine.setValue(steeringSensitivity)
        self.steeringSensitivityLine.setToolTip(
            "Adjust the sensitivity multiplier for mouse movement to joystick input."
        )
        self.steeringSensitivityLine.textChanged.connect(self.setSteeringSensitivity)

        self.steeringSmoothingLine = QDoubleSpinBox()
        self.steeringSmoothingLine.setRange(0.0, 100.0)
        self.steeringSmoothingLine.setSingleStep(0.1)
        self.steeringSmoothingLine.setDecimals(1)
        self.steeringSmoothingLine.setLocale(QtCore.QLocale(QtCore.QLocale.Language.C))
        self.steeringSmoothingLine.setValue(steeringSensitivity)
        self.steeringSmoothingLine.setToolTip(
            "Adjust the temporal do tracking smoothing; 1.0 = no smoothing."
        )
        self.steeringSmoothingLine.textChanged.connect(self.setSteeringSmoothing)

        steeringLayout.addRow("Sensitivity:", self.steeringSensitivityLine)
        steeringLayout.addRow("Smoothing:", self.steeringSmoothingLine)

        steeringGroup.setLayout(steeringLayout)

        steeringCurveGroup = QGroupBox("Steering Sensitivity Curve")
        steeringCurveGroup.setToolTip(
            "Fine-tune how sensitivity scales with movement using a custom curve."
        )
        steeringCurveLayout = QVBoxLayout()
        self.steeringCurve = CurveEditor()
        self.steeringCurve.setToolTip(
            "Left-click + drag to move. Double-click to add. Right-click to delete."
        )
        steeringCurveLayout.addWidget(self.steeringCurve)
        steeringCurveGroup.setLayout(steeringCurveLayout)

        # Group: Pedal Settings
        pedalGroup = QGroupBox("Pedal Settings")

        pedalLayout = QFormLayout()
        self.pedalSensitivityLine = QDoubleSpinBox()
        self.pedalSensitivityLine.setRange(0.0, 100.0)
        self.pedalSensitivityLine.setSingleStep(0.5)
        self.pedalSensitivityLine.setDecimals(1)
        self.pedalSensitivityLine.setLocale(QtCore.QLocale(QtCore.QLocale.Language.C))
        self.pedalSensitivityLine.setValue(steeringSensitivity)
        self.pedalSensitivityLine.setToolTip(
            "Adjust the sensitivity multiplier for mouse movement to joystick input."
        )
        self.pedalSensitivityLine.textChanged.connect(self.setPedalSensitivity)

        self.usePedalCheckbox = QCheckBox("Pedal Tracking")
        self.usePedalCheckbox.setToolTip(
            "Use pulses from the backwheel sensor to control in-game speed. If disabled, control speed with R2."
        )
        self.usePedalCheckbox.setChecked(usePedalTracking)
        self.usePedalCheckbox.stateChanged.connect(self.togglePedalTracking)

        pedalLayout.addRow("Sensitivity:", self.pedalSensitivityLine)
        pedalLayout.addRow("Speed Source:", self.usePedalCheckbox)

        pedalGroup.setLayout(pedalLayout)

        pedalCurveGroup = QGroupBox("Pedal Sensitivity Curve")
        pedalCurveGroup.setToolTip(
            "Fine-tune how sensitivity scales with movement using a custom curve."
        )
        pedalCurveLayout = QVBoxLayout()
        self.pedalCurve = CurveEditor()
        self.pedalCurve.setToolTip(
            "Left-click + drag to move. Double-click to add. Right-click to delete."
        )
        pedalCurveLayout.addWidget(self.pedalCurve)
        pedalCurveGroup.setLayout(pedalCurveLayout)

        # Group: Calibration
        ppsCalibrationGroup = QGroupBox("PPS Calibration")
        ppsLayout = QVBoxLayout()
        calibrationPpsLabel = QLabel()
        calibrationPpsLabel.setText("Current state:")
        self.calibrationPpsLine = QLineEdit(str(self.max_pulse_per_sec or 0))
        self.calibratePps = QPushButton("Calibrate")
        self.calibratePps.setToolTip(
            "Pedal at your max confortable speed, then click this button."
        )
        self.calibratePps.clicked.connect(self.setPps)
        ppsLayout.addWidget(calibrationPpsLabel)
        ppsLayout.addWidget(self.calibrationPpsLine)
        ppsLayout.addWidget(self.calibratePps)
        ppsCalibrationGroup.setLayout(ppsLayout)

        centerCalibrationGroup = QGroupBox("Center Calibration")
        centerLayout = QVBoxLayout()
        calibrationCenterLabel = QLabel()
        calibrationCenterLabel.setText("Current state:")
        self.calibrationCenterLine = QLabel()
        self.calibrationCenterLine.setText(self.state_center or "0.00, 0.00, 0.00")
        self.calibrateCenter = QPushButton("Calibrate")
        self.calibrateCenter.setToolTip(
            "Move the steering to the central position, then click this button."
        )
        self.calibrateCenter.clicked.connect(self.setCenter)
        centerLayout.addWidget(calibrationCenterLabel)
        centerLayout.addWidget(self.calibrationCenterLine)
        centerLayout.addWidget(self.calibrateCenter)
        centerCalibrationGroup.setLayout(centerLayout)

        rightCalibrationGroup = QGroupBox("Right Calibration")
        rightLayout = QVBoxLayout()
        calibrationRightLabel = QLabel()
        calibrationRightLabel.setText("Current state:")
        self.calibrationRightLine = QLabel()
        self.calibrationRightLine.setText(self.state_right or "0.00, 0.00, 0.00")
        self.calibrateRight = QPushButton("Calibrate")
        self.calibrateRight.setToolTip(
            "Move the steering close to 90° right, then click this button."
        )
        self.calibrateRight.clicked.connect(self.setRight)
        rightLayout.addWidget(calibrationRightLabel)
        rightLayout.addWidget(self.calibrationRightLine)
        rightLayout.addWidget(self.calibrateRight)
        rightCalibrationGroup.setLayout(rightLayout)

        leftCalibrationGroup = QGroupBox("Left Calibration")
        leftLayout = QVBoxLayout()
        calibrationLeftLabel = QLabel()
        calibrationLeftLabel.setText("Current state:")
        self.calibrationLeftLine = QLabel()
        self.calibrationLeftLine.setText(self.state_left or "0.00, 0.00, 0.00")
        self.calibrateLeft = QPushButton("Calibrate")
        self.calibrateLeft.setToolTip(
            "Move the steering close to 90° left, then click this button."
        )
        self.calibrateLeft.clicked.connect(self.setLeft)
        leftLayout.addWidget(calibrationLeftLabel)
        leftLayout.addWidget(self.calibrationLeftLine)
        leftLayout.addWidget(self.calibrateLeft)
        leftCalibrationGroup.setLayout(leftLayout)

        # Main Layout
        mainLayout = QHBoxLayout()

        column1 = QVBoxLayout()
        column2 = QVBoxLayout()
        column3 = QVBoxLayout()

        column1.addWidget(internalSettingsGroup)
        column1.addWidget(steeringGroup)
        column1.addWidget(steeringCurveGroup)
        column2.addWidget(configGroup)
        column2.addWidget(pedalGroup)
        column2.addWidget(pedalCurveGroup)
        column3.addWidget(ppsCalibrationGroup)
        column3.addWidget(centerCalibrationGroup)
        column3.addWidget(rightCalibrationGroup)
        column3.addWidget(leftCalibrationGroup)

        mainLayout.addLayout(column1)
        mainLayout.addLayout(column2)
        mainLayout.addLayout(column3)

        self.setLayout(mainLayout)

        self.show()
        self.worker.start_loop()

        latest_config_name = next(
            (f for f in os.listdir(CONFIG_DIR) if f.startswith("latest_")), None
        )
        if latest_config_name:
            try:
                latest_config_path = os.path.join(CONFIG_DIR, latest_config_name)
                with open(latest_config_path, "r") as f:
                    config = json.load(f)
                    self.apply_config(config)
                    self.update_config_dropdown(latest_config_name[:-5])
                    print("Loaded last run config on startup.")
            except Exception as e:
                print(f"Failed to load last run config: {e}")

    def setSteeringSensitivity(self, value):
        self.steeringSensitivity = value
        print(self.steeringSensitivity)

    def setSteeringSmoothing(self, value):
        self.steeringSmoothing = value

    def setPedalSensitivity(self, value):
        self.pedalSensitivity = value

    def togglePedalTracking(self, state):
        global usePedalTracking
        usePedalTracking = state == 2
        print(f"Track pedal: {'enabled' if usePedalTracking else 'disabled'}")

    def setSerialPort(self, value):
        global serialPort
        serialPort = value
        print("serialPort:", value)

    def setBaudRate(self, value):
        global baudRate
        baudRate = value
        print("baudRate:", value)

    def setLoopInterval(self, value):
        global loopInterval
        try:
            val = float(value)
            if val <= 0:
                raise ValueError
            loopInterval = val
            print("loopInterval:", val)
        except ValueError:
            print("Invalid Loop Interval")

    def interpolate_curve(self, input_value, curve):
        """Linearly interpolate output from the curve based on input."""
        for i in range(len(curve) - 1):
            x1, y1 = curve[i]
            x2, y2 = curve[i + 1]
            if x1 <= input_value <= x2:
                # Linear interpolation
                ratio = (input_value - x1) / (x2 - x1)
                return y1 + ratio * (y2 - y1)
        # If input is out of bounds, clamp to end values
        if input_value < curve[0][0]:
            return curve[0][1]
        else:
            return curve[-1][1]

    def update_pedal_curve_input(self, input_value: int):
        if hasattr(self, "pedalCurve"):
            self.pedalCurve.set_current_input(input_value)

    def update_steering_curve_input(self, input_value: int):
        if hasattr(self, "steeringCurve"):
            self.steeringCurve.set_current_input(input_value)

    def get_current_config(self):
        return {
            "serial_port": self.serialPortLine.text(),
            "baud_rate": self.baudRateLine.text(),
            "loop_interval": self.loopIntervalLine.text(),
            "steering_sensitivity": self.steeringSensitivityLine.text(),
            "steering_smoothing": self.steeringSmoothingLine.text(),
            "pedal_sensitivity": self.pedalSensitivityLine.text(),
            "max_pulse_per_sec": self.calibrationPpsLine.text(),
            "use_pedal_tracking": self.usePedalCheckbox.isChecked(),
            "steering_curve_points": (
                self.steeringCurve.serialize_points()
                if hasattr(self, "steeringCurve")
                else None
            ),
            "pedal_curve_points": (
                self.pedalCurve.serialize_points()
                if hasattr(self, "pedalCurve")
                else None
            ),
        }

    def apply_config(self, config):
        self.serialPortLine.setText(str(config.get("serial_port_line", "COM3")))
        self.baudRateLine.setText(str(config.get("baud_rate_line", "115200")))
        self.loopIntervalLine.setText(str(config.get("loop_interval", "0.01")))
        self.steeringSensitivityLine.setValue(
            float(config.get("steering_sensitivity", "1.00"))
        )
        self.steeringSmoothingLine.setValue(
            float(config.get("steering_smoothing", "0.25"))
        )
        self.pedalSensitivityLine.setValue(
            float(config.get("pedal_sensitivity", "1.00"))
        )
        self.usePedalCheckbox.setChecked(config.get("use_pedal_tracking", True))

        max_pps = config.get("max_pulse_per_sec", "0")
        self.max_pulse_per_sec = int(max_pps)
        self.calibrationPpsLine.setText(max_pps)

        steering_curve_points_data = config.get("steering_curve_points")
        if steering_curve_points_data:
            self.steeringCurve.deserialize_points(steering_curve_points_data)

        pedal_curve_points_data = config.get("pedal_curve_points")
        if pedal_curve_points_data:
            self.pedalCurve.deserialize_points(pedal_curve_points_data)

    def save_config(self, name=None):
        current_name = self._save_config(name)
        self._save_config(f"latest_{current_name}")

    def _save_config(self, name=None):
        if name is None:
            text, ok = QInputDialog.getText(self, "Save Config", "Enter config name:")
            if not ok or not text.strip():
                print("Save cancelled or name was empty.")
                return
            name = text.strip()
        elif name.startswith("latest_"):
            old_latest = next(
                (f for f in os.listdir(CONFIG_DIR) if f.startswith("latest_")), None
            )
            if old_latest:
                path = os.path.join(CONFIG_DIR, old_latest)
                if path:
                    os.remove(path)

        config = self.get_current_config()
        path = os.path.join(CONFIG_DIR, f"{name}.json")
        try:
            with open(path, "w") as f:
                json.dump(config, f, indent=4)
            print(f"Config '{name}' saved.")
            self.update_config_dropdown(name)
            return name
        except Exception as e:
            print(f"Failed to save config: {e}")

    def load_config(self):
        name = self.configDropdown.currentText()
        if not name:
            return
        path = os.path.join(CONFIG_DIR, f"{name}.json")
        try:
            with open(path, "r") as f:
                config = json.load(f)
                self.apply_config(config)
                print(f"Config '{name}' loaded.")
                self._save_config(f"latest_{name}")
        except Exception as e:
            print(f"Failed to load config '{name}': {e}")

    def update_config_dropdown(self, name=None):
        self.configDropdown.clear()
        configs = [f[:-5] for f in os.listdir(CONFIG_DIR) if f.endswith(".json")]

        if name is not None:
            configs.insert(0, configs.pop(configs.index(name)).replace("latest_", ""))

        self.configDropdown.addItems(configs)

    def setCenter(self):
        if self.current_state is None:
            print("Both Sense controllers must be connected")
        else:
            self.state_center = self.current_state.copy()
            print(self.state_center)
            self.calibrationCenterLine.setText(
                self.format_sense_state(self.state_center)
            )

    def setRight(self):
        if self.current_state is None:
            print("Both Sense controllers must be connected")
        else:
            self.state_right = self.current_state.copy()
            self.calibrationRightLine.setText(self.format_sense_state(self.state_right))

    def setLeft(self):
        if self.current_state is None:
            print("Both Sense controllers must be connected")
        else:
            self.state_left = self.current_state.copy()
            self.calibrationLeftLine.setText(self.format_sense_state(self.state_left))

    def setPps(self):
        current_pps = self.pedal.get_pps()
        self.max_pulse_per_sec = int(current_pps)
        print(self.max_pulse_per_sec)

    def format_sense_state(self, state):
        return ", ".join(map(lambda x: str(x)[0:4], state))[:-18]


try:
    app = QApplication([])
    window = MainWindow()
    window.show()
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(100)

    app.exec()
except KeyboardInterrupt:
    openvr.shutdown()
