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
    QLabel,
    QCheckBox,
    QComboBox,
    QInputDialog
)

from curve_editor import CurveEditor, MAX_CURVE_VALUE
from open_vr_wrapper import find_controllers, get_controller_state, make_state
from hardware import PedalSensor
from controller import update_virtual_gamepad, calculate_steering

steeringSensitivity = 1.00
steeringSmoothing = 0.25

usePedalTracking = True
pedalSensitivity = 1.00

serialPort = "COM3"
baudRate = 115200
loopInterval = 1

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
        global \
            window, \
            pedalSensitivity, \
            steeringSensitivity, \
            steeringSmoothing, \
            loopInterval

        print("running worker")
        window.current_state = ["1.000", "2.000", "3.000"]

        axis = None
        s_left = None
        s_right = None

        while True:
            current_pps = window.pedal.get_pps()
            pedal_scaled_input = current_pps * pedalSensitivity
            curve_lut = window.pedalCurve.get_or_build_curve_mapping()
            pedal_magnitude = window.interpolate_curve(pedal_scaled_input, curve_lut)
            window.update_pedal_curve_input(pedal_magnitude)
            pedal_trigger = pedal_magnitude / MAX_CURVE_VALUE

            poses = (
                window.vr.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding,
                    0,
                    openvr.k_unMaxTrackedDeviceCount
                )
            )

            (
                left_pos,
                right_pos,
                left_index,
                right_index
            ) = find_controllers(
                window.vr,
                poses
            )

            left_controller_state = (
                get_controller_state(
                    window.vr,
                    left_index
                )
            )

            right_controller_state = (
                get_controller_state(
                    window.vr,
                    right_index
                )
            )

            # =================================================
            # ESTADO DO GUIDÃO
            # =================================================

            current_state = None

            if (
                left_pos is not None
                and
                right_pos is not None
            ):

                current_state = (
                    make_state(
                        left_pos,
                        right_pos
                    )
                )

            # =================================================
            # STEERING
            # =================================================

            raw_steering = 0.0

            if (
                current_state is not None and
                window.state_center is not None and 
                window.state_right is not None and 
                window.state_left is not None and
                window.max_pulse_per_sec is not None
            ):

                raw_steering = calculate_steering(
                    current_state,
                    window.state_center,
                    axis,
                    s_left,
                    s_right
                )

                steering_scaled_input = raw_steering * steeringSensitivity
                steering_curve_lut = window.steeringCurve.get_or_build_curve_mapping()
                steering_magnitude = window.interpolate_curve(steering_scaled_input, steering_curve_lut)
                window.update_steering_curve_input(steering_magnitude)
                steering_stick_x = pedal_magnitude / MAX_CURVE_VALUE
                steering_stick_y = 0.0


            # =================================================
            # XBOX
            # =================================================

            update_virtual_gamepad(
                window.gamepad,
                steering_stick_x,
                steering_stick_y,
                pedal_trigger,
                left_controller_state,
                right_controller_state
            )

            time.sleep(loopInterval)


class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.setWindowTitle("Biketron")
        self.setWindowIcon(QIcon("./resources/icon.webp"))

        self.worker = JoystickWorker()
        # self.worker.update_graph_input_display.connect(self.update_pedal_curve_input)

        self.state_center = None
        self.state_left = None
        self.state_right = None
        self.max_pulse_per_sec = None

        openvr.init(openvr.VRApplication_Other)
        self.vr = openvr.VRSystem()
    
        self.gamepad = (vg.VX360Gamepad())

        pedal = None        
        if usePedalTracking:
            pedal = PedalSensor(serialPort, baudRate)
            pedal.start()
        self.pedal = pedal

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
        self.saveConfigButton.setToolTip("Save the current settings under a custom name.")

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
        self.steeringSensitivityLine = QLineEdit(str(steeringSensitivity))
        self.steeringSensitivityLine.setToolTip("Adjust the sensitivity multiplier for mouse movement to joystick input.")
        self.steeringSensitivityLine.textChanged.connect(self.setSteeringSensitivity)

        self.steeringSmoothingLine = QLineEdit(str(steeringSmoothing))
        self.steeringSmoothingLine.setToolTip("Adjust the temporal do tracking smoothing; 1.0 = no smoothing.")
        self.steeringSmoothingLine.textChanged.connect(self.setSteeringSmoothing)

        steeringLayout.addRow("Sensitivity:", self.steeringSensitivityLine)
        steeringLayout.addRow("Smoothing:", self.steeringSmoothingLine)

        steeringGroup.setLayout(steeringLayout)

        steeringCurveGroup = QGroupBox("Steering Sensitivity Curve")
        steeringCurveGroup.setToolTip("Fine-tune how sensitivity scales with movement using a custom curve.")
        steeringCurveLayout = QVBoxLayout()
        self.steeringCurve = CurveEditor()
        self.steeringCurve.setToolTip("Left-click + drag to move. Double-click to add. Right-click to delete.")
        steeringCurveLayout.addWidget(self.steeringCurve)
        steeringCurveGroup.setLayout(steeringCurveLayout)

        # Group: Pedal Settings
        pedalGroup = QGroupBox("Pedal Settings")

        pedalLayout = QFormLayout()
        self.pedalSensitivityLine = QLineEdit(str(pedalSensitivity))
        self.pedalSensitivityLine.setToolTip("Adjust the sensitivity multiplier for mouse movement to joystick input.")
        self.pedalSensitivityLine.textChanged.connect(self.setPedalSensitivity)

        self.usePedalCheckbox = QCheckBox("Pedal Tracking")
        self.usePedalCheckbox.setToolTip("Use pulses from the backwheel sensor to control in-game speed. If disabled, control speed with R2.")
        self.usePedalCheckbox.setChecked(usePedalTracking)
        self.usePedalCheckbox.stateChanged.connect(self.togglePedalTracking)

        pedalLayout.addRow("Sensitivity:", self.pedalSensitivityLine)
        pedalLayout.addRow("Speed Source:", self.usePedalCheckbox)

        pedalGroup.setLayout(pedalLayout)

        pedalCurveGroup = QGroupBox("Pedal Sensitivity Curve")
        pedalCurveGroup.setToolTip("Fine-tune how sensitivity scales with movement using a custom curve.")
        pedalCurveLayout = QVBoxLayout()
        self.pedalCurve = CurveEditor()
        self.pedalCurve.setToolTip("Left-click + drag to move. Double-click to add. Right-click to delete.")
        pedalCurveLayout.addWidget(self.pedalCurve)
        pedalCurveGroup.setLayout(pedalCurveLayout)

        # Group: Calibration
        centerCalibrationGroup = QGroupBox("Center Calibration")
        centerLayout = QVBoxLayout()
        calibrationCenterLabel = QLabel()
        calibrationCenterLabel.setText("Current state:")
        self.calibrationCenterLine = QLabel()
        self.calibrationCenterLine.setText(self.state_center or "0.0, 0.0, 0.0")
        self.calibrateCenter = QPushButton("Calibrate")
        self.calibrateCenter.setToolTip("Move the steering to the central position, then click this button.")
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
        self.calibrationRightLine.setText(self.state_right or "0.0, 0.0, 0.0")
        self.calibrateRight = QPushButton("Calibrate")
        self.calibrateRight.setToolTip("Move the steering close to 90° right, then click this button.")
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
        self.calibrationLeftLine.setText(self.state_left or "0.0, 0.0, 0.0")
        self.calibrateLeft = QPushButton("Calibrate")
        self.calibrateLeft.setToolTip("Move the steering close to 90° left, then click this button.")
        self.calibrateLeft.clicked.connect(self.setLeft)
        leftLayout.addWidget(calibrationLeftLabel)
        leftLayout.addWidget(self.calibrationLeftLine)
        leftLayout.addWidget(self.calibrateLeft)
        leftCalibrationGroup.setLayout(leftLayout)

        ppsCalibrationGroup = QGroupBox("PPS Calibration")
        ppsLayout = QVBoxLayout()
        calibrationPpsLabel = QLabel()
        calibrationPpsLabel.setText("Current state:")
        self.calibrationPpsLine = QLabel()
        self.calibrationPpsLine.setText(self.max_pulse_per_sec or "0.0, 0.0, 0.0")
        self.calibratePps = QPushButton("Calibrate")
        self.calibratePps.setToolTip("Pedal at your max confortable speed, then click this button.")
        self.calibratePps.clicked.connect(self.setPps)
        ppsLayout.addWidget(calibrationPpsLabel)
        ppsLayout.addWidget(self.calibrationPpsLine)
        ppsLayout.addWidget(self.calibratePps)
        ppsCalibrationGroup.setLayout(ppsLayout)

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
        column3.addWidget(centerCalibrationGroup)
        column3.addWidget(rightCalibrationGroup)
        column3.addWidget(leftCalibrationGroup)
        column3.addWidget(ppsCalibrationGroup)

        mainLayout.addLayout(column1)
        mainLayout.addLayout(column2)
        mainLayout.addLayout(column3)

        self.setLayout(mainLayout)

        self.show()

        latest_config_path = os.path.join(CONFIG_DIR, "last_saved_config.json")
        if os.path.exists(latest_config_path):
            try:
                with open(latest_config_path, "r") as f:
                    config = json.load(f)
                    self.apply_config(config)
                    self.update_config_dropdown("last_saved_config")
                    print("Loaded last run config on startup.")
            except Exception as e:
                print(f"Failed to load last run config: {e}")

    def setSteeringSensitivity(self, value):
        global steeringSensitivity
        try:
            val = float(value)
            if val <= 0:
                raise ValueError
            steeringSensitivity = val
            print("steeringSensitivity:", val)
        except ValueError:
            print("Invalid Steering Sensitivity")

    def setSteeringSmoothing(self, value):
        global steeringSmoothing
        try:
            val = float(value)
            if val <= 0:
                raise ValueError
            steeringSmoothing = val
            print("steeringSmoothing:", val)
        except ValueError:
            print("Invalid Steering Smoothing")

    def setPedalSensitivity(self, value):
        global pedalSensitivity
        try:
            val = float(value)
            if val <= 0:
                raise ValueError
            pedalSensitivity = val
            print("pedalSensitivity:", val)
        except ValueError:
            print("Invalid Pedal Sensitivity")

    def togglePedalTracking(self, state):
        global usePedalTracking
        usePedalTracking = state == 2
        if(usePedalTracking):
            self.worker.start_loop()
        else:
            self.worker.stop_loop()
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
        print("srrent input", input_value)            
        if (
            hasattr(self, "pedalCurve")
        ):
            self.pedalCurve.set_current_input(input_value)

    def get_current_config(self):
        return {
            "serial_port": self.serialPortLine.text(),
            "baud_rate": self.baudRateLine.text(),
            "loop_interval": self.loopIntervalLine.text(),
            "steering_sensitivity": self.steeringSensitivityLine.text(),
            "steering_smoothing": self.steeringSmoothingLine.text(),
            "pedal_sensitivity": self.pedalSensitivityLine.text(),
            "use_pedal_tracking": self.usePedalCheckbox.isChecked(),
            "steering_curve_points": self.steeringCurve.serialize_points()
                if hasattr(self, "steeringCurve")
                else None,
            "pedal_curve_points": self.pedalCurve.serialize_points()
                if hasattr(self, "pedalCurve")
                else None,
        }

    def apply_config(self, config):
        self.serialPortLine.setText(str(config.get("serial_port_line", "COM3")))
        self.baudRateLine.setText(str(config.get("baud_rate_line", "115200")))
        self.loopIntervalLine.setText(str(config.get("loop_interval", "0.01")))
        self.steeringSensitivityLine.setText(str(config.get("steering_sensitivity", "1.00")))
        self.steeringSmoothingLine.setText(str(config.get("steering_smoothing", "0.25")))
        self.pedalSensitivityLine.setText(str(config.get("pedal_sensitivity", "1.00")))
        self.usePedalCheckbox.setChecked(config.get("use_pedal_tracking", True))     

        steering_curve_points_data = config.get("steering_curve_points")
        if steering_curve_points_data:
            self.steeringCurve.deserialize_points(steering_curve_points_data)

        pedal_curve_points_data = config.get("pedal_curve_points")
        if pedal_curve_points_data:
            self.pedalCurve.deserialize_points(pedal_curve_points_data)

    def save_config(self, name=None):
        self._save_config("last_saved_config")
        self._save_config(name)

    def _save_config(self, name=None):
        if name is None:
            text, ok = QInputDialog.getText(self, "Save Config", "Enter config name:")
            if not ok or not text.strip():
                print("Save cancelled or name was empty.")
                return
            name = text.strip()

        config = self.get_current_config()
        path = os.path.join(CONFIG_DIR, f"{name}.json")
        try:
            with open(path, "w") as f:
                json.dump(config, f, indent=4)
            print(f"Config '{name}' saved.")
            self.update_config_dropdown(name)
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
        except Exception as e:
            print(f"Failed to load config '{name}': {e}")

    def update_config_dropdown(self, name=None):
        self.configDropdown.clear()
        configs = [f[:-5] for f in os.listdir(CONFIG_DIR) if f.endswith(".json")]

        if (name is not None):
            configs.insert(0, configs.pop(configs.index(name)))

        self.configDropdown.addItems(configs)

    def setCenter(self):
        if self.current_state is None:        
            print("Both Sense controllers must be connected")
        else:
            self.state_center = self.current_state.copy()
            print(self.state_center)
            self.calibrationCenterLine.setText(", ".join(self.state_center))

    def setRight(self):
        if self.current_state is None:        
            print("Both Sense controllers must be connected")
        else:
            self.state_right = self.current_state.copy()

    def setLeft(self):
        if self.current_state is None:        
            print("Both Sense controllers must be connected")
        else:
            self.state_left = self.current_state.copy()

    def setPps(self):
        current_pps = self.pedal.get_pps()
        if current_pps > 0:
            self.max_pulse_per_sec = current_pps


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
