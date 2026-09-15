import time
import msvcrt

import numpy as np
import openvr
import vgamepad as vg

from sensor import PedalSensor


# ============================================================
# CONFIGURAÇÃO
# ============================================================

ENABLE_PEDAL = True # true lê do ESP32; false lê o R2 do sense controller

LOOP_INTERVAL = 0.01

# Suavização temporal do tracking.
# 1.0 = sem smoothing.
SMOOTHING = 0.25

# Quanto do steering físico vai para o Xbox.
#
# Exemplo com 0.35:
# steering físico +1.0 -> Xbox X +0.35
# steering físico +0.5 -> Xbox X +0.175
STEERING_SENSITIVITY = 1.00

# Ajuste feito por [ e ]
SENSITIVITY_STEP = 0.05


# ============================================================
# MAPEAMENTO SENSE -> XBOX
# ============================================================
#
# AJUSTE OS BITS PARA OS VALORES QUE VOCÊ DESCOBRIU.
#
# Formato:
#
#   ("left"/"right", bit): botão Xbox
#
# ============================================================

SENSE_TO_XBOX = {

    # --------------------------------------------------------
    # FACE BUTTONS
    # --------------------------------------------------------

    # Square -> X
    ("left", 7):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_X,

    # Triangle -> Y
    ("left", 1):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,

    # Cross -> A
    ("right", 7):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_A,

    # Circle -> B
    ("right", 1):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_B,


    # --------------------------------------------------------
    # SHOULDERS
    # --------------------------------------------------------

    # L1 -> LB
    ("left", 34):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,

    # R1 -> RB
    ("right", 34):
        # vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_START,


    # --------------------------------------------------------
    # CREATE / OPTIONS
    # --------------------------------------------------------

    # # Create -> Back / View
    # ("left", 5):
    #     vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,

    # # Options -> Start / Menu
    # ("right", 5):
    #     vg.XUSB_BUTTON.XUSB_GAMEPAD_START,


    # --------------------------------------------------------
    # STICK CLICK
    # --------------------------------------------------------

    # L3
    ("left", 32):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,

    # R3
    ("right", 32):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
}


# ============================================================
# TRIGGERS
# ============================================================
#
# Ajuste depois de descobrir qual rAxis corresponde a L2/R2.
#
# Se não quiser usar os triggers ainda, coloque False.
# ============================================================

L2_AXIS = 1
R2_AXIS = 1


# ============================================================
# ESTADO 6D
# ============================================================

def make_state(left_pos, right_pos):
    """
    Estado completo do guidão:

        [Lx, Ly, Lz, Rx, Ry, Rz]

    Usa somente posição dos Sense.
    """

    return np.concatenate(
        (
            np.asarray(left_pos, dtype=float),
            np.asarray(right_pos, dtype=float)
        )
    )


# ============================================================
# CALIBRAÇÃO
# ============================================================

def build_calibration(
    state_left,
    state_center,
    state_right
):
    """
    Cria eixo LEFT -> RIGHT no espaço 6D.
    """

    axis = (
        state_right
        - state_left
    )

    length = np.linalg.norm(axis)

    if length < 1e-6:
        raise ValueError(
            "LEFT e RIGHT ficaram praticamente iguais."
        )

    axis = axis / length

    s_left = np.dot(
        state_left - state_center,
        axis
    )

    s_right = np.dot(
        state_right - state_center,
        axis
    )

    # Garante LEFT negativo / RIGHT positivo

    if s_left > s_right:

        axis = -axis

        s_left = np.dot(
            state_left - state_center,
            axis
        )

        s_right = np.dot(
            state_right - state_center,
            axis
        )

    if s_left >= 0:
        raise ValueError(
            f"LEFT não ficou negativo: {s_left:.6f}"
        )

    if s_right <= 0:
        raise ValueError(
            f"RIGHT não ficou positivo: {s_right:.6f}"
        )

    return (
        axis,
        s_left,
        s_right
    )


def calculate_steering(
    state,
    state_center,
    axis,
    s_left,
    s_right
):
    """
    Retorna steering físico normalizado:

        LEFT   = -1
        CENTER =  0
        RIGHT  = +1
    """

    s = np.dot(
        state - state_center,
        axis
    )

    if s < 0:

        steering = (
            s / abs(s_left)
        )

        side = "LEFT"

    elif s > 0:

        steering = (
            s / abs(s_right)
        )

        side = "RIGHT"

    else:

        steering = 0.0
        side = "CENTER"

    steering = float(
        np.clip(
            steering,
            -1.0,
            1.0
        )
    )

    return (
        steering,
        float(s),
        side
    )


# ============================================================
# OPENVR
# ============================================================

def find_controllers(vr, poses):

    left_position = None
    right_position = None

    left_index = None
    right_index = None

    for i in range(
        openvr.k_unMaxTrackedDeviceCount
    ):

        if (
            vr.getTrackedDeviceClass(i)
            != openvr.TrackedDeviceClass_Controller
        ):
            continue

        pose = poses[i]

        if not pose.bPoseIsValid:
            continue

        role = (
            vr.getControllerRoleForTrackedDeviceIndex(i)
        )

        matrix = (
            pose.mDeviceToAbsoluteTracking
        )

        position = np.array(
            [
                matrix[0][3],
                matrix[1][3],
                matrix[2][3]
            ],
            dtype=float
        )

        if (
            role
            == openvr.TrackedControllerRole_LeftHand
        ):

            left_position = position
            left_index = i

        elif (
            role
            == openvr.TrackedControllerRole_RightHand
        ):

            right_position = position
            right_index = i

    return (
        left_position,
        right_position,
        left_index,
        right_index
    )


def get_controller_state(vr, device_index):

    if device_index is None:
        return None

    try:

        success, state = (
            vr.getControllerState(
                device_index
            )
        )

        if not success:
            return None

        return state

    except Exception:

        return None


# ============================================================
# BOTÕES
# ============================================================

def button_pressed(state, bit):

    if state is None:
        return False

    return bool(
        state.ulButtonPressed
        & (1 << bit)
    )


def update_sense_buttons(
    gamepad,
    left_state,
    right_state
):

    for (
        side,
        bit
    ), xbox_button in SENSE_TO_XBOX.items():

        if side == "left":
            state = left_state
        else:
            state = right_state

        if button_pressed(
            state,
            bit
        ):

            gamepad.press_button(
                button=xbox_button
            )

        else:

            gamepad.release_button(
                button=xbox_button
            )


# ============================================================
# TRIGGERS
# ============================================================

def get_trigger_value(
    state,
    axis_index
):

    if state is None:
        return 0.0

    value = (
        state.rAxis[axis_index].x
    )

    return float(
        np.clip(
            value,
            0.0,
            1.0
        )
    )


# ============================================================
# STEERING -> XBOX
# ============================================================

def steering_to_stick(
    steering,
    sensitivity
):
    """
    Curva mais sensível perto do centro.

    exponent < 1:
        aumenta resposta perto do centro

    exponent = 1:
        linear

    exponent > 1:
        reduz resposta perto do centro
    """

    STEERING_EXPONENT = 0.65

    steering = float(
        np.clip(
            steering,
            -1.0,
            1.0
        )
    )

    sign = (
        1.0
        if steering >= 0
        else -1.0
    )

    magnitude = (
        abs(steering)
        ** STEERING_EXPONENT
    )

    x = sign * magnitude

    # sensitivity continua sendo um ganho geral
    x *= sensitivity

    x = float(
        np.clip(
            x,
            -1.0,
            1.0
        )
    )

    # IMPORTANTE:
    # steering não usa o eixo Y.
    y = 0.0

    return x, y


def update_virtual_gamepad(
    gamepad,
    pedal,
    max_pulse_per_sec,
    steering,
    sensitivity,
    left_state,
    right_state
):

    # --------------------------------------------------------
    # Analógico esquerdo
    # --------------------------------------------------------

    stick_x, stick_y = (
        steering_to_stick(
            steering,
            sensitivity
        )
    )

    gamepad.left_joystick_float(
        x_value_float=stick_x,
        y_value_float=stick_y
    )

    # --------------------------------------------------------
    # Botões
    # --------------------------------------------------------

    update_sense_buttons(
        gamepad,
        left_state,
        right_state
    )

    # --------------------------------------------------------
    # Triggers
    # --------------------------------------------------------

    l2 = 0.0
    r2 = 0.0


    l2 = get_trigger_value(
        left_state,
        L2_AXIS
    )

    if pedal is not None:
        pps = pedal.get_pps()
        r2 = pps_to_rt(
            pps,
            max_pulse_per_sec
        )
    else:
        r2 = get_trigger_value(
            right_state,
            R2_AXIS
        )

    gamepad.left_trigger_float(
        value_float=l2
    )

    gamepad.right_trigger_float(
        value_float=r2
    )

    gamepad.update()

    return (
        stick_x,
        stick_y,
        l2,
        r2
    )


# ============================================================
# PEDAL -> RT
# ============================================================

def pps_to_rt(
    pps,
    max_pps
):
    if (
        max_pps is None
        or
        max_pps <= 0
    ):
        return 0.0

    return float(
        np.clip(
            pps / max_pps,
            0.0,
            1.0
        )
    )


# ============================================================
# MAIN
# ============================================================

def main():

    global STEERING_SENSITIVITY

    print(
        "Iniciando OpenVR..."
    )

    openvr.init(
        openvr.VRApplication_Other
    )

    vr = openvr.VRSystem()

    print(
        "Criando Xbox 360 virtual..."
    )

    gamepad = (
        vg.VX360Gamepad()
    )

    pedal = None

    if ENABLE_PEDAL:        
        pedal = PedalSensor(
            port="COM3",
            baudrate=115200
        )

    pedal.start()

    # --------------------------------------------------------
    # Calibração
    # --------------------------------------------------------

    max_pulse_per_sec = None

    state_center = None
    state_left = None
    state_right = None

    axis = None

    s_left = None
    s_right = None

    calibration_complete = False

    # --------------------------------------------------------
    # Steering
    # --------------------------------------------------------

    smoothed_steering = 0.0

    print()
    print(
        "BIKETRON VR -> Xbox"
    )
    print(
        "=================="
    )
    print()

    print(
        "C = centro"
    )

    print(
        "L = máximo esquerdo"
    )

    print(
        "R = máximo direito"
    )

    print(
        "X = reset calibração"
    )

    print()
    print(
        "[ = diminuir sensibilidade"
    )

    print(
        "] = aumentar sensibilidade"
    )

    print()
    print(
        "Q = sair"
    )

    print()

    print(
        f"Sensibilidade inicial: "
        f"{STEERING_SENSITIVITY:.2f}"
    )

    print()

    try:

        while True:

            # =================================================
            # OPENVR
            # =================================================

            poses = (
                vr.getDeviceToAbsoluteTrackingPose(
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
                vr,
                poses
            )

            left_controller_state = (
                get_controller_state(
                    vr,
                    left_index
                )
            )

            right_controller_state = (
                get_controller_state(
                    vr,
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
            scalar_position = 0.0
            side = "WAIT"

            if (
                current_state is not None
                and
                calibration_complete
            ):

                (
                    raw_steering,
                    scalar_position,
                    side
                ) = calculate_steering(
                    current_state,
                    state_center,
                    axis,
                    s_left,
                    s_right
                )

                # ---------------------------------------------
                # SOMENTE smoothing.
                #
                # SEM deadzone.
                # SEM curva exponencial.
                # ---------------------------------------------

                smoothed_steering += (
                    raw_steering
                    - smoothed_steering
                ) * SMOOTHING

                if (
                    abs(raw_steering)
                    < 0.01
                ):

                    side = "CENTER"

            # =================================================
            # XBOX
            # =================================================

            (
                stick_x,
                stick_y,
                l2,
                r2
            ) = update_virtual_gamepad(
                gamepad,
                pedal,
                max_pulse_per_sec,
                smoothed_steering,
                STEERING_SENSITIVITY,
                left_controller_state,
                right_controller_state
            )

            # =================================================
            # DISPLAY
            # =================================================

            if calibration_complete:

                print(
                    "\r"
                    f"{side:6} "
                    f"| físico={raw_steering:+.3f} "
                    f"| smooth={smoothed_steering:+.3f} "
                    f"| sens={STEERING_SENSITIVITY:.2f} "
                    f"| Xbox analog={stick_x:+.3f} "
                    f"| R2 ={r2:+.3f}       ",
                    end="",
                    flush=True
                )

            elif current_state is not None:

                missing = []

                if state_center is None:
                    missing.append("C")

                if state_left is None:
                    missing.append("L")

                if state_right is None:
                    missing.append("R")

                if max_pulse_per_sec is None:
                    missing.append("M")
                

                print(
                    "\r"
                    f"Aguardando calibração: "
                    f"{' '.join(missing):8} "
                    f"| Xbox ativo       ",
                    end="",
                    flush=True
                )

            else:

                print(
                    "\r"
                    "Aguardando os dois Sense...       ",
                    end="",
                    flush=True
                )

            # =================================================
            # TECLADO
            # =================================================

            if msvcrt.kbhit():

                key = (
                    msvcrt
                    .getwch()
                    .lower()
                )

                # ------------------------------------------------
                # QUIT
                # ------------------------------------------------

                if key == "q":

                    print()
                    print(
                        "Saindo..."
                    )

                    break

                # ------------------------------------------------
                # SENSIBILIDADE -
                # ------------------------------------------------

                if key == "[":

                    STEERING_SENSITIVITY -= (
                        SENSITIVITY_STEP
                    )

                    STEERING_SENSITIVITY = max(
                        0.05,
                        STEERING_SENSITIVITY
                    )

                    print()
                    print(
                        f"Sensibilidade: "
                        f"{STEERING_SENSITIVITY:.2f}"
                    )

                    continue

                # ------------------------------------------------
                # SENSIBILIDADE +
                # ------------------------------------------------

                if key == "]":

                    STEERING_SENSITIVITY += (
                        SENSITIVITY_STEP
                    )

                    STEERING_SENSITIVITY = min(
                        1.0,
                        STEERING_SENSITIVITY
                    )

                    print()
                    print(
                        f"Sensibilidade: "
                        f"{STEERING_SENSITIVITY:.2f}"
                    )

                    continue

                # ------------------------------------------------
                # RESET
                # ------------------------------------------------

                if key == "x":

                    state_center = None
                    state_left = None
                    state_right = None

                    axis = None

                    s_left = None
                    s_right = None

                    calibration_complete = False

                    smoothed_steering = 0.0

                    print()
                    print(
                        "Calibração apagada."
                    )

                    continue

                # ------------------------------------------------
                # Calibração exige os dois Sense
                # ------------------------------------------------

                if current_state is None:

                    print()
                    print(
                        "Os dois Sense precisam "
                        "estar rastreados."
                    )

                    continue

                # ------------------------------------------------
                # CENTRO
                # ------------------------------------------------

                if key == "c":

                    state_center = (
                        current_state.copy()
                    )

                    calibration_complete = False

                    print()
                    print(
                        "CENTRO salvo."
                    )

                # ------------------------------------------------
                # LEFT
                # ------------------------------------------------

                elif key == "l":

                    state_left = (
                        current_state.copy()
                    )

                    calibration_complete = False

                    print()
                    print(
                        "MÁXIMO ESQUERDO salvo."
                    )

                # ------------------------------------------------
                # RIGHT
                # ------------------------------------------------

                elif key == "r":

                    state_right = (
                        current_state.copy()
                    )

                    calibration_complete = False

                    print()
                    print(
                        "MÁXIMO DIREITO salvo."
                    )

                # ------------------------------------------------
                # PEDAL
                # ------------------------------------------------
                if key == "m":

                    current_pps = pedal.get_pps()

                    if current_pps > 0:

                        max_pulse_per_sec = (
                            current_pps
                        )

                        print()
                        print(
                            "ACELERAÇÃO CALIBRADA:"
                        )

                        print(
                            f"{max_pulse_per_sec:.2f} PPS "
                            "= RT 100%"
                        )

                    else:

                        print()
                        print(
                            "PPS está em zero; "
                            "pedale antes de apertar M."
                        )

                # ------------------------------------------------
                # Temos C/L/R?
                # ------------------------------------------------

                if (
                    state_center is not None
                    and
                    state_left is not None
                    and
                    state_right is not None
                    and
                    max_pulse_per_sec is not None
                ):

                    try:

                        (
                            axis,
                            s_left,
                            s_right
                        ) = build_calibration(
                            state_left,
                            state_center,
                            state_right
                        )

                        calibration_complete = True

                        smoothed_steering = 0.0

                        print()
                        print()

                        print(
                            "=== CALIBRAÇÃO CONCLUÍDA ==="
                        )

                        print(
                            f"LEFT   = "
                            f"{s_left:+.6f}"
                        )

                        print(
                            "CENTER = +0.000000"
                        )

                        print(
                            f"RIGHT  = "
                            f"{s_right:+.6f}"
                        )

                        print(
                            f"MAX PPS  = "
                            f"{max_pulse_per_sec}"
                        )

                        print()

                        print(
                            f"Sensibilidade Xbox: "
                            f"{STEERING_SENSITIVITY:.2f}"
                        )

                        print()

                    except ValueError as e:

                        calibration_complete = False

                        print()
                        print()

                        print(
                            "ERRO NA CALIBRAÇÃO:"
                        )

                        print(e)

                        print()

            time.sleep(
                LOOP_INTERVAL
            )

    finally:

        print()
        print(
            "Resetando Xbox virtual..."
        )

        try:

            gamepad.reset()
            gamepad.update()

        except Exception:

            pass

        openvr.shutdown()

        print(
            "OpenVR finalizado."
        )


if __name__ == "__main__":
    main()
