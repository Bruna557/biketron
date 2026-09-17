import numpy as np
import vgamepad as vg


SENSE_TO_XBOX = {

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

    # L1 -> LB
    ("left", 34):
        vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,

    # R1 -> RB
    ("right", 34):
        # vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
        vg.XUSB_BUTTON.XUSB_GAMEPAD_START,

    # L3
        ("left", 32):
            vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    
        # R3
        ("right", 32):
            vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,

    # --------------------------------------------------------
    # CREATE / OPTIONS
    # --------------------------------------------------------

    # # Create -> Back / View
    # ("left", 5):
    #     vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,

    # # Options -> Start / Menu
    # ("right", 5):
    #     vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
}

L2_AXIS = 1
R2_AXIS = 1


def button_pressed(state, bit):

    if state is None:
        return False

    return bool(
        state.ulButtonPressed
        & (1 << bit)
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
        side
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
# STEERING -> LEFT ANALOG
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
