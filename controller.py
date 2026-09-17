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

def update_virtual_gamepad(
    gamepad,
    steering_stick_x,
    steering_stick_y,
    pedal_trigger,
    left_state,
    right_state
):

    gamepad.left_joystick_float(
        x_value_float=steering_stick_x,
        y_value_float=steering_stick_y
    )

    update_sense_buttons(
        gamepad,
        left_state,
        right_state
    )

    l2 = get_trigger_value(
        left_state,
        L2_AXIS
    )

    r2 = pedal_trigger or get_trigger_value(
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
