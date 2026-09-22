import numpy as np
import vgamepad as vg

SENSE_TO_XBOX = {
    # Square -> X
    ("left", 7): vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
    # Triangle -> Y
    ("left", 1): vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    # Cross -> A
    ("right", 7): vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    # Circle -> B
    ("right", 1): vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    # L1 -> LB
    ("left", 34): vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    # R1 -> RB
    ("right", 34): vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    # L3
    ("left", 32): vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    # R3
    ("right", 32): vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
}

L2_AXIS = 1
R2_AXIS = 1
L_STICK_AXIS = 0
R_STICK_AXIS = 0

ANALOG_DEADZONE = 0.20


def build_calibration(state_left, state_center, state_right):

    # Eixo principal do movimento entre os dois extremos.
    axis = state_right - state_left

    length = np.linalg.norm(axis)

    if length < 1e-6:
        raise RuntimeError("LEFT e RIGHT ficaram praticamente iguais.")

    axis /= length

    # Coordenadas dos extremos relativamente ao centro.
    s_left = np.dot(state_left - state_center, axis)

    s_right = np.dot(state_right - state_center, axis)

    # Garantir convenção:
    #
    # LEFT  < 0
    # RIGHT > 0

    if s_left > s_right:

        axis = -axis

        s_left = np.dot(state_left - state_center, axis)

        s_right = np.dot(state_right - state_center, axis)

    if s_left >= 0 or s_right <= 0:
        raise RuntimeError("CENTER não ficou entre LEFT e RIGHT.")

    return axis, s_left, s_right


def calculate_steering(state, state_center, axis, s_left, s_right):
    """
    Retorna steering físico normalizado:

        LEFT   = -1
        CENTER =  0
        RIGHT  = +1
    """

    s = np.dot(state - state_center, axis)

    if s < 0:

        steering = s / abs(s_left)

        side = "LEFT"

    elif s > 0:

        steering = s / abs(s_right)

        side = "RIGHT"

    else:

        steering = 0.0
        side = "CENTER"

    steering = float(np.clip(steering, -1.0, 1.0))

    return (steering, side)


def button_pressed(state, bit):

    if state is None:
        return False

    return bool(state.ulButtonPressed & (1 << bit))


def update_sense_buttons(gamepad, left_state, right_state):

    for (side, bit), xbox_button in SENSE_TO_XBOX.items():

        if side == "left":
            state = left_state
        else:
            state = right_state

        if button_pressed(state, bit):

            gamepad.press_button(button=xbox_button)

        else:

            gamepad.release_button(button=xbox_button)


def get_trigger_value(state, axis_index):

    if state is None:
        return 0.0

    value = state.rAxis[axis_index].x

    return float(np.clip(value, 0.0, 1.0))


def get_analog_value(state, axis_index):
    if state is None:
        return (0.0, 0.0)

    value_x = state.rAxis[axis_index].x
    value_y = state.rAxis[axis_index].y

    return (value_x, value_y)


def update_virtual_gamepad(
    gamepad, steering_stick_x, steering_stick_y, pedal_trigger, left_state, right_state
):

    # Right analog
    r_stick_x, r_stick_y = get_analog_value(right_state, R_STICK_AXIS)
    gamepad.right_joystick_float(r_stick_x, r_stick_y)

    # Right analog: from Sense or steering
    l_stick_x, l_stick_y = get_analog_value(right_state, L_STICK_AXIS)
    if l_stick_x > ANALOG_DEADZONE or l_stick_y > ANALOG_DEADZONE:
        gamepad.left_joystick_float(l_stick_x, l_stick_y)
    else:
        gamepad.left_joystick_float(
            x_value_float=steering_stick_x, y_value_float=steering_stick_y
        )

    # Triggers
    l2 = get_trigger_value(left_state, L2_AXIS)

    r2 = pedal_trigger

    gamepad.left_trigger_float(value_float=l2)

    gamepad.right_trigger_float(value_float=r2)

    # Other buttons
    update_sense_buttons(gamepad, left_state, right_state)

    # Bind Sense R2 to gamepad start since we can't get the options button to work
    start = get_trigger_value(right_state, R2_AXIS)
    if start > 0.8:
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_START)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_START)

    gamepad.update()
