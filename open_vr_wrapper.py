import openvr
import numpy as np


def make_state(left_pos, right_pos):
    """
    Estado completo do guidão:

        [Lx, Ly, Lz, Rx, Ry, Rz]

    Usa somente posição dos Sense.
    """

    return np.concatenate(
        (np.asarray(left_pos, dtype=float), np.asarray(right_pos, dtype=float))
    )


def find_controllers(vr, poses):

    left_position = None
    right_position = None

    left_index = None
    right_index = None

    for i in range(openvr.k_unMaxTrackedDeviceCount):

        if vr.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
            continue

        pose = poses[i]

        if not pose.bPoseIsValid:
            continue

        role = vr.getControllerRoleForTrackedDeviceIndex(i)

        matrix = pose.mDeviceToAbsoluteTracking

        position = np.array(
            [
                matrix[0][0],
                matrix[0][1],
                matrix[0][2],
                matrix[1][0],
                matrix[1][1],
                matrix[1][2],
                matrix[2][0],
                matrix[2][1],
                matrix[2][2],
            ],
            dtype=float,
        )

        if role == openvr.TrackedControllerRole_LeftHand:

            left_position = position
            left_index = i

        elif role == openvr.TrackedControllerRole_RightHand:

            right_position = position
            right_index = i

    return (left_position, right_position, left_index, right_index)


def get_controller_state(vr, device_index):

    if device_index is None:
        return None

    try:

        success, state = vr.getControllerState(device_index)

        if not success:
            return None

        return state

    except Exception:

        return None
