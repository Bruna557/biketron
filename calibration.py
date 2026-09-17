import numpy as np

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
