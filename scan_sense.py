# Scan Sense controllers to find out which bit maps each button
# Run this, then press each button to get the corresponding bit
# Square
# Triangle
# L1
# L2
# L3
# Create

# Sense direito:
# Cross
# Circle
# R1
# R2
# R3
# Options

import time
import openvr


def bits(value):
    return [
        i
        for i in range(64)
        if value & (1 << i)
    ]


openvr.init(openvr.VRApplication_Other)
vr = openvr.VRSystem()

print("Scanner completo dos Sense")
print("==========================")
print()
print("Aperte CREATE e OPTIONS.")
print("Ctrl+C para sair.")
print()

last_states = {}


try:
    while True:

        for i in range(
            openvr.k_unMaxTrackedDeviceCount
        ):

            if (
                vr.getTrackedDeviceClass(i)
                != openvr.TrackedDeviceClass_Controller
            ):
                continue

            try:
                success, state = (
                    vr.getControllerState(i)
                )
            except Exception:
                continue

            if not success:
                continue

            role = (
                vr.getControllerRoleForTrackedDeviceIndex(i)
            )

            # Captura absolutamente tudo que conseguimos
            current = (
                state.ulButtonPressed,
                state.ulButtonTouched,

                state.rAxis[0].x,
                state.rAxis[0].y,

                state.rAxis[1].x,
                state.rAxis[1].y,

                state.rAxis[2].x,
                state.rAxis[2].y,

                state.rAxis[3].x,
                state.rAxis[3].y,

                state.rAxis[4].x,
                state.rAxis[4].y,
            )

            previous = last_states.get(i)

            if previous != current:

                print()
                print(
                    f"Device {i} | role={role}"
                )

                print(
                    f"Pressed raw: "
                    f"{state.ulButtonPressed}"
                )

                print(
                    "Pressed bits:",
                    bits(
                        state.ulButtonPressed
                    )
                )

                print(
                    f"Touched raw: "
                    f"{state.ulButtonTouched}"
                )

                print(
                    "Touched bits:",
                    bits(
                        state.ulButtonTouched
                    )
                )

                for axis in range(5):

                    print(
                        f"Axis {axis}: "
                        f"x={state.rAxis[axis].x:+.4f} "
                        f"y={state.rAxis[axis].y:+.4f}"
                    )

                last_states[i] = current

        time.sleep(0.005)


except KeyboardInterrupt:
    pass


finally:
    openvr.shutdown()