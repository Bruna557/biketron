import serial
import threading
import time


class PedalSensor:
    def __init__(
        self,
        port="COM3",
        baudrate=115200,
        poll_hz=20.0
    ):
        self.port = port
        self.baudrate = baudrate

        self.poll_interval = (
            1.0 / poll_hz
        )

        self.serial = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()

        # Última amostra recebida do ESP32
        self.last_pulse_count = None
        self.last_timestamp = None

        # PPS calculado
        self.pps = 0.0

        # Para diagnóstico
        self.current_pulse_count = 0
        self.current_timestamp = 0

        self.last_receive_time = 0.0


    # ========================================================
    # START
    # ========================================================

    def start(self):

        self.serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0.2
        )

        # ESP32 frequentemente reseta quando abre a serial.
        # Dá tempo para ele voltar.
        time.sleep(2.0)

        # Descarta lixo de boot / dados antigos.
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        self.running = True

        self.thread = threading.Thread(
            target=self._poll_loop,
            daemon=True
        )

        self.thread.start()

        print(
            f"Pedal conectado em "
            f"{self.port} "
            f"({1.0 / self.poll_interval:.1f} Hz)"
        )


    # ========================================================
    # POLLING
    # ========================================================

    def _poll_loop(self):

        while self.running:

            cycle_start = time.monotonic()

            try:

                # ------------------------------------------------
                # Pergunta ao ESP32:
                #
                # "me dê pulse_count e timestamp"
                # ------------------------------------------------

                self.serial.write(b"R")

                self.serial.flush()

                # ------------------------------------------------
                # Espera resposta
                # ------------------------------------------------

                raw = (
                    self.serial
                    .readline()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                    .strip()
                )

                if raw:

                    self._parse_response(
                        raw
                    )

            except serial.SerialException as e:

                print(
                    f"\nErro serial: {e}"
                )

                time.sleep(0.5)

            # ----------------------------------------------------
            # Mantém frequência aproximadamente constante
            # ----------------------------------------------------

            elapsed = (
                time.monotonic()
                - cycle_start
            )

            sleep_time = (
                self.poll_interval
                - elapsed
            )

            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )


    # ========================================================
    # PARSE
    # ========================================================

    def _parse_response(
        self,
        raw
    ):

        try:

            parts = raw.split(",")

            if len(parts) != 2:
                return

            pulse_count = int(
                parts[0]
            )

            timestamp = int(
                parts[1]
            )

        except ValueError:
            return

        self._process_sample(
            pulse_count,
            timestamp
        )


    # ========================================================
    # PPS
    # ========================================================

    def _process_sample(
        self,
        pulse_count,
        timestamp
    ):

        with self.lock:

            # ----------------------------------------------------
            # Temos uma amostra anterior?
            # ----------------------------------------------------

            if (
                self.last_pulse_count
                is not None
                and
                self.last_timestamp
                is not None
            ):

                delta_pulses = (
                    pulse_count
                    - self.last_pulse_count
                )

                delta_ms = (
                    timestamp
                    - self.last_timestamp
                )

                # ------------------------------------------------
                # Proteção contra reset do ESP32
                # ------------------------------------------------

                if (
                    delta_pulses >= 0
                    and
                    delta_ms > 0
                ):

                    delta_seconds = (
                        delta_ms / 1000.0
                    )

                    self.pps = (
                        delta_pulses
                        / delta_seconds
                    )

                else:

                    # Provável reset/wrap
                    self.pps = 0.0

            # ----------------------------------------------------
            # Guarda amostra atual
            # ----------------------------------------------------

            self.last_pulse_count = (
                pulse_count
            )

            self.last_timestamp = (
                timestamp
            )

            self.current_pulse_count = (
                pulse_count
            )

            self.current_timestamp = (
                timestamp
            )

            self.last_receive_time = (
                time.monotonic()
            )


    # ========================================================
    # GETTERS
    # ========================================================

    def get_pps(self):

        with self.lock:

            pps = self.pps

            age = (
                time.monotonic()
                - self.last_receive_time
            )

        # Sem resposta recente = acelerador zero
        if age > 0.5:
            return 0.0

        return pps


    def get_debug(self):

        with self.lock:

            return (
                self.current_pulse_count,
                self.current_timestamp,
                self.pps
            )


    # ========================================================
    # STOP
    # ========================================================

    def stop(self):

        self.running = False

        if self.thread is not None:

            self.thread.join(
                timeout=1.0
            )

        if self.serial is not None:

            try:
                self.serial.close()
            except Exception:
                pass
