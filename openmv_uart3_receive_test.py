import time
from pyb import UART


# OpenMV Cam H7 Plus UART3:
# P4 = TX
# P5 = RX <- STM32 PB10 (USART3_TX)
UART_BAUDRATE = 115200
NONE_REPORT_INTERVAL_MS = 500


uart = UART(
    3,
    UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    timeout=100,
    timeout_char=10
)

print("UART3 RX TEST: 115200 8N1, RX=P5")

last_none_report_ms = time.ticks_ms()



while True:
    received = uart.read()

    if received:
        hex_data = []

        for value in received:
            hex_data.append("%02X" % value)

        print(
            "RX[%d]: %s  %s"
            % (
                len(received),
                " ".join(hex_data),
                repr(received)
            )
        )
    elif time.ticks_diff(
        time.ticks_ms(),
        last_none_report_ms
    ) >= NONE_REPORT_INTERVAL_MS:
        print("none")
        last_none_report_ms = time.ticks_ms()

    time.sleep_ms(10)
