import time
from pyb import UART


# OpenMV Cam H7 Plus UART3 loopback:
#   P4 = UART3 TX
#   P5 = UART3 RX
# Connect P4 directly to P5 before running this test.
# Disconnect P4/P5 from the STM32 to avoid connecting two TX outputs together.

UART_BAUDRATE = 115200
TEST_INTERVAL_MS = 500
LOOPBACK_WAIT_MS = 20

TEST_FRAME = bytes((0x66, 0x66, 0x82))


uart = UART(
    3,
    UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    timeout=100,
    timeout_char=10
)

print("UART3 LOOPBACK: connect P4(TX) to P5(RX)")

while True:
    # Discard any bytes left from the previous test cycle.
    uart.read()

    written = uart.write(TEST_FRAME)
    print("TX[%s]: 66 66 82" % str(written))

    time.sleep_ms(LOOPBACK_WAIT_MS)

    rx_data = uart.read()

    if rx_data:
        rx_text = []

        for value in rx_data:
            rx_text.append("%02X" % value)

        print(
            "RX[%d]: %s"
            % (len(rx_data), " ".join(rx_text))
        )

        if rx_data == TEST_FRAME:
            print("LOOPBACK_OK")
        else:
            print("LOOPBACK_DATA_ERROR")
    else:
        print("LOOPBACK_NONE")

    time.sleep_ms(TEST_INTERVAL_MS)
