import time
from pyb import UART


# OpenMV Cam H7 Plus UART3:
# P4 = TX -> MCU RX
# P5 = RX <- MCU TX (not required for this one-way test)
# OpenMV GND and MCU GND must be connected.
uart = UART(3, 115200, timeout_char=10)

# Give the UART a short time to become ready after the script starts.
time.sleep_ms(200)

# Protocol:
# 66 66 : frame header
# 02    : chassis translation command
# 00 00 : direction = 0 degrees (straight forward)
# 0A    : distance = 10 cm
# frame = bytearray((0x66, 0x66, 0x07))
frame = bytearray((0x66, 0x66, 0x02, 0x00, 0x00, 0x0A))

written = uart.write(frame)

