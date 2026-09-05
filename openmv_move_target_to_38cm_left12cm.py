import sensor
import time
import math
import pyb
from pyb import UART


# OpenMV Cam H7 Plus, firmware 4.5.9, QVGA (320 x 240).
# Replace these two lines when the new automatic LAB calibration finishes.
PURPLE_THRESHOLD = (62, 78, 7, 25, -39, -17)
ORANGE_THRESHOLD = (81, 97, -5, 16, 19, 38)

# "ANY" accepts either colour, but sends no command if both are visible.
# It may also be set to "PURPLE" or "ORANGE" for a known task colour.
TARGET_COLOR = "ANY"

# Desired final target position relative to the camera.
# Coordinate convention: forward is +Z; camera-right is +X.
TARGET_FORWARD_CM = 38.0
TARGET_RIGHT_CM = -12.0  # -12 cm means 12 cm to camera-left.

# Distance calibration from the five centred measurements:
# (w, Z_cm) = (120,30), (92,40), (74,50), (63,60), (54,70)
# Least-squares model: Z_cm = DISTANCE_A / w + DISTANCE_B.
DISTANCE_A = 3940.04622
DISTANCE_B = -2.881682

# Horizontal calibration at Z=50 cm, excluding the unstable +/-20 cm points.
# Least-squares pinhole model: X = (cx-CX_ZERO) * Z / HORIZONTAL_FOCAL_PX.
# X is positive to camera-right and negative to camera-left.
CX_ZERO = 157.714286
HORIZONTAL_FOCAL_PX = 340.917404

# The calibration is only trusted over approximately 30-70 cm.
MIN_VALID_FORWARD_CM = 28.0
MAX_VALID_FORWARD_CM = 75.0

# Stable-window and safety settings.
FILTER_WINDOW = 3
MAX_CX_RANGE_PX = 6
MAX_CY_RANGE_PX = 6
MAX_W_RANGE_PX = 5
MAX_H_RANGE_PX = 6
EDGE_MARGIN_PX = 2
# The groove may hide roughly half of the block vertically. In that case the
# visible colour region can be about twice as wide as it is tall, while its
# full width (used for distance estimation) remains available.
MIN_VISIBLE_ASPECT_RATIO = 0.60
MAX_VISIBLE_ASPECT_RATIO = 3.00
FORWARD_TOLERANCE_CM = 2.0
RIGHT_TOLERANCE_CM = 2.0
SETTLE_AFTER_DONE_MS = 500
WAIT_DONE_TIMEOUT_MS = 15000
PRINT_EVERY_N_FRAMES = 5

# After each 0x82 completion frame, allow this much time to obtain the
# 3-frame stable target. If no stable target is available, move one
# search step and try again after the MCU reports the next 0x82.
NO_TARGET_SEARCH_TIMEOUT_MS = 500
NO_TARGET_SEARCH_DISTANCE_CM = 10

# Protocol angle 90 degrees means "left". The real chassis currently has its
# lateral direction reversed, so this command moves the real vehicle right.
NO_TARGET_SEARCH_COMMAND_ANGLE_DEG = 90

# Set False when debugging vision without allowing automatic chassis movement.
ENABLE_NO_TARGET_SEARCH = True

# UART3 on OpenMV H7 Plus: TX=P4, RX=P5, 3.3 V logic, common ground.
UART_BAUDRATE = 115200

# True: wait for the MCU frame 66 66 82 before initializing the camera.
# False: initialize the camera immediately, which is convenient for debugging.
WAIT_MCU_82_BEFORE_CAMERA = True

# Print the first bytes received while waiting. If no byte reaches UART3 RX,
# print UART3_RX_NONE once per second.
DEBUG_CAMERA_START_RX = True

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

STATE_STABILIZE = 0
STATE_WAIT_DONE = 1
STATE_SETTLE = 2
STATE_FAULT = 3


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def round_int(value):
    if value >= 0:
        return int(value + 0.5)
    return int(value - 0.5)


def median(values):
    ordered = list(values)
    ordered.sort()
    return ordered[len(ordered) // 2]


def find_largest(blobs):
    largest = None
    for blob in blobs:
        if largest is None or blob.pixels() > largest.pixels():
            largest = blob
    return largest


def blob_is_complete(blob):
    if blob is None:
        return False

    if blob.x() <= EDGE_MARGIN_PX:
        return False
    if blob.y() <= EDGE_MARGIN_PX:
        return False
    if (blob.x() + blob.w()) >= (FRAME_WIDTH - EDGE_MARGIN_PX):
        return False
    if (blob.y() + blob.h()) >= (FRAME_HEIGHT - EDGE_MARGIN_PX):
        return False

    # A vertically half-hidden block in the groove is valid. We deliberately
    # allow a wide rectangle here because distance is calculated from width,
    # not height. A very narrow region is still rejected because horizontal
    # occlusion would make the estimated distance incorrect.
    if blob.h() <= 0:
        return False
    aspect = blob.w() / blob.h()
    return (
        aspect >= MIN_VISIBLE_ASPECT_RATIO
        and aspect <= MAX_VISIBLE_ASPECT_RATIO
    )


class StableTarget:
    def __init__(self):
        self.label = None
        self.samples = []

    def clear(self):
        self.label = None
        self.samples = []

    def add(self, label, blob):
        if self.label != label:
            self.clear()
            self.label = label

        self.samples.append((blob.cx(), blob.cy(), blob.w(), blob.h()))
        if len(self.samples) > FILTER_WINDOW:
            self.samples.pop(0)

    def result(self):
        if len(self.samples) < FILTER_WINDOW:
            return None

        cxs = [sample[0] for sample in self.samples]
        cys = [sample[1] for sample in self.samples]
        widths = [sample[2] for sample in self.samples]
        heights = [sample[3] for sample in self.samples]

        # Checking the whole window prevents a slowly drifting target from being
        # declared stable merely because adjacent frames happen to be close.
        if (max(cxs) - min(cxs)) > MAX_CX_RANGE_PX:
            return None
        if (max(cys) - min(cys)) > MAX_CY_RANGE_PX:
            return None
        if (max(widths) - min(widths)) > MAX_W_RANGE_PX:
            return None
        if (max(heights) - min(heights)) > MAX_H_RANGE_PX:
            return None

        return (
            self.label,
            median(cxs),
            median(cys),
            median(widths),
            median(heights),
        )


def estimate_target_pose(cx, width_px):
    if width_px <= 0:
        return None

    forward_cm = DISTANCE_A / width_px + DISTANCE_B
    right_cm = (
        (cx - CX_ZERO) * forward_cm / HORIZONTAL_FOCAL_PX
    )
    return (forward_cm, right_cm)


def calculate_chassis_move(forward_cm, right_cm):
    # When the camera moves, the stationary target coordinate changes by the
    # opposite amount. Therefore these are the required chassis translations.
    move_forward_cm = forward_cm - TARGET_FORWARD_CM
    move_right_cm = right_cm - TARGET_RIGHT_CM

    distance_cm = math.sqrt(
        move_forward_cm * move_forward_cm
        + move_right_cm * move_right_cm
    )

    # Protocol convention: forward=0 degrees and counter-clockwise is positive.
    # Camera-right movement is clockwise, hence the minus sign here.
    direction_deg = math.atan2(-move_right_cm, move_forward_cm) * 57.2957795
    while direction_deg < 0:
        direction_deg += 360.0
    while direction_deg >= 360.0:
        direction_deg -= 360.0

    return (
        move_forward_cm,
        move_right_cm,
        direction_deg,
        distance_cm,
    )


def build_move_frame(direction_deg, distance_cm):
    angle = round_int(direction_deg) % 360
    distance = clamp(round_int(distance_cm), 0, 255)

    # 66 66 02 angle_hi angle_lo distance_cm
    frame = bytearray(
        (0x66, 0x66, 0x02, (angle >> 8) & 0xFF, angle & 0xFF, distance)
    )
    return (frame, angle, distance)


def print_frame(frame):
    print(
        "UART_TX: %02X %02X %02X %02X %02X %02X"
        % (frame[0], frame[1], frame[2], frame[3], frame[4], frame[5])
    )


uart = UART(
    3,
    UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    timeout=100,
    timeout_char=10
)


def wait_for_camera_start():
    """
    Do not initialize or capture from the camera until the MCU reports that
    the initial chassis task has finished with the frame 66 66 82.
    """
    start_rx_state = 0
    last_empty_report_ms = time.ticks_ms()
    raw_data_printed = False

    print("WAITING_CAMERA_START: 66 66 82")

    while True:
        rx_data = uart.read()

        if rx_data:
            if DEBUG_CAMERA_START_RX and not raw_data_printed:
                raw_text = []

                for raw_value in rx_data[:32]:
                    raw_text.append("%02X" % raw_value)

                print(
                    "UART3_RX_RAW[%d]: %s"
                    % (len(rx_data), " ".join(raw_text))
                )
                raw_data_printed = True

            for value in rx_data:
                if start_rx_state == 0:
                    if value == 0x66:
                        start_rx_state = 1
                elif start_rx_state == 1:
                    if value == 0x66:
                        start_rx_state = 2
                    else:
                        start_rx_state = 0
                else:
                    if value == 0x82:
                        print("CAMERA_START: 66 66 82")
                        return

                    # Keep the last two 0x66 bytes as a possible new header.
                    if value == 0x66:
                        start_rx_state = 2
                    else:
                        start_rx_state = 0
        elif (
            DEBUG_CAMERA_START_RX
            and time.ticks_diff(
                time.ticks_ms(),
                last_empty_report_ms
            ) >= 1000
        ):
            print("UART3_RX_NONE")
            last_empty_report_ms = time.ticks_ms()

        time.sleep_ms(10)


if WAIT_MCU_82_BEFORE_CAMERA:
    wait_for_camera_start()
else:
    print("CAMERA_START_WAIT_DISABLED")

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=3000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()
target_filter = StableTarget()
state = STATE_STABILIZE
state_started_ms = pyb.millis()
rx_state = 0
frame_count = 0
aligned_reported = False


def set_state(new_state):
    global state
    global state_started_ms

    state = new_state
    state_started_ms = pyb.millis()
    target_filter.clear()


def handle_mcu_message(message_type):
    if message_type == 0x33:
        print("MCU_READY: 66 66 33")
    elif message_type == 0x82:
        if state == STATE_WAIT_DONE:
            print("MCU_MOVE_DONE: 66 66 82")
            set_state(STATE_SETTLE)
        else:
            print("STALE_0x82_IGNORED")
    elif message_type == 0x83:
        print("MCU_SEND_DONE: 66 66 83")
    elif message_type == 0x84:
        print("MCU_CAMERA_ROTATE_DONE: 66 66 84")
    elif message_type == 0x85:
        print("MCU_GRAB_DONE: 66 66 85")


def poll_uart():
    global rx_state

    while uart.any():
        value = uart.readchar()
        if value < 0:
            return

        if rx_state == 0:
            if value == 0x66:
                rx_state = 1
        elif rx_state == 1:
            if value == 0x66:
                rx_state = 2
            else:
                rx_state = 0
        else:
            # For 66 66 66 82, the final two 66 bytes can be the new header.
            if value == 0x66:
                rx_state = 2
            else:
                rx_state = 0
                handle_mcu_message(value)


def choose_target(orange, purple):
    if TARGET_COLOR == "ORANGE":
        return ("O", orange) if orange is not None else (None, None)

    if TARGET_COLOR == "PURPLE":
        return ("P", purple) if purple is not None else (None, None)

    # ANY mode deliberately refuses to choose when both colours are present.
    if orange is not None and purple is None:
        return ("O", orange)
    if purple is not None and orange is None:
        return ("P", purple)
    return (None, None)


def send_one_move(move):
    move_direction_deg = move[2]
    move_distance_cm = move[3]
    frame, encoded_angle, encoded_distance = build_move_frame(
        move_direction_deg, move_distance_cm
    )

    # Enter WAIT_DONE before writing so an unusually fast reply cannot be
    # mistaken for a stale completion message.
    set_state(STATE_WAIT_DONE)
    written = uart.write(frame)

    if written is not None and written != len(frame):
        print("UART_WRITE_FAILED: %s/6" % str(written))
        set_state(STATE_FAULT)
        return

    print_frame(frame)
    print(
        "COMMAND: direction_ccw=%d deg, distance=%d cm"
        % (encoded_angle, encoded_distance)
    )


def search_right_if_timed_out():
    if not ENABLE_NO_TARGET_SEARCH:
        return False

    if state != STATE_STABILIZE:
        return False

    if pyb.elapsed_millis(state_started_ms) < NO_TARGET_SEARCH_TIMEOUT_MS:
        return False

    print(
        "NO_STABLE_TARGET_%dMS: search right %d cm "
        "using protocol-left angle %d deg"
        % (
            NO_TARGET_SEARCH_TIMEOUT_MS,
            NO_TARGET_SEARCH_DISTANCE_CM,
            NO_TARGET_SEARCH_COMMAND_ANGLE_DEG,
        )
    )

    send_one_move(
        (
            0.0,
            0.0,
            float(NO_TARGET_SEARCH_COMMAND_ANGLE_DEG),
            float(NO_TARGET_SEARCH_DISTANCE_CM),
        )
    )
    return True


print("TARGET: forward=38 cm, left=12 cm; translation only")
print("Waiting for one stable target...")


while True:
    clock.tick()
    poll_uart()

    if state == STATE_WAIT_DONE:
        if pyb.elapsed_millis(state_started_ms) > WAIT_DONE_TIMEOUT_MS:
            print("FAULT: no 66 66 82 received; command will NOT be resent")
            set_state(STATE_FAULT)
    elif state == STATE_SETTLE:
        if pyb.elapsed_millis(state_started_ms) >= SETTLE_AFTER_DONE_MS:
            print("Camera settled; measuring for correction")
            set_state(STATE_STABILIZE)

    img = sensor.snapshot()

    # Complete both searches before drawing anything onto the framebuffer.
    orange = find_largest(
        img.find_blobs(
            [ORANGE_THRESHOLD],
            pixels_threshold=200,
            area_threshold=200,
            merge=True,
        )
    )
    purple = find_largest(
        img.find_blobs(
            [PURPLE_THRESHOLD],
            pixels_threshold=200,
            area_threshold=200,
            merge=True,
        )
    )

    if orange is not None:
        img.draw_rectangle(orange.rect(), color=(255, 128, 0), thickness=2)
        img.draw_cross(orange.cx(), orange.cy(), color=(255, 128, 0))

    if purple is not None:
        img.draw_rectangle(purple.rect(), color=(255, 0, 255), thickness=2)
        img.draw_cross(purple.cx(), purple.cy(), color=(255, 0, 255))

    frame_count += 1
    should_print = frame_count >= PRINT_EVERY_N_FRAMES
    if should_print:
        frame_count = 0

    if state != STATE_STABILIZE:
        if should_print and state == STATE_WAIT_DONE:
            print("WAITING_FOR_66_66_82")
        continue

    label, selected = choose_target(orange, purple)

    if TARGET_COLOR == "ANY" and orange is not None and purple is not None:
        target_filter.clear()
        if should_print:
            print("BOTH_TARGETS_NO_COMMAND")
        search_right_if_timed_out()
        continue

    if selected is None:
        target_filter.clear()
        aligned_reported = False
        if should_print:
            print("NO_TARGET")
        search_right_if_timed_out()
        continue

    if not blob_is_complete(selected):
        target_filter.clear()
        if should_print:
            print("TARGET_TOUCHES_EDGE_OR_BAD_SHAPE_NO_COMMAND")
        search_right_if_timed_out()
        continue

    target_filter.add(label, selected)
    stable = target_filter.result()
    if stable is None:
        if should_print:
            print("STABILIZING: %d/%d" % (len(target_filter.samples), FILTER_WINDOW))
        search_right_if_timed_out()
        continue

    stable_label = stable[0]
    cx = stable[1]
    cy = stable[2]
    width = stable[3]
    height = stable[4]

    pose = estimate_target_pose(cx, width)
    if pose is None:
        target_filter.clear()
        search_right_if_timed_out()
        continue

    forward_cm = pose[0]
    right_cm = pose[1]
    move = calculate_chassis_move(forward_cm, right_cm)
    move_forward_cm = move[0]
    move_right_cm = move[1]
    move_direction_deg = move[2]
    move_distance_cm = move[3]

    if should_print:
        print(
            "%s,cx=%d,cy=%d,w=%d,h=%d,Z=%.2fcm,X_right=%.2fcm,"
            "move_fwd=%.2fcm,move_right=%.2fcm,dir_ccw=%.1f,dist=%.2fcm"
            % (
                stable_label,
                cx,
                cy,
                width,
                height,
                forward_cm,
                right_cm,
                move_forward_cm,
                move_right_cm,
                move_direction_deg,
                move_distance_cm,
            )
        )

    if (
        forward_cm < MIN_VALID_FORWARD_CM
        or forward_cm > MAX_VALID_FORWARD_CM
    ):
        if should_print:
            print("OUTSIDE_30_TO_70_CM_CALIBRATION_NO_COMMAND")
        search_right_if_timed_out()
        continue

    if (
        abs(move_forward_cm) <= FORWARD_TOLERANCE_CM
        and abs(move_right_cm) <= RIGHT_TOLERANCE_CM
    ):
        if not aligned_reported:
            print("TARGET_AT_DESIRED_POSITION")
            aligned_reported = True
        continue

    aligned_reported = False
    send_one_move(move)
