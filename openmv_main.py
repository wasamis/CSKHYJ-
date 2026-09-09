import sensor
import time
import math
import pyb
from pyb import Servo, UART


# ============================================================
# Mineral and camera configuration
# ============================================================

# LAB thresholds for normal indoor lighting.
PURPLE_THRESHOLD = (25, 85, 4, 32, -45, -8)
ORANGE_THRESHOLD = (35, 100, -12, 28, 10, 50)

ORANGE_REQUIRED = 2
PURPLE_REQUIRED = 1

# True: skip the orange mission and test only purple recognition/pickup.
# The complete 0x00 mission described below requires this to be False.
PURPLE_ONLY_TEST = False

# Desired block position at the instant the 66 66 05 grab command is sent.
# Coordinates are relative to the camera:
#   forward > 0: block is in front of the camera
#   right   > 0: block is to the camera's right
#   right   < 0: block is to the camera's left
#
# Increase FORWARD to make the chassis stop farther from the block; decrease
# it to make the chassis move closer.  Adjust orange and purple separately
# because the camera faces forward for orange and left for purple.
ORANGE_GRAB_TARGET_FORWARD_CM = 35.0
ORANGE_GRAB_TARGET_RIGHT_CM = -11.0

PURPLE_GRAB_TARGET_FORWARD_CM = 38.0
PURPLE_GRAB_TARGET_RIGHT_CM = -12.0

# Distance calibration:
# (width_px, distance_cm) = (120,30), (92,40), (74,50), (63,60), (54,70)
DISTANCE_A = 3940.04622
DISTANCE_B = -2.881682

# Horizontal calibration at approximately 50 cm.
CX_ZERO = 157.714286
HORIZONTAL_FOCAL_PX = 340.917404

# The measured calibration samples cover 30-70 cm.  A lower limit of 10 cm
# lets a fully visible nearby block enter the positioning state.  Distances
# below 30 cm use the same inverse-width curve by extrapolation.
MIN_VALID_FORWARD_CM = 10.0
MAX_VALID_FORWARD_CM = 75.0

FILTER_WINDOW = 3
MAX_CX_RANGE_PX = 6
MAX_CY_RANGE_PX = 6
MAX_W_RANGE_PX = 5
MAX_H_RANGE_PX = 6

EDGE_MARGIN_PX = 2
MIN_VISIBLE_ASPECT_RATIO = 0.60
MAX_VISIBLE_ASPECT_RATIO = 3.00

FORWARD_TOLERANCE_CM = 2.0
RIGHT_TOLERANCE_CM = 2.0

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
PRINT_EVERY_N_FRAMES = 5


# ============================================================
# Mission movement configuration
# ============================================================

# These angles follow the current real-vehicle behavior specified for this
# mission: 0=forward, 90=right, 180=backward, 270=left.
ANGLE_FORWARD_DEG = 0
ANGLE_RIGHT_DEG = 90
ANGLE_BACKWARD_DEG = 180
ANGLE_LEFT_DEG = 270

SEARCH_STEP_CM = 10
ORANGE_TO_PURPLE_LEFT_CM = 100
PURPLE_WALL_RIGHT_OFFSET_CM = 10
PURPLE_ENTRY_CLOCKWISE_DEG = 90
PURPLE_ALIGN_FORWARD_CM = 20
PURPLE_ALIGN_BACKWARD_CM = 10
PURPLE_FINAL_BACKWARD_CM = 72
PURPLE_FINAL_CLOCKWISE_DEG = 90

NO_TARGET_TIMEOUT_MS = 500
MOVE_SETTLE_MS = 500
GIMBAL_SETTLE_MS = 1000

WAIT_MOVE_TIMEOUT_MS = 30000
WAIT_GRAB_TIMEOUT_MS = 60000
WAIT_ROUTE_TIMEOUT_MS = 40000
WAIT_BUILD_TIMEOUT_MS = 60000


# ============================================================
# UART protocol
# ============================================================

UART_BAUDRATE = 115200

CMD_CHASSIS_MOVE = 0x02
CMD_CHASSIS_ROTATE = 0x03
CMD_ARM_GRAB = 0x05
CMD_LINE_TRACE = 0x06
CMD_ARM_BUILD = 0x08

MCU_CHASSIS_DONE = 0x82
MCU_ARM_GRAB_DONE = 0x83
MCU_BUILD_AREA_ARRIVED = 0x84
MCU_ARM_BUILD_DONE = 0x85

LINE_TRACE_TO_BUILD = 0x01

# In purple-only test mode the chassis is assumed to have been placed near
# the purple area manually, so camera processing starts without waiting for
# the initial 0x00 route completion frame.
WAIT_MCU_82_BEFORE_CAMERA = not PURPLE_ONLY_TEST


# ============================================================
# Gimbal configuration
# ============================================================

# Horizontal servo (P7): 270-degree model, 500-2500 us.
PAN_SERVO_MIN_US = 500
PAN_SERVO_MAX_US = 2500
PAN_SERVO_MAX_ANGLE_DEG = 270

# Vertical servo (P8): 270-degree model, 500-2500 us.
TILT_SERVO_MIN_US = 500
TILT_SERVO_MAX_US = 2500
TILT_SERVO_MAX_ANGLE_DEG = 270

# Gimbal position applied immediately when this script starts.
# Pan  : Servo(1) -> P7
# Tilt : Servo(2) -> P8
PAN_INITIAL_ANGLE_DEG = 42

TILT_INITIAL_ANGLE_DEG = 147

PAN_FORWARD_ANGLE_DEG = 135

# Change 180 to 0 if the installed servo turns in the opposite direction.
PAN_LEFT_ANGLE_DEG = 180

# Gimbal pose used while travelling to and locating the build-area marker.
PAN_BUILD_QR_ANGLE_DEG = 225
TILT_BUILD_QR_ANGLE_DEG = 135


# ============================================================
# Build marker localization configuration
# ============================================================

# Run openmv_qr_target_calibrate.py once at the exact build pose. It records
# only the white frame centre and apparent size in this file on the OpenMV.
BUILD_QR_REFERENCE_FILE = "build_qr_target.txt"

# These values must match the standalone calibration program. The internal
# 8x8 pattern is deliberately ignored; only the continuous white frame is
# used for localization.
BUILD_MARKER_WHITE_THRESHOLD = (220, 255)
BUILD_MARKER_MIN_BLOB_PIXELS = 250
BUILD_MARKER_MIN_BLOB_AREA = 1600
BUILD_MARKER_MIN_SIZE_PX = 40
BUILD_MARKER_MAX_SIZE_PX = 220
BUILD_MARKER_MAX_SQUARE_ERROR_PERCENT = 22
BUILD_MARKER_MIN_FRAME_DENSITY_PERCENT = 7
BUILD_MARKER_MAX_FRAME_DENSITY_PERCENT = 70
BUILD_MARKER_EDGE_MARGIN_PX = 4

# Physical camera-to-QR distance at the calibrated target pose.  The
# reference QR width supplies the relative scale for subsequent corrections.
BUILD_QR_TARGET_FORWARD_CM = 38.0

BUILD_QR_FORWARD_TOLERANCE_CM = 2.0
BUILD_QR_RIGHT_TOLERANCE_CM = 2.0
BUILD_QR_MAX_CORRECTION_CM = 30.0
BUILD_QR_MIN_WIDTH_PX = 12


# ============================================================
# Mission states
# ============================================================

STATE_MINERAL_STABILIZE = 0
STATE_WAIT_CHASSIS_82 = 1
STATE_SETTLE = 2
STATE_WAIT_GRAB_83 = 3
STATE_WAIT_ROUTE_01_84 = 4
STATE_BUILD_QR = 5
STATE_WAIT_BUILD_85 = 6
STATE_COMPLETE = 7
STATE_FAULT = 8
STATE_SEND_ROUTE_01 = 9

COLOR_ORANGE = 0
COLOR_PURPLE = 1

AFTER_MOVE_NONE = 0
AFTER_MOVE_RECHECK_ORANGE = 1
AFTER_MOVE_RECHECK_PURPLE = 2
AFTER_MOVE_START_PURPLE = 3
AFTER_MOVE_PURPLE_FINAL_BACKWARD_DONE = 4
AFTER_MOVE_PURPLE_FINAL_TURN_DONE = 5
AFTER_MOVE_RECHECK_BUILD_QR = 6
AFTER_MOVE_PURPLE_WALL_REACHED = 7
AFTER_MOVE_PURPLE_RIGHT_OFFSET_DONE = 8
AFTER_MOVE_PURPLE_TURN_DONE = 9
AFTER_MOVE_PURPLE_ALIGN_FORWARD_DONE = 10


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


def build_marker_candidate_is_valid(blob):
    width = blob.w()
    height = blob.h()

    if (
        width < BUILD_MARKER_MIN_SIZE_PX
        or height < BUILD_MARKER_MIN_SIZE_PX
        or width > BUILD_MARKER_MAX_SIZE_PX
        or height > BUILD_MARKER_MAX_SIZE_PX
    ):
        return False

    if (
        blob.x() <= BUILD_MARKER_EDGE_MARGIN_PX
        or blob.y() <= BUILD_MARKER_EDGE_MARGIN_PX
        or (blob.x() + width)
        >= (FRAME_WIDTH - BUILD_MARKER_EDGE_MARGIN_PX)
        or (blob.y() + height)
        >= (FRAME_HEIGHT - BUILD_MARKER_EDGE_MARGIN_PX)
    ):
        return False

    longer_side = max(width, height)
    square_error_percent = abs(width - height) * 100 // longer_side

    if square_error_percent > BUILD_MARKER_MAX_SQUARE_ERROR_PERCENT:
        return False

    density_percent = blob.pixels() * 100 // (width * height)

    return (
        density_percent >= BUILD_MARKER_MIN_FRAME_DENSITY_PERCENT
        and density_percent <= BUILD_MARKER_MAX_FRAME_DENSITY_PERCENT
    )


def find_largest_build_marker(img):
    blobs = img.find_blobs(
        [BUILD_MARKER_WHITE_THRESHOLD],
        pixels_threshold=BUILD_MARKER_MIN_BLOB_PIXELS,
        area_threshold=BUILD_MARKER_MIN_BLOB_AREA,
        merge=False
    )
    selected = None

    for blob in blobs:
        if not build_marker_candidate_is_valid(blob):
            continue

        if selected is None:
            selected = blob
        elif (blob.w() * blob.h()) > (selected.w() * selected.h()):
            selected = blob

    return selected


def load_build_qr_reference():
    try:
        with open(BUILD_QR_REFERENCE_FILE, "r") as reference_file:
            fields = reference_file.readline().strip().split(",")

        if len(fields) != 4:
            raise ValueError("expected cx,cy,w,h")

        reference = (
            int(fields[0]),
            int(fields[1]),
            int(fields[2]),
            int(fields[3]),
        )

        if reference[2] < BUILD_QR_MIN_WIDTH_PX or reference[3] <= 0:
            raise ValueError("invalid marker size")

        print(
            "BUILD_MARKER_REFERENCE_LOADED: cx=%d,cy=%d,w=%d,h=%d"
            % reference
        )
        return reference
    except Exception as error:
        print(
            "BUILD_MARKER_REFERENCE_MISSING: run calibration first (%s)"
            % str(error)
        )
        return None


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

        self.samples.append(
            (blob.cx(), blob.cy(), blob.w(), blob.h())
        )

        if len(self.samples) > FILTER_WINDOW:
            self.samples.pop(0)

    def result(self):
        if len(self.samples) < FILTER_WINDOW:
            return None

        cxs = [sample[0] for sample in self.samples]
        cys = [sample[1] for sample in self.samples]
        widths = [sample[2] for sample in self.samples]
        heights = [sample[3] for sample in self.samples]

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

    camera_forward_cm = DISTANCE_A / width_px + DISTANCE_B
    camera_right_cm = (
        (cx - CX_ZERO)
        * camera_forward_cm
        / HORIZONTAL_FOCAL_PX
    )

    return (camera_forward_cm, camera_right_cm)


def calculate_chassis_move(
    camera_forward_cm,
    camera_right_cm,
    target_forward_cm,
    target_right_cm
):
    move_camera_forward_cm = (
        camera_forward_cm - target_forward_cm
    )
    move_camera_right_cm = (
        camera_right_cm - target_right_cm
    )

    # The gimbal keeps the camera facing the same direction as the chassis:
    #   camera-forward -> chassis-forward
    #   camera-right   -> chassis-right
    # Therefore a target on the image's left also maps to chassis-left.
    move_chassis_forward_cm = move_camera_forward_cm
    move_chassis_right_cm = move_camera_right_cm

    distance_cm = math.sqrt(
        move_chassis_forward_cm * move_chassis_forward_cm
        + move_chassis_right_cm * move_chassis_right_cm
    )

    # Current mission convention:
    # 0=forward, 90=right, 180=backward, 270=left.
    direction_deg = (
        math.atan2(
            move_chassis_right_cm,
            move_chassis_forward_cm
        )
        * 57.2957795
    )

    while direction_deg < 0:
        direction_deg += 360.0
    while direction_deg >= 360.0:
        direction_deg -= 360.0

    return (
        move_camera_forward_cm,
        move_camera_right_cm,
        move_chassis_forward_cm,
        move_chassis_right_cm,
        direction_deg,
        distance_cm,
    )


def build_move_frame(direction_deg, distance_cm):
    angle = round_int(direction_deg) % 360
    distance = clamp(round_int(distance_cm), 0, 255)

    frame = bytearray(
        (
            0x66,
            0x66,
            CMD_CHASSIS_MOVE,
            (angle >> 8) & 0xFF,
            angle & 0xFF,
            distance,
        )
    )

    return (frame, angle, distance)


def build_rotate_frame(clockwise_deg):
    clockwise = clamp(round_int(clockwise_deg), -180, 180)
    encoded = clockwise + 180

    frame = bytearray(
        (
            0x66,
            0x66,
            CMD_CHASSIS_ROTATE,
            (encoded >> 8) & 0xFF,
            encoded & 0xFF,
        )
    )

    return (frame, clockwise)


def angle_to_pulse_us(
    angle_deg,
    min_pulse_us,
    max_pulse_us,
    max_angle_deg
):
    angle_deg = clamp(angle_deg, 0, max_angle_deg)
    pulse_us = (
        min_pulse_us
        + (max_pulse_us - min_pulse_us)
        * angle_deg
        // max_angle_deg
    )
    return pulse_us


uart = UART(
    3,
    UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    timeout=100,
    timeout_char=10
)

# On STM32-based OpenMV boards Servo(1) is P7 and Servo(2) is P8.
# This API is supported by the installed firmware, which does not provide
# machine.PWM.
servo_pan = Servo(1)
servo_tilt = Servo(2)


def set_pan(angle_deg):
    servo_pan.pulse_width(
        angle_to_pulse_us(
            angle_deg,
            PAN_SERVO_MIN_US,
            PAN_SERVO_MAX_US,
            PAN_SERVO_MAX_ANGLE_DEG
        )
    )
    print("PAN_ANGLE: %d" % angle_deg)


def set_tilt(angle_deg):
    servo_tilt.pulse_width(
        angle_to_pulse_us(
            angle_deg,
            TILT_SERVO_MIN_US,
            TILT_SERVO_MAX_US,
            TILT_SERVO_MAX_ANGLE_DEG
        )
    )
    print("TILT_ANGLE: %d" % angle_deg)


def set_gimbal(pan_angle_deg, tilt_angle_deg):
    set_pan(pan_angle_deg)
    set_tilt(tilt_angle_deg)


def prepare_build_marker_camera():
    # Mineral detection no longer runs after route 0x01, so the camera can be
    # switched from RGB565 to grayscale for the white-frame detector without
    # changing the calibrated orange and purple stages.
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
    sensor.skip_frames(time=1000)
    print("BUILD_MARKER_CAMERA_READY: QVGA grayscale")


def wait_for_initial_route():
    rx_state_local = 0

    print("WAITING_INITIAL_ROUTE_DONE: 66 66 82")

    while True:
        while uart.any():
            value = uart.readchar()

            if value < 0:
                break

            if rx_state_local == 0:
                if value == 0x66:
                    rx_state_local = 1
            elif rx_state_local == 1:
                if value == 0x66:
                    rx_state_local = 2
                else:
                    rx_state_local = 0
            else:
                if value == MCU_CHASSIS_DONE:
                    print("INITIAL_ROUTE_DONE: 66 66 82")
                    return

                if value == 0x66:
                    rx_state_local = 2
                else:
                    rx_state_local = 0

        time.sleep_ms(10)


set_gimbal(
    PAN_INITIAL_ANGLE_DEG,
    TILT_INITIAL_ANGLE_DEG
)


if WAIT_MCU_82_BEFORE_CAMERA:
    wait_for_initial_route()
else:
    print("INITIAL_ROUTE_WAIT_DISABLED")


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=3000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

set_pan(PAN_FORWARD_ANGLE_DEG)

clock = time.clock()
target_filter = StableTarget()
build_qr_reference = load_build_qr_reference()

state = STATE_MINERAL_STABILIZE
state_started_ms = pyb.millis()
rx_state = 0

if PURPLE_ONLY_TEST:
    current_color = COLOR_PURPLE
else:
    current_color = COLOR_ORANGE
orange_collected = 0
purple_collected = 0

after_move_action = AFTER_MOVE_NONE
settle_next_state = STATE_MINERAL_STABILIZE
settle_duration_ms = MOVE_SETTLE_MS

frame_count = 0


def color_name(color):
    if color == COLOR_ORANGE:
        return "ORANGE"
    return "PURPLE"


def set_state(new_state):
    global state
    global state_started_ms

    state = new_state
    state_started_ms = pyb.millis()
    target_filter.clear()


def begin_settle(next_state, duration_ms):
    global settle_next_state
    global settle_duration_ms

    settle_next_state = next_state
    settle_duration_ms = duration_ms
    set_state(STATE_SETTLE)


def send_uart_frame(frame):
    written = uart.write(frame)

    if written is not None and written != len(frame):
        print(
            "UART_WRITE_FAILED: %s/%d"
            % (str(written), len(frame))
        )
        set_state(STATE_FAULT)
        return False

    text = []

    for value in frame:
        text.append("%02X" % value)

    print("UART_TX: %s" % " ".join(text))
    return True


def send_move(direction_deg, distance_cm, next_action):
    global after_move_action

    frame, encoded_angle, encoded_distance = build_move_frame(
        direction_deg,
        distance_cm
    )

    after_move_action = next_action
    set_state(STATE_WAIT_CHASSIS_82)

    if not send_uart_frame(frame):
        return

    print(
        "MOVE_COMMAND: angle=%d deg, distance=%d cm"
        % (encoded_angle, encoded_distance)
    )


def send_rotate_clockwise(clockwise_deg, next_action):
    global after_move_action

    frame, encoded_clockwise = build_rotate_frame(
        clockwise_deg
    )

    after_move_action = next_action
    set_state(STATE_WAIT_CHASSIS_82)

    if not send_uart_frame(frame):
        return

    print(
        "ROTATE_COMMAND: clockwise=%d deg"
        % encoded_clockwise
    )


def send_grab_command():
    set_state(STATE_WAIT_GRAB_83)
    send_uart_frame(
        bytearray((0x66, 0x66, CMD_ARM_GRAB))
    )


def send_route_01_command():
    set_state(STATE_WAIT_ROUTE_01_84)
    send_uart_frame(
        bytearray(
            (0x66, 0x66, CMD_LINE_TRACE, LINE_TRACE_TO_BUILD)
        )
    )


def send_build_command():
    set_state(STATE_WAIT_BUILD_85)
    send_uart_frame(
        bytearray((0x66, 0x66, CMD_ARM_BUILD))
    )


def dispatch_after_move():
    global current_color

    if after_move_action == AFTER_MOVE_RECHECK_ORANGE:
        current_color = COLOR_ORANGE
        begin_settle(
            STATE_MINERAL_STABILIZE,
            MOVE_SETTLE_MS
        )
    elif after_move_action == AFTER_MOVE_RECHECK_PURPLE:
        current_color = COLOR_PURPLE
        begin_settle(
            STATE_MINERAL_STABILIZE,
            MOVE_SETTLE_MS
        )
    elif after_move_action == AFTER_MOVE_START_PURPLE:
        current_color = COLOR_PURPLE
        begin_settle(
            STATE_MINERAL_STABILIZE,
            GIMBAL_SETTLE_MS
        )
    elif after_move_action == AFTER_MOVE_PURPLE_WALL_REACHED:
        send_move(
            ANGLE_RIGHT_DEG,
            PURPLE_WALL_RIGHT_OFFSET_CM,
            AFTER_MOVE_PURPLE_RIGHT_OFFSET_DONE
        )
    elif after_move_action == AFTER_MOVE_PURPLE_RIGHT_OFFSET_DONE:
        send_rotate_clockwise(
            PURPLE_ENTRY_CLOCKWISE_DEG,
            AFTER_MOVE_PURPLE_TURN_DONE
        )
    elif after_move_action == AFTER_MOVE_PURPLE_TURN_DONE:
        send_move(
            ANGLE_FORWARD_DEG,
            PURPLE_ALIGN_FORWARD_CM,
            AFTER_MOVE_PURPLE_ALIGN_FORWARD_DONE
        )
    elif after_move_action == AFTER_MOVE_PURPLE_ALIGN_FORWARD_DONE:
        send_move(
            ANGLE_BACKWARD_DEG,
            PURPLE_ALIGN_BACKWARD_CM,
            AFTER_MOVE_START_PURPLE
        )
    elif after_move_action == AFTER_MOVE_PURPLE_FINAL_BACKWARD_DONE:
        send_rotate_clockwise(
            PURPLE_FINAL_CLOCKWISE_DEG,
            AFTER_MOVE_PURPLE_FINAL_TURN_DONE
        )
    elif after_move_action == AFTER_MOVE_PURPLE_FINAL_TURN_DONE:
        set_gimbal(
            PAN_BUILD_QR_ANGLE_DEG,
            TILT_BUILD_QR_ANGLE_DEG
        )
        begin_settle(
            STATE_SEND_ROUTE_01,
            GIMBAL_SETTLE_MS
        )
    elif after_move_action == AFTER_MOVE_RECHECK_BUILD_QR:
        begin_settle(
            STATE_BUILD_QR,
            MOVE_SETTLE_MS
        )
    else:
        print("FAULT: invalid action after chassis move")
        set_state(STATE_FAULT)


def handle_grab_done():
    global orange_collected
    global purple_collected

    if current_color == COLOR_ORANGE:
        orange_collected += 1
        print(
            "ORANGE_GRABBED: %d/%d"
            % (orange_collected, ORANGE_REQUIRED)
        )

        if orange_collected < ORANGE_REQUIRED:
            # Move left once immediately, then continue the same 500 ms
            # search policy until the second orange mineral is found.
            send_move(
                ANGLE_LEFT_DEG,
                SEARCH_STEP_CM,
                AFTER_MOVE_RECHECK_ORANGE
            )
        else:
            send_move(
                ANGLE_LEFT_DEG,
                ORANGE_TO_PURPLE_LEFT_CM,
                AFTER_MOVE_PURPLE_WALL_REACHED
            )
    else:
        purple_collected += 1
        print(
            "PURPLE_GRABBED: %d/%d"
            % (purple_collected, PURPLE_REQUIRED)
        )

        if PURPLE_ONLY_TEST:
            print("PURPLE_ONLY_TEST_COMPLETE")
            set_state(STATE_COMPLETE)
            return

        if purple_collected < PURPLE_REQUIRED:
            send_move(
                ANGLE_LEFT_DEG,
                SEARCH_STEP_CM,
                AFTER_MOVE_RECHECK_PURPLE
            )
        else:
            send_move(
                ANGLE_BACKWARD_DEG,
                PURPLE_FINAL_BACKWARD_CM,
                AFTER_MOVE_PURPLE_FINAL_BACKWARD_DONE
            )


def handle_mcu_message(message_type):
    if message_type == MCU_CHASSIS_DONE:
        if state == STATE_WAIT_CHASSIS_82:
            print("MCU_CHASSIS_DONE: 66 66 82")
            dispatch_after_move()
        else:
            print("STALE_0x82_IGNORED")
    elif message_type == MCU_ARM_GRAB_DONE:
        if state == STATE_WAIT_GRAB_83:
            print("MCU_GRAB_DONE: 66 66 83")
            handle_grab_done()
        else:
            print("STALE_0x83_IGNORED")
    elif message_type == MCU_BUILD_AREA_ARRIVED:
        if state == STATE_WAIT_ROUTE_01_84:
            print("BUILD_AREA_ARRIVED: 66 66 84")
            prepare_build_marker_camera()
            begin_settle(
                STATE_BUILD_QR,
                GIMBAL_SETTLE_MS
            )
        else:
            print("STALE_0x84_IGNORED")
    elif message_type == MCU_ARM_BUILD_DONE:
        if state == STATE_WAIT_BUILD_85:
            print("BUILD_DONE: 66 66 85")
            set_state(STATE_COMPLETE)
        else:
            print("STALE_0x85_IGNORED")
    elif message_type == 0x33:
        print("MCU_READY: 66 66 33")
    else:
        print("MCU_UNKNOWN_FRAME: 66 66 %02X" % message_type)


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
            if value == 0x66:
                rx_state = 2
            else:
                rx_state = 0
                handle_mcu_message(value)


def search_if_timed_out(
    direction_deg,
    next_action,
    direction_name
):
    if state != STATE_MINERAL_STABILIZE:
        return False

    if pyb.elapsed_millis(state_started_ms) < NO_TARGET_TIMEOUT_MS:
        return False

    print(
        "%s_NO_3_STABLE_FRAMES_%dMS: search %s %d cm"
        % (
            color_name(current_color),
            NO_TARGET_TIMEOUT_MS,
            direction_name,
            SEARCH_STEP_CM,
        )
    )

    send_move(
        direction_deg,
        SEARCH_STEP_CM,
        next_action
    )
    return True


def process_mineral_target(
    label,
    selected,
    target_forward_cm,
    target_right_cm,
    search_direction_deg,
    search_next_action,
    search_direction_name,
    should_print
):
    if selected is None:
        target_filter.clear()

        if should_print:
            print("%s_NO_TARGET" % label)

        search_if_timed_out(
            search_direction_deg,
            search_next_action,
            search_direction_name
        )
        return

    if not blob_is_complete(selected):
        target_filter.clear()

        if should_print:
            print("%s_TARGET_INCOMPLETE" % label)

        search_if_timed_out(
            search_direction_deg,
            search_next_action,
            search_direction_name
        )
        return

    target_filter.add(label, selected)
    stable = target_filter.result()

    if stable is None:
        if should_print:
            print(
                "%s_STABILIZING: %d/%d"
                % (
                    label,
                    len(target_filter.samples),
                    FILTER_WINDOW,
                )
            )

        search_if_timed_out(
            search_direction_deg,
            search_next_action,
            search_direction_name
        )
        return

    cx = stable[1]
    cy = stable[2]
    width = stable[3]
    height = stable[4]

    pose = estimate_target_pose(cx, width)

    if pose is None:
        target_filter.clear()
        return

    camera_forward_cm = pose[0]
    camera_right_cm = pose[1]

    if (
        camera_forward_cm < MIN_VALID_FORWARD_CM
        or camera_forward_cm > MAX_VALID_FORWARD_CM
    ):
        if should_print:
            print("%s_OUTSIDE_DISTANCE_CALIBRATION" % label)

        search_if_timed_out(
            search_direction_deg,
            search_next_action,
            search_direction_name
        )
        return

    move = calculate_chassis_move(
        camera_forward_cm,
        camera_right_cm,
        target_forward_cm,
        target_right_cm
    )

    move_camera_forward_cm = move[0]
    move_camera_right_cm = move[1]
    move_direction_deg = move[4]
    move_distance_cm = move[5]

    if should_print:
        print(
            "%s,cx=%d,cy=%d,w=%d,h=%d,Z=%.2fcm,X=%.2fcm,"
            "target_Z=%.2fcm,target_X=%.2fcm,"
            "camera_df=%.2fcm,camera_dr=%.2fcm,"
            "chassis_dir=%.1fdeg,chassis_dist=%.2fcm"
            % (
                label,
                cx,
                cy,
                width,
                height,
                camera_forward_cm,
                camera_right_cm,
                target_forward_cm,
                target_right_cm,
                move_camera_forward_cm,
                move_camera_right_cm,
                move_direction_deg,
                move_distance_cm,
            )
        )

    if (
        abs(move_camera_forward_cm) <= FORWARD_TOLERANCE_CM
        and abs(move_camera_right_cm) <= RIGHT_TOLERANCE_CM
    ):
        print("%s_TARGET_ALIGNED: send grab" % label)
        send_grab_command()
        return

    send_move(
        move_direction_deg,
        move_distance_cm,
        search_next_action
    )


def decide_orange_mining(orange_blob, should_print):
    if orange_collected >= ORANGE_REQUIRED:
        return

    process_mineral_target(
        "ORANGE",
        orange_blob,
        ORANGE_GRAB_TARGET_FORWARD_CM,
        ORANGE_GRAB_TARGET_RIGHT_CM,
        ANGLE_LEFT_DEG,
        AFTER_MOVE_RECHECK_ORANGE,
        "left",
        should_print
    )


def decide_purple_mining(purple_blob, should_print):
    if purple_collected >= PURPLE_REQUIRED:
        return

    process_mineral_target(
        "PURPLE",
        purple_blob,
        PURPLE_GRAB_TARGET_FORWARD_CM,
        PURPLE_GRAB_TARGET_RIGHT_CM,
        ANGLE_LEFT_DEG,
        AFTER_MOVE_RECHECK_PURPLE,
        "left",
        should_print
    )


def process_build_marker(img, should_print):
    if build_qr_reference is None:
        if should_print:
            print(
                "BUILD_MARKER_DISABLED: missing %s"
                % BUILD_QR_REFERENCE_FILE
            )
        return

    marker = find_largest_build_marker(img)

    if marker is None:
        target_filter.clear()

        if should_print:
            print("BUILD_MARKER_NOT_FOUND")
        return

    img.draw_rectangle(
        marker.rect(),
        color=255,
        thickness=2
    )
    img.draw_cross(
        marker.cx(),
        marker.cy(),
        color=127
    )

    if not blob_is_complete(marker):
        target_filter.clear()

        if should_print:
            print("BUILD_MARKER_INCOMPLETE")
        return

    target_filter.add("BUILD_MARKER", marker)
    stable = target_filter.result()

    if stable is None:
        if should_print:
            print(
                "BUILD_MARKER_STABILIZING: %d/%d"
                % (len(target_filter.samples), FILTER_WINDOW)
            )
        return

    cx = stable[1]
    cy = stable[2]
    width = stable[3]
    height = stable[4]

    if width < BUILD_QR_MIN_WIDTH_PX:
        target_filter.clear()

        if should_print:
            print("BUILD_MARKER_TOO_SMALL")
        return

    target_cx = build_qr_reference[0]
    target_cy = build_qr_reference[1]
    target_width = build_qr_reference[2]
    target_height = build_qr_reference[3]

    current_forward_cm = (
        BUILD_QR_TARGET_FORWARD_CM
        * target_width
        / width
    )
    move_camera_forward_cm = (
        current_forward_cm
        - BUILD_QR_TARGET_FORWARD_CM
    )
    move_camera_right_cm = (
        (cx - target_cx)
        * current_forward_cm
        / HORIZONTAL_FOCAL_PX
    )

    move_distance_cm = math.sqrt(
        move_camera_forward_cm * move_camera_forward_cm
        + move_camera_right_cm * move_camera_right_cm
    )

    if move_distance_cm > 0:
        move_direction_deg = (
            math.atan2(
                move_camera_right_cm,
                move_camera_forward_cm
            )
            * 57.2957795
        )
    else:
        move_direction_deg = 0.0

    while move_direction_deg < 0:
        move_direction_deg += 360.0
    while move_direction_deg >= 360.0:
        move_direction_deg -= 360.0

    if move_distance_cm > BUILD_QR_MAX_CORRECTION_CM:
        move_distance_cm = BUILD_QR_MAX_CORRECTION_CM

    if should_print:
        print(
            "BUILD_MARKER,cx=%d,cy=%d,w=%d,h=%d,"
            "target_cx=%d,target_cy=%d,target_w=%d,target_h=%d,"
            "Z=%.2fcm,camera_df=%.2fcm,camera_dr=%.2fcm,"
            "chassis_dir=%.1fdeg,chassis_dist=%.2fcm"
            % (
                cx,
                cy,
                width,
                height,
                target_cx,
                target_cy,
                target_width,
                target_height,
                current_forward_cm,
                move_camera_forward_cm,
                move_camera_right_cm,
                move_direction_deg,
                move_distance_cm,
            )
        )

    if (
        abs(move_camera_forward_cm)
        <= BUILD_QR_FORWARD_TOLERANCE_CM
        and abs(move_camera_right_cm)
        <= BUILD_QR_RIGHT_TOLERANCE_CM
    ):
        print("BUILD_MARKER_ALIGNED: send build command")
        send_build_command()
        return

    send_move(
        move_direction_deg,
        move_distance_cm,
        AFTER_MOVE_RECHECK_BUILD_QR
    )


if PURPLE_ONLY_TEST:
    print("PURPLE_ONLY_TEST: orange mission disabled")
else:
    print(
        "MISSION_0x00: collect %d orange and %d purple"
        % (ORANGE_REQUIRED, PURPLE_REQUIRED)
    )


while True:
    clock.tick()
    poll_uart()

    elapsed_ms = pyb.elapsed_millis(state_started_ms)

    if state == STATE_WAIT_CHASSIS_82:
        if elapsed_ms > WAIT_MOVE_TIMEOUT_MS:
            print("FAULT: timeout waiting for 66 66 82")
            set_state(STATE_FAULT)
    elif state == STATE_WAIT_GRAB_83:
        if elapsed_ms > WAIT_GRAB_TIMEOUT_MS:
            print("FAULT: timeout waiting for 66 66 83")
            set_state(STATE_FAULT)
    elif state == STATE_WAIT_ROUTE_01_84:
        if elapsed_ms > WAIT_ROUTE_TIMEOUT_MS:
            print("FAULT: timeout waiting for 66 66 84")
            set_state(STATE_FAULT)
    elif state == STATE_WAIT_BUILD_85:
        if elapsed_ms > WAIT_BUILD_TIMEOUT_MS:
            print("FAULT: timeout waiting for 66 66 85")
            set_state(STATE_FAULT)
    elif state == STATE_SETTLE:
        if elapsed_ms >= settle_duration_ms:
            set_state(settle_next_state)
    elif state == STATE_SEND_ROUTE_01:
        send_route_01_command()

    img = sensor.snapshot()

    frame_count += 1
    should_print = frame_count >= PRINT_EVERY_N_FRAMES

    if should_print:
        frame_count = 0

    if state == STATE_MINERAL_STABILIZE:
        if current_color == COLOR_ORANGE:
            orange = find_largest(
                img.find_blobs(
                    [ORANGE_THRESHOLD],
                    pixels_threshold=200,
                    area_threshold=200,
                    merge=True
                )
            )

            if orange is not None:
                img.draw_rectangle(
                    orange.rect(),
                    color=(255, 128, 0),
                    thickness=2
                )
                img.draw_cross(
                    orange.cx(),
                    orange.cy(),
                    color=(255, 128, 0)
                )

            decide_orange_mining(
                orange,
                should_print
            )
        else:
            purple = find_largest(
                img.find_blobs(
                    [PURPLE_THRESHOLD],
                    pixels_threshold=200,
                    area_threshold=200,
                    merge=True
                )
            )

            if purple is not None:
                img.draw_rectangle(
                    purple.rect(),
                    color=(255, 0, 255),
                    thickness=2
                )
                img.draw_cross(
                    purple.cx(),
                    purple.cy(),
                    color=(255, 0, 255)
                )

            decide_purple_mining(
                purple,
                should_print
            )
    elif state == STATE_BUILD_QR:
        process_build_marker(
            img,
            should_print
        )
    elif state == STATE_COMPLETE:
        if should_print:
            print(
                "MISSION_COMPLETE: return route is not configured"
            )
    elif state == STATE_FAULT:
        if should_print:
            print("MISSION_FAULT")
