import sensor
import time
from pyb import Servo


# Place the chassis at the exact build target pose before running this file.
# The custom marker is a continuous white square frame containing an 8x8 grid.
# Only its position and apparent size are calibrated. The grid contents are
# deliberately ignored.

REFERENCE_FILE = "build_qr_target.txt"
REPORT_INTERVAL_MS = 500

STABLE_SAMPLES = 15
MAX_CX_RANGE_PX = 4
MAX_CY_RANGE_PX = 4
MAX_W_RANGE_PX = 4
MAX_H_RANGE_PX = 4

# Grayscale detector settings. Adjust WHITE_THRESHOLD_MIN first if the white
# frame is not boxed in white. Lower it in a dim environment and raise it when
# bright background objects are merged into the frame.
WHITE_THRESHOLD_MIN = 220
WHITE_THRESHOLD_MAX = 255
MIN_BLOB_PIXELS = 250
MIN_BLOB_AREA = 1600
MIN_MARKER_SIZE_PX = 40
MAX_MARKER_SIZE_PX = 220
MAX_SQUARE_ERROR_PERCENT = 22
MIN_FRAME_DENSITY_PERCENT = 7
MAX_FRAME_DENSITY_PERCENT = 70
IMAGE_EDGE_MARGIN_PX = 4

# P7 horizontal gimbal servo: 270 degrees, 500-2500 us.
PAN_SERVO_MIN_US = 500
PAN_SERVO_MAX_US = 2500
PAN_SERVO_MAX_ANGLE_DEG = 270

# P8 vertical gimbal servo: 270 degrees, 500-2500 us.
TILT_SERVO_MIN_US = 500
TILT_SERVO_MAX_US = 2500
TILT_SERVO_MAX_ANGLE_DEG = 270

PAN_BUILD_QR_ANGLE_DEG = 225
TILT_BUILD_QR_ANGLE_DEG = 135


def angle_to_pulse_us(
    angle_deg,
    min_pulse_us,
    max_pulse_us,
    max_angle_deg
):
    if angle_deg < 0:
        angle_deg = 0
    elif angle_deg > max_angle_deg:
        angle_deg = max_angle_deg

    return (
        min_pulse_us
        + (max_pulse_us - min_pulse_us)
        * angle_deg
        // max_angle_deg
    )


def median(values):
    ordered = list(values)
    ordered.sort()
    return ordered[len(ordered) // 2]


def marker_candidate_is_valid(blob, image_width, image_height):
    x = blob.x()
    y = blob.y()
    width = blob.w()
    height = blob.h()

    if width < MIN_MARKER_SIZE_PX or height < MIN_MARKER_SIZE_PX:
        return False

    if width > MAX_MARKER_SIZE_PX or height > MAX_MARKER_SIZE_PX:
        return False

    if (
        x <= IMAGE_EDGE_MARGIN_PX
        or y <= IMAGE_EDGE_MARGIN_PX
        or (x + width) >= (image_width - IMAGE_EDGE_MARGIN_PX)
        or (y + height) >= (image_height - IMAGE_EDGE_MARGIN_PX)
    ):
        return False

    longer_side = max(width, height)
    square_error_percent = abs(width - height) * 100 // longer_side

    if square_error_percent > MAX_SQUARE_ERROR_PERCENT:
        return False

    density_percent = blob.pixels() * 100 // (width * height)

    # A white ring is neither almost empty nor a solid white square. This also
    # rejects most bright walls, paper sheets and the long white obstacles.
    if (
        density_percent < MIN_FRAME_DENSITY_PERCENT
        or density_percent > MAX_FRAME_DENSITY_PERCENT
    ):
        return False

    return True


def find_largest_grid_marker(img):
    blobs = img.find_blobs(
        [(WHITE_THRESHOLD_MIN, WHITE_THRESHOLD_MAX)],
        pixels_threshold=MIN_BLOB_PIXELS,
        area_threshold=MIN_BLOB_AREA,
        merge=False
    )
    selected = None

    for blob in blobs:
        if not marker_candidate_is_valid(blob, img.width(), img.height()):
            continue

        if selected is None:
            selected = blob
        elif (blob.w() * blob.h()) > (selected.w() * selected.h()):
            selected = blob

    return selected


servo_pan = Servo(1)
servo_tilt = Servo(2)
servo_pan.pulse_width(
    angle_to_pulse_us(
        PAN_BUILD_QR_ANGLE_DEG,
        PAN_SERVO_MIN_US,
        PAN_SERVO_MAX_US,
        PAN_SERVO_MAX_ANGLE_DEG
    )
)
servo_tilt.pulse_width(
    angle_to_pulse_us(
        TILT_BUILD_QR_ANGLE_DEG,
        TILT_SERVO_MIN_US,
        TILT_SERVO_MAX_US,
        TILT_SERVO_MAX_ANGLE_DEG
    )
)


sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.skip_frames(time=3000)

samples = []
last_report_ms = time.ticks_ms()
reference_saved = False

print("GRID_MARKER_CALIBRATION_READY: keep chassis still")


while True:
    img = sensor.snapshot()
    marker = find_largest_grid_marker(img)
    now_ms = time.ticks_ms()

    if marker is None:
        samples = []

        if time.ticks_diff(now_ms, last_report_ms) >= REPORT_INTERVAL_MS:
            print("GRID_MARKER_NONE")
            last_report_ms = now_ms

        continue

    img.draw_rectangle(marker.rect(), color=255, thickness=2)
    img.draw_cross(marker.cx(), marker.cy(), color=127, size=8)

    samples.append(
        (
            marker.cx(),
            marker.cy(),
            marker.w(),
            marker.h(),
        )
    )

    if len(samples) > STABLE_SAMPLES:
        samples.pop(0)

    if len(samples) < STABLE_SAMPLES:
        if time.ticks_diff(now_ms, last_report_ms) >= REPORT_INTERVAL_MS:
            print(
                "GRID_MARKER_STABILIZING: %d/%d"
                % (len(samples), STABLE_SAMPLES)
            )
            last_report_ms = now_ms

        continue

    cxs = [sample[0] for sample in samples]
    cys = [sample[1] for sample in samples]
    widths = [sample[2] for sample in samples]
    heights = [sample[3] for sample in samples]

    geometry_is_stable = (
        (max(cxs) - min(cxs)) <= MAX_CX_RANGE_PX
        and (max(cys) - min(cys)) <= MAX_CY_RANGE_PX
        and (max(widths) - min(widths)) <= MAX_W_RANGE_PX
        and (max(heights) - min(heights)) <= MAX_H_RANGE_PX
    )

    if not geometry_is_stable:
        if time.ticks_diff(now_ms, last_report_ms) >= REPORT_INTERVAL_MS:
            print("GRID_MARKER_UNSTABLE")
            last_report_ms = now_ms

        continue

    reference_cx = median(cxs)
    reference_cy = median(cys)
    reference_w = median(widths)
    reference_h = median(heights)

    if not reference_saved:
        with open(REFERENCE_FILE, "w") as reference_file:
            reference_file.write(
                "%d,%d,%d,%d\n"
                % (
                    reference_cx,
                    reference_cy,
                    reference_w,
                    reference_h,
                )
            )

        reference_saved = True
        print("GRID_MARKER_REFERENCE_SAVED: %s" % REFERENCE_FILE)

    if time.ticks_diff(now_ms, last_report_ms) < REPORT_INTERVAL_MS:
        continue

    print(
        "GRID_MARKER_REFERENCE: cx=%d, cy=%d, w=%d, h=%d"
        % (
            reference_cx,
            reference_cy,
            reference_w,
            reference_h,
        )
    )
    last_report_ms = now_ms
