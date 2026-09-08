import sensor
import time


# Run this file while the chassis is placed at the exact build target pose
# and the camera is facing the QR code.  Keep the robot still.  Copy the
# printed QR_REFERENCE line back into the mission program.

STABLE_SAMPLES = 15
MAX_CX_RANGE_PX = 4
MAX_CY_RANGE_PX = 4
MAX_W_RANGE_PX = 4
MAX_H_RANGE_PX = 4
REPORT_INTERVAL_MS = 500
REFERENCE_FILE = "build_qr_target.txt"


def median(values):
    ordered = list(values)
    ordered.sort()
    return ordered[len(ordered) // 2]


def largest_qr(qr_codes):
    selected = None

    for qr_code in qr_codes:
        if selected is None:
            selected = qr_code
        elif (qr_code.w() * qr_code.h()) > (selected.w() * selected.h()):
            selected = qr_code

    return selected


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=3000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

samples = []
last_report_ms = time.ticks_ms()
reference_saved = False

print("QR_CALIBRATION_READY: keep chassis still at build target")


while True:
    img = sensor.snapshot()
    qr_code = largest_qr(img.find_qrcodes())

    if qr_code is None:
        samples = []

        if time.ticks_diff(time.ticks_ms(), last_report_ms) >= REPORT_INTERVAL_MS:
            print("QR_REFERENCE_NONE")
            last_report_ms = time.ticks_ms()

        continue

    img.draw_rectangle(qr_code.rect(), color=(0, 255, 0), thickness=2)
    img.draw_cross(qr_code.cx(), qr_code.cy(), color=(255, 0, 0))

    samples.append(
        (
            qr_code.cx(),
            qr_code.cy(),
            qr_code.w(),
            qr_code.h(),
        )
    )

    if len(samples) > STABLE_SAMPLES:
        samples.pop(0)

    if len(samples) < STABLE_SAMPLES:
        continue

    cxs = [sample[0] for sample in samples]
    cys = [sample[1] for sample in samples]
    widths = [sample[2] for sample in samples]
    heights = [sample[3] for sample in samples]

    stable = (
        (max(cxs) - min(cxs)) <= MAX_CX_RANGE_PX
        and (max(cys) - min(cys)) <= MAX_CY_RANGE_PX
        and (max(widths) - min(widths)) <= MAX_W_RANGE_PX
        and (max(heights) - min(heights)) <= MAX_H_RANGE_PX
    )

    if not stable:
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
        print("QR_REFERENCE_SAVED: %s" % REFERENCE_FILE)

    if time.ticks_diff(time.ticks_ms(), last_report_ms) < REPORT_INTERVAL_MS:
        continue

    print(
        "QR_REFERENCE: cx=%d, cy=%d, w=%d, h=%d, payload=%s"
        % (
            reference_cx,
            reference_cy,
            reference_w,
            reference_h,
            qr_code.payload(),
        )
    )
    last_report_ms = time.ticks_ms()
