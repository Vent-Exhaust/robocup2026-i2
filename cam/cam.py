import sensor, time, machine, math

# -------------------------
# Window / resolution
# -------------------------
window_x = 448
window_y = 352

# -------------------------
# Debug flags
# -------------------------
DEBUG_GOALS = False
DEBUG_BALL = False
DEBUG_FPS = False
DEBUG_DRAW = True

# -------------------------
# Camera setup
# -------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((int((640 - window_x) / 2), int((480 - window_y) / 2), window_x, window_y))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=20000)

# -------------------------
# Colour thresholds
# -------------------------
blue_thresholds   = [(23, 77, -79, 26, -47, -13)]
yellow_thresholds = [(41, 100, -12, 6, 18, 61)]
ball_thresholds   = [(0, 79, 20, 72, -128, 127)]

# -------------------------
# UART
# -------------------------
uart_obj = machine.UART(1, 115200)

color_configs = [
    ("BLUE",   blue_thresholds,   (0, 0, 255)),
    ("YELLOW", yellow_thresholds, (255, 255, 0)),
]

# -------------------------
# Mirror coordinate conversion
# -------------------------
CX = window_x // 2   # 224
CY = window_y // 2   # 176

# Degree-3 polynomial coefficients fit from calibration data
# d_cm = a*r^3 + b*r^2 + c*r + d  where r is radial pixel distance from centre
# X-axis fit (valid ~19–139 cm, r=105–206px)
POLY_X = (1.94466527e-04, -7.59780478e-02, 1.01680149e+01, -4.36382611e+02)
# Y-axis fit (valid ~19–89 cm, r=92–175px)
POLY_Y = (2.56781054e-04, -9.28917129e-02, 1.14339158e+01, -4.46999817e+02)

# Calibrated range limits
MIN_R_PX = 92.0    # closest calibration point (~19cm)
MAX_R_X  = 206.0   # furthest X calibration point (~139cm)
MAX_R_Y  = 175.0   # furthest Y calibration point (~89cm)

def _poly(coeffs, r):
    return coeffs[0]*r*r*r + coeffs[1]*r*r + coeffs[2]*r + coeffs[3]

def pixel_to_real(px, py):
    """
    Convert image pixel (px, py) to real-world position (x_cm, y_cm)
    relative to the robot centre.
    +x = right, +y = forward (up in image).
    """
    dx = px - CX
    dy = CY - py  # flip: up in image = positive Y

    r = math.sqrt(dx * dx + dy * dy)

    if r < 1.0:
        return (0.0, 0.0)

    # Blend X and Y polynomial fits weighted by angle
    # On X axis: wx=1, wy=0 -> POLY_X only
    # On Y axis: wx=0, wy=1 -> POLY_Y only
    # Diagonal: smooth blend
    cos_t = abs(dx) / r
    sin_t = abs(dy) / r
    wx = cos_t * cos_t
    wy = sin_t * sin_t

    # Clamp r to each axis's calibrated range
    r_x = max(MIN_R_PX, min(r, MAX_R_X))
    r_y = max(MIN_R_PX, min(r, MAX_R_Y))

    d_cm = wx * _poly(POLY_X, r_x) + wy * _poly(POLY_Y, r_y)

    x_cm = math.copysign(d_cm * cos_t, dx)
    y_cm = math.copysign(d_cm * sin_t, dy)

    return (x_cm, y_cm)

# -------------------------
# Main loop
# -------------------------
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    output_list = []

    # -------------------------
    # Goal detection
    # -------------------------
    for color_name, thresholds, draw_color in color_configs:
        blobs = img.find_blobs(
            thresholds,
            pixels_threshold=30,
            area_threshold=30,
            merge=False,
        )
        if blobs:
            cx, cy = window_x // 2, window_y // 2
            blobs = [b for b in blobs if b.pixels() > 50
                     and ((b.cx() - cx)**2 + (b.cy() - cy)**2) > 90**2]

        if blobs:
            if DEBUG_GOALS:
                for blob in blobs:
                    img.draw_rectangle(blob.rect(), color=draw_color)
                    img.draw_string(blob.x(), blob.y() - 10,
                                    "%s %dpx" % (color_name, blob.pixels()),
                                    color=draw_color)

            min_x = min(blob.x() for blob in blobs)
            min_y = min(blob.y() for blob in blobs)
            max_x = max(blob.x() + blob.w() for blob in blobs)
            max_y = max(blob.y() + blob.h() for blob in blobs)

            center_x = int(min_x + (max_x - min_x) / 2)
            center_y = int(min_y + (max_y - min_y) / 2)

            if DEBUG_DRAW:
                img.draw_cross(center_x, center_y, color=draw_color)

            output_list.extend([str(center_x), str(center_y)])
        else:
            output_list.extend(["none", "none"])

    # -------------------------
    # Ball detection
    # -------------------------
    ball_blobs = img.find_blobs(
        ball_thresholds,
        pixels_threshold=1,
        area_threshold=1,
        x_stride=4,
        y_stride=3,
    )

    if ball_blobs:
        if DEBUG_BALL:
            for blob in ball_blobs:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                img.draw_string(blob.x(), blob.y() - 10,
                                "%dpx r%.2f" % (blob.pixels(), blob.roundness()),
                                color=(255, 0, 0))

        best_ball = max(ball_blobs, key=lambda b: b.roundness())
        ball_px = best_ball.cx()
        ball_py = best_ball.cy()

        x_cm, y_cm = pixel_to_real(ball_px, ball_py)

        print("ball px=(%d,%d)  real=(%.1f, %.1f) cm" % (ball_px, ball_py, x_cm, y_cm))

        if DEBUG_DRAW:
            img.draw_cross(ball_px, ball_py, color=(255, 0, 0), size=10, thickness=2)
            img.draw_string(ball_px + 12, ball_py,
                            "%.0f,%.0f" % (x_cm, y_cm),
                            color=(255, 0, 0))

        output_list.extend([str(int(x_cm)), str(int(y_cm))])
    else:
        output_list.extend(["none", "none"])

    if DEBUG_FPS:
        img.draw_string(5, 5, "FPS: %.1f" % clock.fps(), color=(255, 255, 255))

    # UART output: BLUE_X,BLUE_Y,YELLOW_X,YELLOW_Y,BALL_X_CM,BALL_Y_CM
    uart_obj.write(",".join(output_list) + "\n")
