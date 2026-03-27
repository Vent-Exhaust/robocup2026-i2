import sensor, time, machine, math

# -------------------------
# Window / resolution
# -------------------------
window_x = 480
window_y = 480

OPTICAL_OFFSET_X = 23
OPTICAL_OFFSET_Y = 0

OPTICAL_CENTER_TRIM_X = 0
OPTICAL_CENTER_TRIM_Y = 6

# -------------------------
# Debug flags
# -------------------------
DEBUG_GOALS    = True
DEBUG_BALL     = False
DEBUG_FPS      = False
DEBUG_DRAW     = True
DEBUG_BALL_PX  = False   # If True: only prints ball pixel coords relative to center

# -------------------------
# Camera setup
# -------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((
    int((640 - window_x) / 2) + OPTICAL_OFFSET_X,
    int((480 - window_y) / 2) + OPTICAL_OFFSET_Y,
    window_x,
    window_y
))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
# sensor.set_auto_exposure(False, exposure_us=20000) # Robo lab values
sensor.set_auto_exposure(False, exposure_us=35000) # CX home values
sensor.set_contrast(2)

# -------------------------
# Colour thresholds
# -------------------------

# Robo lab values
# blue_thresholds   = [(23, 77, -79, 26, -47, -13)]
# yellow_thresholds = [(41, 100, -12, 6, 18, 61)]
# ball_thresholds   = [(0, 79, 20, 72, -128, 127)]

# CX home values
blue_thresholds = [(0, 55, -31, -4, -128, -4)]
yellow_thresholds = [(41, 71, -7, 41, 12, 69)]
ball_thresholds = [(54, 100, 7, 42, 16, 127)]
# ball_thresholds = [(42, 100, 4, 20, 12, 21)]

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
CX = window_x // 2 + OPTICAL_CENTER_TRIM_X
CY = window_y // 2 + OPTICAL_CENTER_TRIM_Y

# Degree-5 polynomial fits (d_cm = a*r^5 + b*r^4 + c*r^3 + d*r^2 + e*r + f)
# Valid ~19-139 cm, r = 90-184 px
POLY_X = ( 3.13748841e-07, -2.12615728e-04,  5.70654064e-02,
          -7.56527068e+00,  4.94903474e+02, -1.27472480e+04)
POLY_Y = ( 3.13748841e-07, -2.12615728e-04,  5.70654064e-02,
          -7.56527068e+00,  4.94903474e+02, -1.27472480e+04)

MIN_R_PX = 90.0
MAX_R_X  = 184.0
MAX_R_Y  = 184.0

def _poly(coeffs, r):
    return (coeffs[0]*r**5 + coeffs[1]*r**4 + coeffs[2]*r**3
          + coeffs[3]*r**2 + coeffs[4]*r  + coeffs[5])

def pixel_to_image_xy(px, py):
    """
    Convert pixel (px, py) to real-world XY in the IMAGE frame (cm).
    +x_img = right in image, +y_img = up in image.
    """
    dx = px - CX
    dy = CY - py

    r = math.sqrt(dx * dx + dy * dy)
    if r < 1.0:
        return (0.0, 0.0)

    cos_t = abs(dx) / r
    sin_t = abs(dy) / r
    wx = cos_t * cos_t
    wy = sin_t * sin_t

    r_x = max(MIN_R_PX, min(r, MAX_R_X))
    r_y = max(MIN_R_PX, min(r, MAX_R_Y))

    d_cm = wx * _poly(POLY_X, r_x) + wy * _poly(POLY_Y, r_y)

    x_img = math.copysign(d_cm * cos_t, dx)
    y_img = math.copysign(d_cm * sin_t, dy)
    return (x_img, y_img)

def image_to_robot_xy(x_img, y_img):
    return (-y_img, -x_img)

def robot_angle(x_robot, y_robot):
    """
    Bearing in robot frame, degrees, 0-360 clockwise.
    0 deg = forward (+Y_robot)
    90 deg = right  (+X_robot)
    """
    angle = math.degrees(math.atan2(x_robot, y_robot))
    if angle < 0:
        angle += 360.0
    return angle

def pixel_to_robot(px, py):
    """Full pipeline: pixel -> image XY -> robot XY -> dist + bearing."""
    x_img, y_img = pixel_to_image_xy(px, py)
    x_rob, y_rob = image_to_robot_xy(x_img, y_img)
    dist  = math.sqrt(x_rob * x_rob + y_rob * y_rob)
    angle = robot_angle(x_rob, y_rob)
    return (x_rob, y_rob, dist, angle)

# -------------------------
# Main loop
# -------------------------
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # UART packet layout:
    # BLUE_X, BLUE_Y, BLUE_DIST, BLUE_ANGLE,
    # YELLOW_X, YELLOW_Y, YELLOW_DIST, YELLOW_ANGLE,
    # BALL_X, BALL_Y, BALL_DIST, BALL_ANGLE
    # All distances in cm, angles in degrees (0-360). "none" if not detected.
    output_list = []

    # -------------------------
    # Goal detection
    # -------------------------
    for color_name, thresholds, draw_color in color_configs:
        blobs = img.find_blobs(
            thresholds,
            pixels_threshold=30,
            area_threshold=30,
            x_stride = 5,
            y_stride = 5,
            merge=False,
        )
        if blobs:
            blobs = [b for b in blobs if b.pixels() > 50
                     and ((b.cx() - CX)**2 + (b.cy() - CY)**2) > 90**2]

        if blobs:
            min_x = min(b.x() for b in blobs)
            min_y = min(b.y() for b in blobs)
            max_x = max(b.x() + b.w() for b in blobs)
            max_y = max(b.y() + b.h() for b in blobs)
            center_px = int(min_x + (max_x - min_x) / 2)
            center_py = int(min_y + (max_y - min_y) / 2)

            x_rob, y_rob, dist, angle = pixel_to_robot(center_px, center_py)

            if DEBUG_GOALS:
                for b in blobs:
                    img.draw_rectangle(b.rect(), color=draw_color)
                    img.draw_string(b.x(), b.y() - 10,
                                    "%s %dpx" % (color_name, b.pixels()),
                                    color=draw_color)
            if DEBUG_DRAW:
                img.draw_cross(center_px, center_py, color=draw_color)
                img.draw_string(center_px + 8, center_py,
                                "%.0fcm %.0fd" % (dist, angle),
                                color=draw_color)

            output_list.extend([
                "%.1f" % x_rob,
                "%.1f" % y_rob,
                "%.1f" % dist,
                "%.1f" % angle,
            ])
        else:
            output_list.extend(["none", "none", "none", "none"])

    # -------------------------
    # Ball detection
    # -------------------------
    ball_blobs = img.find_blobs(
        ball_thresholds,
        pixels_threshold=1,
        area_threshold=1,
        x_stride=2,
        y_stride=2,
    )

    if ball_blobs:
        best_ball = max(ball_blobs, key=lambda b: b.roundness())
        ball_px = best_ball.cx()
        ball_py = best_ball.cy()

        x_rob, y_rob, dist, angle = pixel_to_robot(ball_px, ball_py)

        if DEBUG_BALL:
            for b in ball_blobs:
                img.draw_rectangle(b.rect(), color=(255, 0, 0))
                img.draw_string(b.x(), b.y() - 10,
                                "%dpx r%.2f" % (b.pixels(), b.roundness()),
                                color=(255, 0, 0))
        if DEBUG_DRAW:
            img.draw_cross(ball_px, ball_py, color=(255, 0, 0), size=10, thickness=2)
            img.draw_string(ball_px + 12, ball_py,
                            "%.0fcm %.0fd" % (dist, angle),
                            color=(255, 0, 0))

        output_list.extend([
            "%.1f" % x_rob,
            "%.1f" % y_rob,
            "%.1f" % dist,
            "%.1f" % angle,
        ])
    else:
        output_list.extend(["none", "none", "none", "none"])

    if DEBUG_FPS:
        img.draw_string(5, 5, "FPS: %.1f" % clock.fps(), color=(255, 255, 255))

    # -------------------------
    # Output
    # -------------------------
    if DEBUG_BALL_PX:
        if ball_blobs:
            print("ball_px_rel=(%d, %d)" % (ball_px - CX, ball_py - CY))
        else:
            print("ball: none")
    else:
        print(",".join(output_list))
        uart_obj.write(",".join(output_list) + "\n")
