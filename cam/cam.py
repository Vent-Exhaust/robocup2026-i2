import sensor, time, machine

window_x = 448
window_y = 352

# Debug flags
DEBUG_GOALS = False
DEBUG_BALL = False
DEBUG_FPS = False

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.set_windowing((int((640-window_x)/2), int((480-window_y)/2), window_x, window_y))
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=20000)

# Goal thresholds
blue_thresholds = [(23, 77, -79, 26, -47, -13)]
yellow_thresholds = [(41, 100, -12, 6, 18, 61)]
ball_thresholds = [(0, 79, 20, 72, -128, 127)]

# UART
uart_obj = machine.UART(1, 115200)

color_configs = [
    ("BLUE", blue_thresholds, (0, 0, 255)),
    ("YELLOW", yellow_thresholds, (255, 255, 0))
]

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    output_list = []

    # -------------------------
    # GOAL DETECTION
    # -------------------------
    for color_name, thresholds, draw_color in color_configs:
        blobs = img.find_blobs(
            thresholds,
            pixels_threshold=30,
            area_threshold=30,
            merge=False
        )
        if blobs:
            cx, cy = window_x // 2, window_y // 2
            blobs = [b for b in blobs if b.pixels() > 50
                     and ((b.cx() - cx)**2 + (b.cy() - cy)**2) > 90**2]
        if blobs:
            if DEBUG_GOALS:
                for blob in blobs:
                    img.draw_rectangle(blob.rect(), color=draw_color)
                    img.draw_string(blob.x(), blob.y() - 10, "%s %dpx" % (color_name, blob.pixels()), color=draw_color)
            min_x = min(blob.x() for blob in blobs)
            min_y = min(blob.y() for blob in blobs)
            max_x = max(blob.x() + blob.w() for blob in blobs)
            max_y = max(blob.y() + blob.h() for blob in blobs)
            width = max_x - min_x
            height = max_y - min_y
            center_x = int(min_x + width / 2)
            center_y = int(min_y + height / 2)
            img.draw_cross(center_x, center_y, color=draw_color)
            output_list.extend([str(center_x), str(center_y)])
        else:
            output_list.extend(["none", "none"])

    # -------------------------
    # BALL DETECTION
    # -------------------------
    ball_blobs = img.find_blobs(
        ball_thresholds,
        pixels_threshold=1,
        area_threshold=1,
        x_stride=4,
        y_stride=3
    )
    if ball_blobs:
        if DEBUG_BALL:
            for blob in ball_blobs:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0))
                img.draw_string(blob.x(), blob.y() - 10, "%dpx r%.2f" % (blob.pixels(), blob.roundness()), color=(255, 0, 0))
        best_ball = max(ball_blobs, key=lambda b: b.roundness())
        ball_x = best_ball.cx()
        ball_y = best_ball.cy()
        img.draw_cross(ball_x, ball_y, color=(255, 0, 0), size=10, thickness=2)
        output_list.extend([str(ball_x), str(ball_y)])
    else:
        output_list.extend(["none", "none"])

    if DEBUG_FPS:
        img.draw_string(5, 5, "FPS: %.1f" % clock.fps(), color=(255, 255, 255))
