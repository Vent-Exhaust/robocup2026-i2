import sensor, time, machine

# Camera setup
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False, exposure_us=10000)

# Goal thresholds
orange_goal_thresholds = [(100, 0, 127, 27, -93, 127)]
blue_thresholds = [(23, 77, -79, 26, -47, -13)]
yellow_thresholds = [(41, 100, -12, 6, 18, 61)]
ball_thresholds = [(0, 79, 20, 72, -128, 127)]
# (23, 77, -79, 26, -47, -13)
# (34, 77, 31, 65, -128, 127)

# Ball threshold (can adjust later)

# UART
uart_obj = machine.UART(1,115200)

color_configs = [
    ("ORANGE_GOAL", orange_goal_thresholds, (255,165,0)),
    ("BLUE", blue_thresholds, (0,0,255)),
    ("YELLOW", yellow_thresholds, (255,255,0))
]

while True:

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

            for blob in blobs:
                img.draw_rectangle(blob.rect(), color=draw_color)

            min_x = min(blob.x() for blob in blobs)
            min_y = min(blob.y() for blob in blobs)
            max_x = max(blob.x()+blob.w() for blob in blobs)
            max_y = max(blob.y()+blob.h() for blob in blobs)

            width = max_x - min_x
            height = max_y - min_y

            img.draw_rectangle(min_x, min_y, width, height, color=draw_color, thickness=3)

            center_x = int(min_x + width/2)
            center_y = int(min_y + height/2)

            img.draw_cross(center_x, center_y, color=draw_color)

            output_list.extend([str(center_x), str(center_y)])

        else:
            output_list.extend(["none","none"])

# -------------------------
# BALL DETECTION (LAB RANGE)
# -------------------------


ball_blobs = img.find_blobs(
    ball_thresholds,
    pixels_threshold=0,
    area_threshold=0
)

if ball_blobs:

    # choose the most circular blob
    best_ball = None
    best_score = 0

    for blob in ball_blobs:

        circularity = blob.roundness()

        if circularity > best_score:
            best_score = circularity
            best_ball = blob

    if best_ball:

        img.draw_rectangle(best_ball.rect(), color=(255,0,0))
        img.draw_cross(best_ball.cx(), best_ball.cy(), color=(255,0,0))

        ball_x = best_ball.cx()
        ball_y = best_ball.cy()

        output_list.extend([str(ball_x), str(ball_y)])

else:
    output_list.extend(["none","none"])
