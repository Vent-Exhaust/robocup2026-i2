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

BALL_ROI_SIZE = 160  # square side length, adjust as needed

ball_roi = (
    240 - BALL_ROI_SIZE // 2,
    240 - BALL_ROI_SIZE // 2,
    BALL_ROI_SIZE,
    BALL_ROI_SIZE,
)

# -------------------------
# Debug flags
# -------------------------
DEBUG_DISABLE_ALL = False  # If True: disables ALL drawing and debug output for max speed
DEBUG_GOALS    = True
DEBUG_BALL     = True
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
sensor.set_auto_exposure(False, exposure_us=20000) # Jh home values
# sensor.set_auto_exposure(False, exposure_us=10000) # Robo lab values
# sensor.set_auto_exposure(False, exposure_us=20000) # CX home values (Night)
# sensor.set_auto_exposure(False, exposure_us=10000) # CX home values (Afternoon)
sensor.set_contrast(3)

# -------------------------
# Colour thresholds
# -------------------------

# Robo lab values
blue_thresholds   = [(43, 79, -16, 10, -49, -13)]
# yellow_thresholds = [(41, 100, -12, 6, 18, 61)]
# ball_thresholds   = [(0, 79, 20, 72, -128, 127)]
# blue_thresholds = [((30, 56, -128, -3, -128, -6))]
# yellow_thresholds = [(64, 96, 37, -21, 88, 31)]
# ball_thresholds = [(50, 100, 62, 16, 39, 12)]
# ball_thresholds = [(40, 80, 8, 53, 16, 36)]
# CX home values
# blue_thresholds = [(17, 52, -19, -3, -15, -4)]
# yellow_thresholds = [(41, 71, -7, 0, 12, 69)]
# yellow_thresholds = [(50, 59, -10, -2, 10, 20)]
# yellow_thresholds = [(61, 100, -4, 13, 40, 17)]
yellow_thresholds = [(87, 98, -14, -10, 32, 127)]
# ball_thresholds = [(29, 100, 2, 11, -2, 20)]
# Jh home values
ball_thresholds = [(52, 91, 14, 50, 18, 47)]

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

# Precompute distance at calibration floor for linear extrapolation
D_AT_MIN_R_X = _poly(POLY_X, MIN_R_PX)
D_AT_MIN_R_Y = _poly(POLY_Y, MIN_R_PX)

def _poly_safe(coeffs, r, r_min, r_max, d_at_min):
    """
    Evaluate poly within calibrated range; linearly extrapolate below r_min
    so that d_cm -> 0 as r -> 0 (monotonic, no inversion).
    """
    if r >= r_min:
        return _poly(coeffs, min(r, r_max))
    return d_at_min * (r / r_min)

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

    d_cm = (wx * _poly_safe(POLY_X, r, MIN_R_PX, MAX_R_X, D_AT_MIN_R_X)
          + wy * _poly_safe(POLY_Y, r, MIN_R_PX, MAX_R_Y, D_AT_MIN_R_Y))

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

def _inv_poly_safe(coeffs, d_cm, r_min, r_max, d_at_min):
    """Invert _poly_safe: d_cm -> r. For debug drawing only."""
    if d_cm <= 0.0:
        return 0.0
    if d_cm < d_at_min:
        # Linear region: d_cm = d_at_min * (r / r_min)  =>  r = r_min * d_cm / d_at_min
        return r_min * d_cm / d_at_min
    # Bisection in calibrated range
    lo, hi = r_min, r_max
    for _ in range(16):
        mid = (lo + hi) * 0.5
        if _poly(coeffs, mid) < d_cm:
            lo = mid
        else:
            hi = mid
    return (lo + hi) * 0.5

def robot_to_pixel(x_rob, y_rob):
    """Inverse of pixel_to_robot. Approximate, for debug drawing only."""
    # robot -> image: inverse of (-y_img, -x_img)
    x_img = -y_rob
    y_img = -x_rob
    d_cm = math.sqrt(x_img * x_img + y_img * y_img)
    if d_cm < 0.5:
        return (CX, CY)
    cos_t = abs(x_img) / d_cm
    sin_t = abs(y_img) / d_cm
    wx = cos_t * cos_t
    wy = sin_t * sin_t
    # Invert each axis, then blend
    r_x = _inv_poly_safe(POLY_X, d_cm, MIN_R_PX, MAX_R_X, D_AT_MIN_R_X) if wx > 0.01 else MIN_R_PX
    r_y = _inv_poly_safe(POLY_Y, d_cm, MIN_R_PX, MAX_R_Y, D_AT_MIN_R_Y) if wy > 0.01 else MIN_R_PX
    r = wx * r_x + wy * r_y
    dx = math.copysign(r * cos_t, x_img)
    dy = math.copysign(r * sin_t, y_img)
    px = int(CX + dx)
    py = int(CY - dy)
    return (px, py)

# -------------------------
# Ball Kalman filter
# -------------------------
# --- Tuning knobs ---
KALMAN_Q         = 2.0    # Process noise (cm^2/frame^2). Higher = trusts measurements more, tracks fast motion.
KALMAN_R         = 8.0    # Measurement noise (cm^2). Higher = smoother output but more lag.
KALMAN_MAX_COAST = 5      # Frames of velocity-based prediction before freezing position.
KALMAN_MAX_HOLD  = 60     # Frames to hold last position after coast expires. Then gives up. (e.g. 60 @ 20fps = 3s)
KALMAN_V_DECAY   = 0.95   # Velocity decay per coast frame (0-1). Prevents runaway extrapolation.

# Disappearance classification: how close (px) must the last blob be to the
# ROI border to count as "left the ROI" vs "went under the bot".
ROI_EDGE_MARGIN  = 15     # pixels from ROI edge

# Precompute ROI bounds
_ROI_X0 = ball_roi[0]
_ROI_Y0 = ball_roi[1]
_ROI_X1 = ball_roi[0] + ball_roi[2]
_ROI_Y1 = ball_roi[1] + ball_roi[3]

def _ball_near_roi_edge(px, py):
    """True if pixel is within ROI_EDGE_MARGIN of any ROI border."""
    return (px - _ROI_X0 < ROI_EDGE_MARGIN or _ROI_X1 - px < ROI_EDGE_MARGIN or
            py - _ROI_Y0 < ROI_EDGE_MARGIN or _ROI_Y1 - py < ROI_EDGE_MARGIN)

class BallKalman:
    """
    Lightweight 2D constant-velocity Kalman filter.
    State = [x, y, vx, vy] in robot-frame cm.
    Diagonal covariance only (no cross-terms) to stay fast on H7.
    """

    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.x  = [0.0, 0.0, 0.0, 0.0]   # state
        self.p  = [500.0, 500.0, 500.0, 500.0]  # diagonal covariance
        self.initialised = False
        self.coast_count = 0
        self.exit_to_edge = False  # True = ball left via ROI edge, False = went under bot

    def reset(self):
        self.initialised = False
        self.coast_count = 0
        self.exit_to_edge = False

    def _predict(self):
        self.x[0] += self.x[2]
        self.x[1] += self.x[3]
        self.p[0] += self.p[2] + self.q
        self.p[1] += self.p[3] + self.q
        self.p[2] += self.q
        self.p[3] += self.q

    def _update(self, zx, zy):
        kx  = self.p[0] / (self.p[0] + self.r)
        ky  = self.p[1] / (self.p[1] + self.r)
        kvx = self.p[2] / (self.p[0] + self.r)
        kvy = self.p[3] / (self.p[1] + self.r)
        ix = zx - self.x[0]
        iy = zy - self.x[1]
        self.x[0] += kx  * ix
        self.x[1] += ky  * iy
        self.x[2] += kvx * ix
        self.x[3] += kvy * iy
        self.p[0] *= (1.0 - kx)
        self.p[1] *= (1.0 - ky)
        self.p[2] *= (1.0 - kvx)
        self.p[3] *= (1.0 - kvy)

    def step(self, detected, zx=0.0, zy=0.0, near_roi_edge=False):
        """
        Call once per frame.
        Returns (valid, x, y, vx, vy).

        When ball disappears:
          - If last position was near ROI edge: coast briefly then none (ball left).
          - If last position was near center: coast then hold (ball is under bot).
        """
        # First detection
        if detected and not self.initialised:
            self.x = [zx, zy, 0.0, 0.0]
            self.p = [self.r, self.r, self.q * 4.0, self.q * 4.0]
            self.initialised = True
            self.coast_count = 0
            self.exit_to_edge = False
            return (True, zx, zy, 0.0, 0.0)

        if not self.initialised:
            return (False, 0.0, 0.0, 0.0, 0.0)

        if detected:
            self._predict()
            self._update(zx, zy)
            self.coast_count = 0
            self.exit_to_edge = False
        else:
            # On first missing frame, latch how the ball disappeared
            if self.coast_count == 0:
                self.exit_to_edge = near_roi_edge

            self.coast_count += 1

            if self.exit_to_edge:
                # Ball left the ROI — coast briefly then give up
                if self.coast_count <= KALMAN_MAX_COAST:
                    self._predict()
                    self.x[2] *= KALMAN_V_DECAY
                    self.x[3] *= KALMAN_V_DECAY
                else:
                    self.reset()
                    return (False, 0.0, 0.0, 0.0, 0.0)
            else:
                # Ball went under the bot — coast then hold
                if self.coast_count <= KALMAN_MAX_COAST:
                    self._predict()
                    self.x[2] *= KALMAN_V_DECAY
                    self.x[3] *= KALMAN_V_DECAY
                elif self.coast_count <= KALMAN_MAX_COAST + KALMAN_MAX_HOLD:
                    self.x[2] = 0.0
                    self.x[3] = 0.0
                else:
                    self.reset()
                    return (False, 0.0, 0.0, 0.0, 0.0)

        return (True, self.x[0], self.x[1], self.x[2], self.x[3])

ball_kf = BallKalman(KALMAN_Q, KALMAN_R)

# -------------------------
# Goal EMA filter
# -------------------------
GOAL_EMA_ALPHA = 0.7  # 0.0–1.0; lower = smoother but more lag

class GoalEMA:
    def __init__(self, alpha):
        self.a = alpha
        self.x = 0.0
        self.y = 0.0
        self.init = False

    def update(self, x, y):
        if not self.init:
            self.x, self.y = x, y
            self.init = True
        else:
            self.x += self.a * (x - self.x)
            self.y += self.a * (y - self.y)
        return self.x, self.y

    def reset(self):
        self.init = False

goal_filters = [GoalEMA(GOAL_EMA_ALPHA), GoalEMA(GOAL_EMA_ALPHA)]  # [blue, yellow]

# Track last known ball pixel position for disappearance classification
last_ball_px = CX
last_ball_py = CY

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
            x_stride=5,
            y_stride=5,
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

            # Apply EMA filter
            idx = 0 if color_name == "BLUE" else 1
            x_rob, y_rob = goal_filters[idx].update(x_rob, y_rob)
            dist  = math.sqrt(x_rob * x_rob + y_rob * y_rob)
            angle = robot_angle(x_rob, y_rob)

            if not DEBUG_DISABLE_ALL:
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
            if not DEBUG_DISABLE_ALL and DEBUG_DRAW:
                ema_px, ema_py = robot_to_pixel(x_rob, y_rob)
                img.draw_circle(ema_px, ema_py, 6, color=draw_color, thickness=2)
                img.draw_string(ema_px + 8, ema_py + 12,
                                "EMA %.0fcm %.0fd" % (dist, angle),
                                color=draw_color)

            output_list.extend([
                "%.1f" % x_rob,
                "%.1f" % y_rob,
                "%.1f" % dist,
                "%.1f" % angle,
            ])
        else:
            goal_filters[0 if color_name == "BLUE" else 1].reset()
            output_list.extend(["none", "none", "none", "none"])

    # -------------------------
    # Ball detection + Kalman
    # -------------------------
    ball_blobs = img.find_blobs(
        ball_thresholds,
        pixels_threshold=1,
        area_threshold=1,
        x_stride=2,
        y_stride=2,
        roi=ball_roi
    )

    ball_detected = False
    raw_x_rob = 0.0
    raw_y_rob = 0.0

    if ball_blobs:
        best_ball = max(ball_blobs, key=lambda b: b.roundness())
        ball_px = best_ball.cx()
        ball_py = best_ball.cy()

        raw_x_rob, raw_y_rob, _, _ = pixel_to_robot(ball_px, ball_py)
        ball_detected = True
        last_ball_px = ball_px
        last_ball_py = ball_py

        if not DEBUG_DISABLE_ALL and DEBUG_BALL:
            for b in ball_blobs:
                img.draw_rectangle(b.rect(), color=(255, 0, 0))
                img.draw_string(b.x(), b.y() - 10,
                                "%dpx r%.2f" % (b.pixels(), b.roundness()),
                                color=(255, 0, 0))

    # Run Kalman filter
    kf_valid, kf_x, kf_y, kf_vx, kf_vy = ball_kf.step(
        ball_detected, raw_x_rob, raw_y_rob,
        near_roi_edge=_ball_near_roi_edge(last_ball_px, last_ball_py) if not ball_detected else False
    )

    if kf_valid:
        dist  = math.sqrt(kf_x * kf_x + kf_y * kf_y)
        angle = robot_angle(kf_x, kf_y)

        if not DEBUG_DISABLE_ALL and DEBUG_DRAW:
            # Convert filtered robot-frame position back to pixel for drawing
            kf_px, kf_py = robot_to_pixel(kf_x, kf_y)

            # Green circle = Kalman filtered/predicted position
            img.draw_circle(kf_px, kf_py, 8, color=(0, 255, 0), thickness=2)
            img.draw_string(kf_px + 12, kf_py,
                            "%.0fcm %.0fd" % (dist, angle),
                            color=(0, 255, 0))

            # Red cross = raw measurement (only when detected)
            if ball_detected:
                img.draw_cross(ball_px, ball_py, color=(255, 0, 0), size=10, thickness=2)

            # Coast/hold indicator
            if ball_kf.coast_count > 0:
                if ball_kf.exit_to_edge:
                    img.draw_string(5, 460, "EDGE %d" % ball_kf.coast_count,
                                    color=(255, 128, 0))
                elif ball_kf.coast_count <= KALMAN_MAX_COAST:
                    img.draw_string(5, 460, "COAST %d" % ball_kf.coast_count,
                                    color=(255, 128, 0))
                else:
                    hold_frame = ball_kf.coast_count - KALMAN_MAX_COAST
                    img.draw_string(5, 460, "HOLD %d" % hold_frame,
                                    color=(255, 0, 255))

        output_list.extend([
            "%.1f" % kf_x,
            "%.1f" % kf_y,
            "%.1f" % dist,
            "%.1f" % angle,
        ])
    else:
        output_list.extend(["none", "none", "none", "none"])

    if not DEBUG_DISABLE_ALL:
        if DEBUG_FPS:
            img.draw_string(5, 5, "FPS: %.1f" % clock.fps(), color=(255, 255, 255))

    # -------------------------
    # Output
    # -------------------------
    if DEBUG_DISABLE_ALL:
        uart_obj.write(",".join(output_list) + "\n")
    elif DEBUG_BALL_PX:
        if ball_blobs:
            print("ball_px_rel=(%d, %d)" % (ball_px - CX, ball_py - CY))
        else:
            print("ball: none")
    else:
        print(",".join(output_list))
        uart_obj.write(",".join(output_list) + "\n")
