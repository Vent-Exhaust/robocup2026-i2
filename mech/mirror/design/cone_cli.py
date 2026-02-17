import math
import argparse

# ------------------------------
# Simple 2D Vector
# ------------------------------
class Vec2:
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, b):
        return Vec2(self.x + b.x, self.y + b.y)

    def __sub__(self, b):
        return Vec2(self.x - b.x, self.y - b.y)

    def __mul__(self, s):
        return Vec2(self.x * s, self.y * s)

    def normalize(self):
        l = math.hypot(self.x, self.y)
        return Vec2(self.x/l, self.y/l) if l else Vec2(0,0)


# ------------------------------
# Cone math (same as your C++)
# ------------------------------
class Cone:
    def __init__(self):
        self.cameraPos = Vec2(0, 15.8)
        self.resolution = 400
        self.range = 350.0
        self.approximateConeRadius = 1.5
        self.coneStartPos = Vec2(0, 17)

        self.targetPoints = []
        self.conePoints = []

    def generate(self):
        self.targetPoints = [
            Vec2(i*self.range/self.resolution, 0)
            for i in range(self.resolution)
        ]

        self.conePoints = [self.coneStartPos]

        for i in range(1, self.resolution):
            prev = self.conePoints[-1]

            pt_cam = self.cameraPos - prev
            pt_tgt = self.targetPoints[i] - prev

            a1 = math.atan2(pt_tgt.y, pt_tgt.x)
            a2 = math.atan2(pt_cam.y, pt_cam.x)

            angle = math.pi/2 - (a1-a2)/2

            dx = pt_tgt.x*math.cos(angle) - pt_tgt.y*math.sin(angle)
            dy = pt_tgt.x*math.sin(angle) + pt_tgt.y*math.cos(angle)

            d = Vec2(dx,dy).normalize() * (self.approximateConeRadius/self.resolution)
            self.conePoints.append(prev + d)

    def diameter(self):
        return 2*self.conePoints[-1].x


# ------------------------------
# Export OpenSCAD
# ------------------------------
def export_scad(cone, filename="cone.scad"):
    pts = [[0, cone.conePoints[0].y]] + \
          [[p.x, p.y] for p in cone.conePoints] + \
          [[0, cone.conePoints[-1].y]]

    with open(filename, "w") as f:
        f.write("points=[\n")
        for p in pts:
            f.write(f"[{p[0]:.6f},{p[1]:.6f}],\n")
        f.write("];\n\n")
        f.write("rotate_extrude($fn=200) polygon(points);\n")


# ------------------------------
# MAIN with CLI
# ------------------------------
if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("--reso", type=int, default=400)
    parser.add_argument("--cam", type=float, default=15.8)
    parser.add_argument("--range", type=float, default=350)
    parser.add_argument("--radius", type=float, default=1.5)
    parser.add_argument("--start", type=float, default=17)
    parser.add_argument("--diameter", type=float, default=None)

    args = parser.parse_args()

    cone = Cone()

    cone.resolution = args.reso
    cone.cameraPos.y = args.cam
    cone.range = args.range
    cone.approximateConeRadius = args.radius
    cone.coneStartPos.y = args.start

    # ------------------------------
    # Optional: force diameter (auto solve)
    # ------------------------------
    if args.diameter:
        target = args.diameter / 2
        for _ in range(30):   # binary search radius
            cone.generate()
            r = cone.conePoints[-1].x
            if r < target:
                cone.approximateConeRadius *= 1.1
            else:
                cone.approximateConeRadius *= 0.9

    cone.generate()
    export_scad(cone)

    print("Final diameter:", round(cone.diameter(),2), "cm")
    print("Wrote cone.scad")
