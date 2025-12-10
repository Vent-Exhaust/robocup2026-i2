import math

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
        length = math.sqrt(self.x * self.x + self.y * self.y)
        if length == 0:
            return Vec2(0, 0)
        return Vec2(self.x / length, self.y / length)

    def __repr__(self):
        return f"({self.x:.5f}, {self.y:.5f})"


# ------------------------------
# Cone Generator (Math Only)
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

    def generate_targets(self):
        self.targetPoints.clear()
        for i in range(self.resolution):
            x = i * self.range / self.resolution
            self.targetPoints.append(Vec2(x, 0))

    def generate_cone(self):
        self.conePoints.clear()
        self.conePoints.append(self.coneStartPos)

        for i in range(1, self.resolution):
            prev = self.conePoints[i - 1]

            pointToCamera = self.cameraPos - prev
            pointToTarget = self.targetPoints[i] - prev

            a1 = math.atan2(pointToTarget.y, pointToTarget.x)
            a2 = math.atan2(pointToCamera.y, pointToCamera.x)

            angle = (a1 - a2) / 2.0
            angle = math.pi / 2.0 - angle

            dx = pointToTarget.x * math.cos(angle) - pointToTarget.y * math.sin(angle)
            dy = pointToTarget.x * math.sin(angle) + pointToTarget.y * math.cos(angle)

            direction = Vec2(dx, dy).normalize()
            direction = direction * (self.approximateConeRadius / self.resolution)

            self.conePoints.append(prev + direction)


# ------------------------------
# Main Execution
# ------------------------------
if __name__ == "__main__":

    cone = Cone()

    # >>> You can tweak these freely <<<
    cone.resolution = 400
    cone.range = 350
    cone.cameraPos.y = 15.8
    cone.approximateConeRadius = 1.5
    cone.coneStartPos.y = 17

    cone.generate_targets()
    cone.generate_cone()

    # ------------------------------
    # OUTPUT AS CSV
    # ------------------------------
    print("radius_cm,height_cm")
    for p in cone.conePoints:
        print(f"{p.x:.6f},{p.y:.6f}")

    top = cone.conePoints[-1]
    print("\nFinal Radius:", round(top.x, 4), "cm")
    print("Final Height:", round(top.y, 4), "cm")
