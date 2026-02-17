#include "raylib.h"
#include <glm/glm.hpp>
#include <iostream>
#include <vector>

// ALL UNITS ARE CM
// 00 is the center of the robot at the point of the ground

class Cone {
public:
  glm::vec2 cameraPos = {0, 13};
  int resolution = 200;
  int range = 200;

  glm::vec2 origin = {45 * 16, 26 * 32 - 200};
  glm::vec2 factor = {20, -20};

  float approximateConeRadius = 3;

  std::vector<glm::vec2> targetPoints;

  std::vector<glm::vec2> conePoints;

  glm::vec2 coneStartPos = {0, 15};

  Cone() {
    for (int i = 0; i < resolution; i++) {
      targetPoints.push_back({i * range / resolution, 0});
    }
  }

  void draw() {

    DrawLine(origin.x, 0, origin.x, GetScreenHeight(), {0, 0, 0, 255});
    DrawLine(0, origin.y, GetScreenWidth(), origin.y, {0, 0, 0, 255});

    drawCircleFromOrigin(cameraPos);

    for (int i = 0; i < resolution; i++) {
      drawCircleFromOrigin(targetPoints[i], 5, {0, 0, 255, 255});
    }

    drawCone();
  }

  void update() {
    if (IsMouseButtonDown(0)) {
      /* cameraPos = {(GetMouseX() - origin.x) / factor.x, (GetMouseY() - origin.y) / factor.y}; */
      cameraPos.y = (GetMouseY() - origin.y) / factor.y;
    }
    if (IsMouseButtonDown(2)) {
      coneStartPos.y = (GetMouseY() - origin.y) / factor.y;
    }
    generateCone();
  }

private:
  void drawCircleFromOrigin(glm::vec2 pos, int r = 5, Color col = {255, 0, 0, 255}) {
    DrawCircle(pos.x * factor.x + origin.x, origin.y + pos.y * factor.y, r, col);
  }

  void generateCone() {
    conePoints = {coneStartPos};
    for (int i = 1; i < resolution; i++) {
      glm::vec2 pointToCamera = cameraPos - conePoints[i - 1];
      glm::vec2 pointToTarget = targetPoints[i] - conePoints[i - 1];

      float angle = (atan2(pointToTarget.y, pointToTarget.x) - atan2(pointToCamera.y, pointToCamera.x)) / 2;
      angle = 1.5707963268 - angle;

      glm::vec2 pointDiff = {pointToTarget.x * glm::cos(angle) - pointToTarget.y * sin(angle), pointToTarget.x * glm::sin(angle) + pointToTarget.y * cos(angle)};
      pointDiff = glm::normalize(pointDiff);
      pointDiff *= approximateConeRadius / resolution;

      conePoints.push_back(conePoints[i - 1] + pointDiff);

      std::cout << angle << std::endl;
    }
  }

  void drawCone() {
    for (int i = 0; i < resolution; i++) {
      drawCircleFromOrigin(conePoints[i], 1, {255, 0, 0, 255});
      drawCircleFromOrigin({-conePoints[i].x, conePoints[i].y}, 1, {255, 0, 0, 255});
    }
  }
};

void update(Cone &cone) {
  cone.update();
}

void draw(Cone &cone) {
  cone.draw();
}

int main() {
  const int screenWidth = 32 * 45;
  const int screenHeight = 32 * 26;

  InitWindow(screenWidth, screenHeight, "Game");
  SetTargetFPS(60);

  Cone cone = Cone();

  while (!WindowShouldClose()) {
    BeginDrawing();
    ClearBackground(RAYWHITE);
    update(cone);

    draw(cone);

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
