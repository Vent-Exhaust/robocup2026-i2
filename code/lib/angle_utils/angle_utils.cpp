#include "angle_utils.h"
#include <cmath>

double clipAngleTo360(double angle) {
    angle = fmod(angle, 360);
    return angle < 0 ? angle + 360 : angle;
}

double smallerAngleDifference(double leftAngle, double rightAngle) {
    double diff = fmod(fabs(rightAngle - leftAngle), 360);
    return diff > 180 ? 360 - diff : diff;
}

double angleBisector(double leftAngle, double rightAngle) {
    double diff = smallerAngleDifference(leftAngle, rightAngle);
    return clipAngleTo360(leftAngle + diff / 2.0);
}
