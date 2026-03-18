#ifndef CALIBRATION_H
#define CALIBRATION_H

// Run the light ring calibration routine.
//
// Place the robot on the field (green) and move it across the white line
// during the calibration window. When done, thresholds are printed to Serial
// — copy the output into LDR_THRESHOLD in config.h.
//
// Duration is set by CALIBRATION_DURATION_MS in config.h.

void calibrateLightRing();

#endif // CALIBRATION_H
