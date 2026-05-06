# robocup2026
# Vent Exhaust - RoboCup Singapore Soccer Infrared Junior 

<!-- <img src="https://github.com/2-chairs/robocup2025/blob/main/zLogo/v3/logo%20render%20v3.png" alt="" width="100"/> -->

*Repository for the official electronic, mechanical, and software files of Vent Exhaust (VE) 1, formerly known as 2 Chairs.*

<p align="center">
  <a href="media/photos/_DSC8658.jpg">
    <img src="media/photos/thumbs/hero-robot.png" alt="Vent Exhaust robot" width="500">
  </a>
</p>

## About

This repository is for **Vent Exhaust**, a Singapore RoboCup Infrared Soccer Junior team, and includes relevant design files and resources.

## Repository Structure

- **elec/** - PCB designs, schematics, and component footprints.
- **mech/** - 3D-printable files in **.3MF** and **.STL** formats; includes rendered images of the robot and its mirrored counterpart for reference and other files related to mechanical design.
- **code/** - PlatformIO firmware for the 3-layer robot architecture (Layer 1-3 Teensies / ESP32).
- **cam/** - MicroPython camera firmware (`cam.py`) running on an OpenMV/MaixCam, detecting goals and ball over serial.
- **branding/** - Official team branding and design assets.
- **media/** - Robot photos, gallery thumbnails, screenshots, and reference images used by the README and project documentation.

## Mechanical CAD

- [View the Fusion 360 model](https://a360.co/4tRNy74)

## Robot Gallery

<p align="center">
  <a href="media/photos/_DSC9160.jpg">
    <img src="media/photos/thumbs/robot-front.png" alt="Robot front view" width="180">
  </a>
  <a href="media/photos/_DSC9165.jpg">
    <img src="media/photos/thumbs/robot-side.png" alt="Robot side view" width="180">
  </a>
  <a href="media/photos/_DSC9409.jpg">
    <img src="media/photos/thumbs/robot-9409.png" alt="Robot gallery photo" width="180">
  </a>
  <br>
  <a href="media/photos/_DSC9808.jpg">
    <img src="media/photos/thumbs/robot-9808.png" alt="Robot gallery photo" width="180">
  </a>
  <a href="media/photos/_DSC9811.jpg">
    <img src="media/photos/thumbs/robot-9811.png" alt="Robot gallery photo" width="180">
  </a>
  <a href="media/photos/_DSC9834.jpg">
    <img src="media/photos/thumbs/robot-9834.png" alt="Robot gallery photo" width="180">
  </a>
</p>

## Robot Architecture

Three processors communicate over UART using a custom SerialComm packet protocol:

| Layer | Hardware | Role |
|-------|----------|------|
| Layer 1 | Teensy 4.0 | 32-LDR light ring - white line detection |
| Layer 2 | Teensy 4.0 | Main brain - strategy, X-drive motors, IMU heading hold, localisation, scoring |
| Layer 3 | Seeed XIAO ESP32-C3 | 28-IR sensor ring - ball detection + 4 DIP switches |
| Camera | MaixCam (MicroPython) | Dual-goal + ball vision -> serial CSV to Layer 2 |

## Branch Archive

`main` is the active branch. Previous development branches are preserved as
Git tags under `archive/`, such as `archive/i4-v3` and `archive/striker`.

## License & Copyright

Copyright 2026 Vent Exhaust. All rights reserved, except where otherwise stated.
