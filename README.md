# PedroTest

FTC 2024–2025 INTO THE DEEP robot code based on the Pedro Pathing quickstart. It contains competition autonomous and teleop programs, lift/extension/intake subsystems, vision experiments, and the Pedro localisation/tuning suite.

## Competition programs

- `Pedro 5+0`, `Red Sample: 0+4`, and `Red Vision: 0+1` autonomous routines
- main, red-sample, and red-specimen teleop modes
- sample/specimen robot abstractions and subsystem implementations
- OpenCV, Limelight, homography, claw, lift, extension, pivot, and autonomous test OpModes

## Build and deploy

Install Android Studio, open the repository, allow Gradle to sync, and build:

```bash
./gradlew :TeamCode:assembleDebug
```

Use Android Studio's `FtcRobotController` run configuration to install on a Control Hub/Robot Controller device.

## Robot configuration

The robot code expects drive motors `left_front`, `left_back`, `right_front`, and `right_back`; paired lift and extension motors; intake/angle/rotate servos; `spinMotor`; an analog `encoder`; goBILDA Pinpoint `odo`; `limelight`; and, for some tests, `Webcam 1`.

Check `robot/*Constants.java`, `pedroPathing/constants/`, and the selected OpMode before running. Hardware directions, gains, servo positions, vision thresholds, and paths are specific to the original robot.

## Source layout

- `TeamCode/src/main/java/opmodes/` and `teleop/` — primary programs.
- `robot/` and `shared/` — robot and mechanism abstractions.
- `pedroPathing/` — constants, examples, and tuning programs.
- `vision/` and `util/` — camera pipelines and geometry helpers.
- `tests/` — hardware, vision, and autonomous experiments.

For Pedro Pathing setup and tuning concepts, see [pedropathing.com](https://pedropathing.com/).
