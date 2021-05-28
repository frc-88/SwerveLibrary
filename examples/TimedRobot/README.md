# Timed Robot Example

This example demonstrates basic controls for a swerve drive using the TimedRobot framework without
Command-based programming.

Note that the project will not run if copied outside of this repository, because it references the
swerve library code directly in the settings.gradle file.

## Controls

The code used an XBox gamepad for controlls, with the following mapping:

 * Left stick - Steer the direction of translation. Does not set speed.
 * Right trigger - Sets the translation speed of the robot.
 * Right stick X - Spin the robot about it's center.
 * Right bumper - Hold for robot-centric steering. Otherwise, steering isn field-centric.
 * Y button - Zeros the gyro so that the robot is facing forwards. Only works while disabled.

## Important files

The following files are important to look at for the example.

* src\main\deploy\swerve.toml - The configuration file for the swerve drive.
* src\main\java\frc\robot\Robot.java - The code for the example.
