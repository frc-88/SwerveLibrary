# Command Based Robot Example

This example demonstrates basic controls for a swerve drive using WPILIB's command-based
programming framework.

Note that the project will not run if copied outside of this repository, because it references the
swerve library code directly in the settings.gradle file.

To run the project outside of this repository you will need to change the line in build.gradle from:

```groovy
compile project(':SwerveLibrary')
```

To:

```groovy
compile "io.github.frc-88:swerve:0.2.0"
```

And then remove these lines from settings.gradle:

```groovy
include 'SwerveLibrary'
project(':SwerveLibrary').projectDir = new File('../../')
```

## Controls

The code uses an XBox gamepad for controls, with 4 options controlled by a chooser on the Dashboard:

2 Joysticks with Gas Pedal:
 * Left stick - Steer the direction of translation. Does not set speed.
 * Right trigger - Sets the translation speed of the robot.
 * Right stick X - Spin the robot about it's center.

Split Joysticks with Triggers Turning:
 * Left stick Y - Steer and control the speed of the Y direction of translation.
 * Right stick X - Steer and control the speed of the X direction of translation.
 * Left Trigger - Spin the robot counter clockwise
 * Right Trigger - Spin the robot clockwise

Single Joystick with Triggers Turning:
 * Left stick - Steer and control the speed of the direction of translation.
 * Left Trigger - Spin the robot counter clockwise
 * Right Trigger - Spin the robot clockwise

Single Joystick with Joystick X Turning:
 * Left stick - Steer and control the speed of the direction of translation.
 * Right stick X - Spin the robot about it's center.

All:
 * Right bumper - Hold for robot-centric steering. Otherwise, steering is field-centric.
 * Left bumper - Turtle mode, significantly reduces the speed for fine control.
 * Y button - Zeros the gyro so that the robot is facing forwards. Only works while disabled.

## Important files

The following files are important to look at for the example.

* src\main\deploy\swerve.toml - The configuration file for the swerve drive.
* src\main\java\frc\robot\subsystems\Drivetrain.java - The subsystem for the swerve drivetrain.
* src\main\java\frc\robot\commands\ManualControl.java - Maps the gamepad to controls for teleop driving.
* src\main\java\frc\robot\commands\SetGyroYaw.java - Sets the heading of the robot.
* src\main\java\frc\robot\Constants.java - Contains some constants used throughout the code.
* src\main\java\frc\robot\RobotContainer.java - Sets up the subsystem and commands.
* src\main\java\frc\robot\Robot.java - Slightly modified to update the swerve after running the command scheduler in `robotPeriodic()`
