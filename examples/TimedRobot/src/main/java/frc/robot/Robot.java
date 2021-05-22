// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team88.swerve.SwerveController;

/**
 * Robot using Team 88's SwerveLibrary, using the TimedRobot framework without command-based
 * programming. The controller in use is an XBox controller, with the following mapping:
 * 
 * Left stick - Steer the direction of translation. Does not set speed.
 * Right trigger - Sets the translation speed of the robot.
 * Right stick X - Spin the robot about it's center.
 * Right bumper - Hold for robot-centric steering. Otherwise, steering isn field-centric.
 * Y button - Zeros the gyro so that the robot is facing forwards. Only works while disabled.
 */
public class Robot extends TimedRobot {

  // The swerve controller object. Used for all swerve-drive related operations.
  private SwerveController swerve;

  // The xBox gamepad.
  private Joystick gamepad;

  // The max translation and rotation speeds to be commanded the robot.
  private static final double MAX_SPEED = 14.7; // feet per second
  private static final double MAX_ROTATION = 90.; // degrees per second

  // The port of the XBox gamepad.
  private static final int GAMEPAD_PORT = 0;
  
  // The deadband to use for XBox gamepad joysticks.
  private static final double JOYSTICK_DEADBAND = 0.1;

  // The threshold for left stick magnitude to change translation direction.
  private static final double CHANGE_DIRECTION_THRESHOLD = 0.25;

  // The speed thresholds for hold modules in their current direction.
  private static final double HOLD_DIRECTION_TRANSLATION_THRESHOLD = 0.1;
  private static final double HOLD_DIRECTION_ROTATION_THRESHOLD = 1;

  @Override
  public void robotInit() {
    // swerve.toml is a file in the src/main/deploy directory that is used to configure your
    // swerve drive.
    this.swerve = new SwerveController("swerve.toml");

    // Zero the gyro upon initialization.
    this.swerve.setGyroYaw(0);

    this.gamepad = new Joystick(GAMEPAD_PORT);
  }

  @Override
  public void robotPeriodic() {
    // Must be called on every loop of code to update controls, publishing values, etc.
    // This happens after the teleopInit() method, so the latest command will already be set.
    this.swerve.update();
  }

  @Override
  public void disabledPeriodic() {
    // Y button zeros the gyro, which tells the robot that it is currently facing forwards.
    if (gamepad.getRawButton(4)) {
      this.swerve.setGyroYaw(0);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Set the swerve drive to just hold the current position of the modules without driving the
    // wheels.
    this.swerve.holdDirection();
  }

  /**
   * Calculates the angle of translation set by the left stick.
   * 
   * @return The angle of translation, in degrees. 0 corresponds to forwards, and positive
   * corresponds to counterclockwise.
   */
  private double calculateTranslationDirection() {
    // The x and y axis values. Y is inverted so that down is positive on XBox controllers, so we
    // need to invert it back.
    double x = gamepad.getRawAxis(0);
    double y = -gamepad.getRawAxis(1);
    
    // Calculate the angle. By swapping x and y, and inverting x, we get the desired coordinate
    // system where 0 is forwards and positive is counterclockwise.
    return Math.toDegrees(Math.atan2(-x, y));
  }

  /**
   * Determines if the left stick is pressed out far enough to merit changing the translation
   * direction. If the joystick is close to the center, it is too difficult to control the
   * direction.
   * 
   * @return True if the current translation direction should be changed, false if it should stay
   * the same.
   */
  private boolean shouldChangeDirection() {
    // The x and y axis values. Y is inverted so that down is positive on XBox controllers, so we
    // need to invert it back.
    double x = gamepad.getRawAxis(0);
    double y = -gamepad.getRawAxis(1);

    // Calculate the magnitude of the joystick position and use it as the threshold.
    return Math.sqrt(x * x + y * y) >= CHANGE_DIRECTION_THRESHOLD;
  }

  @Override
  public void teleopPeriodic() {
    // Get the translation speed from the right trigger, scaled linearly so that fully pressed
    // commands our max speed in feet per second. Because the triggers on the XBox controller
    // actually go to zero when released, no deadband is needed.
    double translationSpeed = this.gamepad.getRawAxis(3);

    // Get the rotation velocity from the right stick X axis, scaled linearly so that fully pushed
    // commands our max rotation speed in rotations per second. Uses a deadband since XBox
    // controller joysticks don't get exactly to zero when released.
    double rotationVelocity = this.gamepad.getRawAxis(4);
    if (Math.abs(rotationVelocity) < JOYSTICK_DEADBAND) {
      rotationVelocity = 0;
    }
    rotationVelocity *= MAX_ROTATION;

    // Set the translation speed and rotation velocities.
    this.swerve.setVelocity(
        translationSpeed, rotationVelocity);

    // Determine if the left stick is pressed enough to merit changing the direction.
    if (this.shouldChangeDirection()) {
      // The translation direction is field-centric if the right bumper is not pressed.
      boolean isFieldCentric = !this.gamepad.getRawButton(6);

      // Set the translation direction from the left stick.
      this.swerve.setTranslationDirection(
        this.calculateTranslationDirection(), isFieldCentric);

    // If we aren't changing translation direction, and we aren't commanding a significant speed in
    // either translation or rotation, just hold the modules in their current position.
    } else if (translationSpeed < HOLD_DIRECTION_TRANSLATION_THRESHOLD
        && Math.abs(rotationVelocity) < HOLD_DIRECTION_ROTATION_THRESHOLD) {
      this.swerve.holdDirection();
    }
  }

  @Override
  public void testPeriodic() {
    // Set the swerve drive to just hold the current position of the modules without driving the
    // wheels.
    this.swerve.holdDirection();
  }
}
