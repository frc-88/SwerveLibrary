// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Commands the swerve drive using commands from an XBox controller with the following mapping:
 * 
 * Left stick - Steer the direction of translation. Does not set speed.
 * Right trigger - Sets the translation speed of the robot.
 * Right stick X - Spin the robot about it's center.
 * Right bumper - Hold for robot-centric steering. Otherwise, steering isn field-centric.
 */
public class ManualDrive extends CommandBase {

  // The drivetrain subsystem.
  private final Drivetrain m_drivetrain;

  // The XBox controller.
  private final Joystick m_controller;

  // The deadband to use for XBox gamepad joysticks.
  private static final double JOYSTICK_DEADBAND = 0.1;

  // The threshold for left stick magnitude to change translation direction.
  private static final double CHANGE_DIRECTION_THRESHOLD = 0.25;

  // The speed thresholds for hold modules in their current direction.
  private static final double HOLD_DIRECTION_TRANSLATION_THRESHOLD = 0.1;
  private static final double HOLD_DIRECTION_ROTATION_THRESHOLD = 1;

  /**
   * Construct.
   *
   * @param drivetrain The drivetrain subsystem used by this command.
   * @param controller The XBox controller used to control the drivetrain.
   */
  public ManualDrive(Drivetrain drivetrain, Joystick controller) {
    m_drivetrain = drivetrain;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the translation speed from the right trigger, scaled linearly so that fully pressed
    // commands our max speed in feet per second. Because the triggers on the XBox controller
    // actually go to zero when released, no deadband is needed.
    double translationSpeed = m_controller.getRawAxis(3);

    // Get the rotation velocity from the right stick X axis, scaled linearly so that fully pushed
    // commands our max rotation speed in rotations per second. Uses a deadband since XBox
    // controller joysticks don't get exactly to zero when released.
    double rotationVelocity = m_controller.getRawAxis(4);
    if (Math.abs(rotationVelocity) < JOYSTICK_DEADBAND) {
      rotationVelocity = 0;
    }
    rotationVelocity *= Constants.MAX_SPEED;

    // Set the translation speed and rotation velocities.
    m_drivetrain.setVelocity(
        translationSpeed, rotationVelocity);

    // Determine if the left stick is pressed enough to merit changing the direction.
    if (this.shouldChangeDirection()) {
      // The translation direction is field-centric if the right bumper is not pressed.
      boolean isFieldCentric = !m_controller.getRawButton(6);

      // Set the translation direction from the left stick.
      m_drivetrain.setTranslationDirection(
        this.calculateTranslationDirection(), isFieldCentric);

    // If we aren't changing translation direction, and we aren't commanding a significant speed in
    // either translation or rotation, just hold the modules in their current position.
    } else if (translationSpeed < HOLD_DIRECTION_TRANSLATION_THRESHOLD
        && Math.abs(rotationVelocity) < HOLD_DIRECTION_ROTATION_THRESHOLD) {
      m_drivetrain.holdDirection();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain.
    m_drivetrain.holdDirection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Manual drive ends when it is interrupted by another drivetrain command.
    return false;
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
    double x = m_controller.getRawAxis(0);
    double y = -m_controller.getRawAxis(1);
    
    // Calculate the angle.
    return Math.toDegrees(Math.atan2(y, x));
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
    double x = m_controller.getRawAxis(0);
    double y = -m_controller.getRawAxis(1);

    // Calculate the magnitude of the joystick position and use it as the threshold.
    return Math.sqrt(x * x + y * y) >= CHANGE_DIRECTION_THRESHOLD;
  }
}
