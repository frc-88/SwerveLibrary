// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.team88.swerve.SwerveController;

/**
 * It is tradition on team 88 for every subsystem to be accompanied by a haiku.
 *
 * <p>The drive subsystem <br>
 * contains the swerve controller <br>
 * and calls its methods.
 */
public class Drivetrain extends SubsystemBase {

  // The controller for the swerve drive.
  private SwerveController swerve;

  // Chooser for selecting the Joystick Control Style used
  private final SendableChooser<DriveControls> oiChooser = new SendableChooser<>();

  // Variables for control with different joystick setups
  private double xDirection;
  private double yDirection;
  private double rotationSpeed;
  private double translationSpeed;

  public enum DriveControls {
    SPLIT_TRIGGER("Split Joysticks with Triggers Turning"),
    SINGLE_TRIGGER("Single Joystick with Triggers Turning"),
    SINGLE_JOYSTICK("Single Joystick with Joystick X Turning"),
    TWO_JOYSTICK_GAS("2 Joysticks with Gas Pedal");

    private String displayName;

    DriveControls(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }
  }

  /** Constructor. */
  public Drivetrain() {
    this.swerve = new SwerveController(DriveConstants.SWERVE_CONFIG);
    this.setYaw(0.);

    // Creates a chooser dropdown with each of the driver control options
    oiChooser.addOption(
        DriveControls.SPLIT_TRIGGER.displayName, DriveControls.SPLIT_TRIGGER);
    oiChooser.addOption(
        DriveControls.SINGLE_TRIGGER.displayName, DriveControls.SINGLE_TRIGGER);
    oiChooser.addOption(
        DriveControls.SINGLE_JOYSTICK.displayName, DriveControls.SINGLE_JOYSTICK);
    oiChooser.setDefaultOption(
        DriveControls.TWO_JOYSTICK_GAS.displayName, DriveControls.TWO_JOYSTICK_GAS);
    SmartDashboard.putData("Drive Controls Chooser", oiChooser);
  }

  public void manualDrive(XboxController m_gamepad) {
    // Pass the correct joystick properties of whichever chooser option is selected.
    switch (oiChooser.getSelected()) {
      case TWO_JOYSTICK_GAS:
        xDirection = m_gamepad.getX(Hand.kLeft);
        yDirection = -m_gamepad.getY(Hand.kLeft);
        rotationSpeed = applyDeadband(m_gamepad.getX(Hand.kRight));
        translationSpeed = m_gamepad.getTriggerAxis(Hand.kRight);
      case SPLIT_TRIGGER:
        xDirection = m_gamepad.getX(Hand.kRight);
        yDirection = -m_gamepad.getY(Hand.kLeft);
        rotationSpeed =
            m_gamepad.getTriggerAxis(Hand.kLeft) - m_gamepad.getTriggerAxis(Hand.kRight);
        translationSpeed = applyDeadband(Math.max(xDirection, yDirection));
      case SINGLE_TRIGGER:
        xDirection = m_gamepad.getX(Hand.kLeft);
        yDirection = -m_gamepad.getY(Hand.kLeft);
        rotationSpeed =
            m_gamepad.getTriggerAxis(Hand.kLeft) - m_gamepad.getTriggerAxis(Hand.kRight);
        translationSpeed = applyDeadband(Math.max(xDirection, yDirection));
      case SINGLE_JOYSTICK:
        xDirection = m_gamepad.getX(Hand.kLeft);
        yDirection = -m_gamepad.getY(Hand.kLeft);
        rotationSpeed = applyDeadband(m_gamepad.getX(Hand.kRight));
        translationSpeed = applyDeadband(Math.max(xDirection, yDirection));
    }

    // If left bumper is pressed go into "Turtle Mode" for fine control both scale from % to fps
    if (m_gamepad.getBumper(Hand.kLeft)) {
      translationSpeed *= DriveConstants.TURTLE_SPEED;
    } else {
      translationSpeed *= DriveConstants.MAX_SPEED;
    }

    // Scales from % to degrees per second
    rotationSpeed *= DriveConstants.MAX_ROTATION;

    var translationDirection = calculateTranslationDirection(xDirection, yDirection);
    setVelocity(translationSpeed, rotationSpeed);
    setTranslationDirection(translationDirection, !m_gamepad.getBumper(Hand.kRight));
  }

  /** Updates the swerve controller. Should be called at the end of each program loop. */
  public void update() {
    this.swerve.update();
  }

  /**
   * Sets the current yaw to be read by the gyro.
   *
   * @param yaw The yaw to be set as the current heading of the robot.
   */
  public void setYaw(double yaw) {
    this.swerve.setGyroYaw(yaw);
  }

  /** Sets the translation and rotation speeds to zero and holds the module steering in place. */
  public void holdDirection() {
    this.swerve.holdDirection();
  }

  /**
   * Sets the translation speed and rotation velocity of the robot, without modifying the direction
   * of translation.
   *
   * @param translationSpeed The speed for translation, in feet per second.
   * @param rotationVelocity The velocity for rotation, in degrees per second.
   */
  public void setVelocity(double translationSpeed, double rotationVelocity) {
    this.swerve.setVelocity(translationSpeed, rotationVelocity);
  }

  /**
   * Sets the direction of translation as either a field-centric or robot-centric angle.
   *
   * @param translationDirection The direction to translation, in degrees increasing
   *     counterclockwise with forwards at 0.
   * @param fieldCentric If true, the direction will be intrepretted relative to the gyro's zero
   *     point. If false, it will be interpretted relative to the front of the robot.
   */
  public void setTranslationDirection(double translationDirection, boolean fieldCentric) {
    this.swerve.setTranslationDirection(translationDirection, fieldCentric);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update() is not called here because it would occur before commands are run.
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double applyDeadband(double value) {
    if (Math.abs(value) < OIConstants.JOYSTICK_DEADBAND) {
      return 0;
    } else {
      return value;
    }
  }

  /**
   * Calculates the angle of translation set by the left stick.
   *
   * @return The angle of translation, in degrees. 0 corresponds to forwards, and positive
   *     corresponds to counterclockwise.
   */
  private double calculateTranslationDirection(double x, double y) {
    // Calculate the angle.
    // Swapping x/y and inverting y because our coordinate system has +x forwards and -y right
    return Math.toDegrees(Math.atan2(x, -y));
  }
}
