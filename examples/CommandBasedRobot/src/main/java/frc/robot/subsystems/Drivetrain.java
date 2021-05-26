// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  // The path to the swerve config file in the deploy directory.
  private static final String SWERVE_CONFIG = "swerve.toml";

  /** Constructor. */
  public Drivetrain() {
    this.swerve = new SwerveController(SWERVE_CONFIG);
    this.setYaw(0.);
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
}
