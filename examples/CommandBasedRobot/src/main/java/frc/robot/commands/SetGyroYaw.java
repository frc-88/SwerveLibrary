// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Sets the current robot heading. */
public class SetGyroYaw extends InstantCommand {

  // The drivetrain subsystem.
  private final Drivetrain m_drivetrain;

  // The yaw to set when this command is run.
  private final double m_yaw;

  /**
   * Constructor.
   *
   * @param drivetrain The drivetrain subsystem used by this command.
   * @param yaw The yaw to set as the current heading when this command is run, increasing
   *     counterclockwise.
   */
  public SetGyroYaw(Drivetrain drivetrain, double yaw) {
    m_drivetrain = drivetrain;
    m_yaw = yaw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setYaw(m_yaw);
  }
}
