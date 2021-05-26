package frc.team88.swerve.data.representations;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.module.motor.SwerveMotor;
import java.util.Objects;

/** Represents data from a motor. */
public class MotorData implements NetworkTablePopulator {

  private final double velocity;
  private final double commandVelocity;
  private final double commandVoltage;
  private final double currentDraw;

  /**
   * Constructor.
   *
   * @param motor The motor to collect data from.
   */
  public MotorData(SwerveMotor motor) {
    Objects.requireNonNull(motor);
    this.velocity = motor.getVelocity();
    this.commandVelocity = motor.getCommandVelocity();
    this.commandVoltage = motor.getCommandVoltage();
    this.currentDraw = motor.getCurrentDraw();
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    table.getEntry("velocity").setDouble(this.velocity);
    table.getEntry("commandVelocity").setDouble(this.commandVelocity);
    table.getEntry("commandVoltage").setDouble(this.commandVoltage);
    table.getEntry("currentDraw").setDouble(this.currentDraw);
  }
}
