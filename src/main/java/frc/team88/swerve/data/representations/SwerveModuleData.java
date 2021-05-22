package frc.team88.swerve.data.representations;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.module.SwerveModule;
import java.util.Objects;

/** Represents data from a swerve module */
public class SwerveModuleData implements NetworkTablePopulator {

  private final MotorData motor0;
  private final MotorData motor1;

  private final double wheelVelocity;
  private final double azimuthPosition;
  private final double azimuthVelocity;

  private final double commandWheelVelocity;
  private final double commandAzimuthPosition;
  private final double commandAzimuthVelocity;

  private final double targetWheelVelocity;
  private final double targetAzimuthPosition;
  private final double targetAzimuthVelocity;

  private final double locationX;
  private final double locationY;

  public SwerveModuleData(SwerveModule module) {
    Objects.requireNonNull(module);

    this.motor0 = new MotorData(module.getMotors()[0]);
    this.motor1 = new MotorData(module.getMotors()[1]);

    this.wheelVelocity = module.getWheelVelocity();
    this.azimuthPosition = module.getAzimuthPosition().asDouble();
    this.azimuthVelocity = module.getAzimuthVelocity();

    this.commandWheelVelocity = module.getCommandedWheelVelocity();
    this.commandAzimuthPosition = module.getCommandedAzimuthPosition().asDouble();
    this.commandAzimuthVelocity = module.getCommandedAzimuthVelocity();

    this.targetWheelVelocity = module.getTargetWheelVelocity();
    this.targetAzimuthPosition = module.getTargetAzimuthPosition().asDouble();
    this.targetAzimuthVelocity = module.getTargetAzimuthVelocity();

    this.locationX = module.getLocation().getX();
    this.locationY = module.getLocation().getY();
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    this.motor0.populateNetworkTable(table.getSubTable("motor0"));
    this.motor1.populateNetworkTable(table.getSubTable("motor1"));

    table.getEntry("wheelVelocity").setDouble(this.wheelVelocity);
    table.getEntry("azimuthPosition").setDouble(this.azimuthPosition);
    table.getEntry("azimuthVelocity").setDouble(this.azimuthVelocity);

    table.getEntry("commandWheelVelocity").setDouble(this.commandWheelVelocity);
    table.getEntry("commandAzimuthPosition").setDouble(this.commandAzimuthPosition);
    table.getEntry("commandAzimuthVelocity").setDouble(this.commandAzimuthVelocity);

    table.getEntry("targetWheelVelocity").setDouble(this.targetWheelVelocity);
    table.getEntry("targetAzimuthPosition").setDouble(this.targetAzimuthPosition);
    table.getEntry("targetAzimuthVelocity").setDouble(this.targetAzimuthVelocity);

    table.getEntry("locationX").setDouble(this.locationX);
    table.getEntry("locationY").setDouble(this.locationY);
  }
}
