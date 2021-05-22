package frc.team88.swerve.data.representations;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.motion.SwerveChassis;

/** Represents data about the swerve chassis. */
public class ChassisData implements NetworkTablePopulator {
  private final boolean inHoldAzimuthMode;

  public ChassisData(SwerveChassis chassis) {
    this.inHoldAzimuthMode = chassis.inHoldAzimuthMode();
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    table.getEntry("holdAzimuthMode").setBoolean(this.inHoldAzimuthMode);
  }
}
