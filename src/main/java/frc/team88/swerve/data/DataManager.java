package frc.team88.swerve.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.data.representations.ChassisData;
import frc.team88.swerve.data.representations.GyroData;
import frc.team88.swerve.data.representations.SwerveModuleData;
import frc.team88.swerve.motion.SwerveChassis;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.tuning.TuningManager;
import frc.team88.swerve.util.RobotControllerWrapper;
import java.util.Objects;
import java.util.stream.Stream;

/** Handles the collection and logging/publishing of data. */
public class DataManager {

  // The overall configuration for the swerve drive.
  private final Configuration config;

  // The swerve chassis generating data.
  private final SwerveChassis chassis;

  // The tuning manager for the swerve drive.
  private final TuningManager tuningManager;

  // If the data should be published to NetworkTables.
  private boolean enableNetworkTablesPublishing = true;

  public DataManager(Configuration config, SwerveChassis chassis, TuningManager tuningManager) {
    this.config = Objects.requireNonNull(config);
    this.chassis = Objects.requireNonNull(chassis);
    this.tuningManager = tuningManager;
  }

  /** Publishes to NetworkTables, if enabled. */
  public void update() {
    if (!enableNetworkTablesPublishing) {
      return;
    }

    GyroData gyroData = new GyroData(this.config.getGyro());
    SwerveModuleData moduleData[] =
        Stream.of(this.config.getModules())
            .map(m -> new SwerveModuleData(m))
            .toArray(SwerveModuleData[]::new);
    ChassisData chassisData = new ChassisData(this.chassis);
    VelocityState targetState = this.chassis.getTargetState();
    VelocityState constrainedCommandState = this.chassis.getConstrainedCommandState();
    OdomState odometryState = this.chassis.getOdomState();

    if (enableNetworkTablesPublishing) {
      NetworkTable mainTable = NetworkTableInstance.getDefault().getTable("swerveLibrary");

      this.config.populateNetworkTable(mainTable.getSubTable("configuration"));
      this.tuningManager.populateNetworkTable(mainTable.getSubTable("tuning"));

      gyroData.populateNetworkTable(mainTable.getSubTable("gyro"));
      for (int idx = 0; idx < moduleData.length; idx++) {
        moduleData[idx].populateNetworkTable(
            mainTable.getSubTable("modules").getSubTable(Integer.toString(idx)));
      }
      chassisData.populateNetworkTable(mainTable.getSubTable("chassis"));
      targetState.populateNetworkTable(mainTable.getSubTable("targetState"));
      constrainedCommandState.populateNetworkTable(
          mainTable.getSubTable("constrainedCommandState"));
      odometryState.populateNetworkTable(mainTable.getSubTable("odometryState"));
      mainTable.getEntry("timestamp").setDouble(RobotControllerWrapper.getInstance().getFPGATime());
    }
  }

  /**
   * Is NetworkTables publishing enabled?
   *
   * @return True if NetworkTables publishing is enabled, false if it is disabled.
   */
  public boolean isEnableNetworkTablesPublishing() {
    return this.enableNetworkTablesPublishing;
  }

  /**
   * Is NetworkTables publishing enabled?
   *
   * @return True if NetworkTables publishing is enabled, false if it is disabled.
   */
  public boolean getEnableNetworkTablesPublishing() {
    return this.enableNetworkTablesPublishing;
  }

  /**
   * Set if NetworkTables publishing should be enabled.
   *
   * @param enable True if NetworkTables publishing should be enabled, false if it should be
   *     disabled.
   */
  public void setEnableNetworkTablesPublishing(boolean enable) {
    this.enableNetworkTablesPublishing = enable;
  }
}
