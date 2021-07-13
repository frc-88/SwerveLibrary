package frc.team88.swerve.data;

import edu.wpi.first.networktables.EntryListenerFlags;
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

  // Callback object for NetworkTable entry listener
  private final NetworkTableCommandListener ntCommandListener;

  // Root table for all Swerve Library data;
  NetworkTable mainTable;

  public DataManager(Configuration config, SwerveChassis chassis, TuningManager tuningManager) {
    this.config = Objects.requireNonNull(config);
    this.chassis = Objects.requireNonNull(chassis);
    this.tuningManager = tuningManager;
    this.ntCommandListener = new NetworkTableCommandListener();
    mainTable = NetworkTableInstance.getDefault().getTable("swerveLibrary");
    setupCallbacks();
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
   * Sets up callbacks for commands sent via network tables. Only listens for
   * swerveLibrary/commands/timestamp. The callback assumes if this entry has updated, then the
   * other command entries have update too
   */
  private void setupCallbacks() {
    NetworkTable commandTable = mainTable.getSubTable("commands");
    ntCommandListener.setTable(commandTable);
    commandTable.addEntryListener("timestamp", ntCommandListener, EntryListenerFlags.kUpdate);
  }

  /**
   * Gets the last command sent via network tables
   *
   * @return VelocityState
   */
  public VelocityState getNetworkTableCommand() {
    return ntCommandListener.getCommand();
  }

  /**
   * Gets the time at which the last command was sent via network tables
   *
   * @return FPGA time in microseconds
   */
  public long getNetworkTableCommandTime() {
    return ntCommandListener.getCommandTime();
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
