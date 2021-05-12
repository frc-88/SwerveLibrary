package frc.team88.swerve.data;

import java.util.Objects;
import java.util.stream.Stream;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.data.representations.GyroData;
import frc.team88.swerve.data.representations.SwerveModuleData;
import frc.team88.swerve.motion.SwerveChassis;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.RobotControllerWrapper;

/**
 * Handles the collection and logging/publishing of data.
 */
public class DataManager {
    
    // The overall configuration for the swerve drive.
    private final Configuration config;

    // The swerve chassis generating data.
    private final SwerveChassis chassis;

    // If the data should be logged to a JSONL file.
    private boolean enableDataLogging = true;

    // If the data should be published to NetworkTables.
    private boolean enableNetworkTablesPublishing = true;

    public DataManager(Configuration config, SwerveChassis chassis) {
        this.config = Objects.requireNonNull(config);
        this.chassis = Objects.requireNonNull(chassis);
    }

    /**
     * Updates JSONL logs and publishes to NetworkTables, if enabled.
     */
    public void update() {
        if (!enableDataLogging && !enableNetworkTablesPublishing) {
            return;
        }

        GyroData gyroData = new GyroData(this.config.getGyro());
        SwerveModuleData moduleData[] = (SwerveModuleData[])Stream.of(this.config.getModules()).map(m -> new SwerveModuleData(m)).toArray();
        VelocityState targetState = this.chassis.getTargetState();
        VelocityState constrainedCommandState = this.chassis.getConstrainedCommandState();
        OdomState odometryState = this.chassis.getOdomState();

        if (enableDataLogging) {
            DataLogger.getInstance().addData("gyro", gyroData);
            DataLogger.getInstance().addData("modules", moduleData);
            DataLogger.getInstance().addData("targetState", targetState);
            DataLogger.getInstance().addData("constrainedCommandState", constrainedCommandState);
            DataLogger.getInstance().addData("odometryState", odometryState);
            DataLogger.getInstance().logData();
        }

        if (enableNetworkTablesPublishing) {
            NetworkTable mainTable = NetworkTableInstance.getDefault().getTable("swerveLibrary");
            gyroData.populateNetworkTable(mainTable.getSubTable("gyro"));
            for (int idx = 0; idx < moduleData.length; idx++) {
                moduleData[idx].populateNetworkTable(mainTable.getSubTable("modules").getSubTable(Integer.toString(idx)));
            }
            targetState.populateNetworkTable(mainTable.getSubTable("targetState"));
            constrainedCommandState.populateNetworkTable(mainTable.getSubTable("constrainedCommandState"));
            odometryState.populateNetworkTable(mainTable.getSubTable("odometryState"));
            mainTable.getEntry("timestamp").setDouble(RobotControllerWrapper.getInstance().getFPGATime());
        }
    }

    /**
     * Is JSONL data logging enabled?
     * 
     * @return True if JSONL data logging is enabled, false if it is disabled.
     */
    public boolean isEnableDataLogging() {
        return this.enableDataLogging;
    }

    /**
     * Is JSONL data logging enabled?
     * 
     * @return True if JSONL data logging is enabled, false if it is disabled.
     */
    public boolean getEnableDataLogging() {
        return this.enableDataLogging;
    }

    /**
     * Set if JSONL data logging should be enabled.
     * 
     * @param enable True if JSONL data logging should be enabled, false if it
     *               should be disabled.
     */
    public void setEnableDataLogging(boolean enable) {
        this.enableDataLogging = enable;
    }

    /**
     * Is NetworkTables publishing enabled?
     * 
     * @return True if NetworkTables publishing is enabled, false if it is
     *         disabled.
     */
    public boolean isEnableNetworkTablesPublishing() {
        return this.enableNetworkTablesPublishing;
    }

    /**
     * Is NetworkTables publishing enabled?
     * 
     * @return True if NetworkTables publishing is enabled, false if it is
     *         disabled.
     */
    public boolean getEnableNetworkTablesPublishing() {
        return this.enableNetworkTablesPublishing;
    }

    /**
     * Set if NetworkTables publishing should be enabled.
     * 
     * @param enable True if NetworkTables publishing should be enabled, false
     *               if it should be disabled.
     */
    public void setEnableNetworkTablesPublishing(boolean enable) {
        this.enableNetworkTablesPublishing = enable;
    }

}
