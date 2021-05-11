package frc.team88.swerve.data.representations;

import java.util.Objects;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.gyro.SwerveGyro;

/**
 * Represents data from a gyroscope.
 */
public class GyroData implements NetworkTablePopulator {

    private final double yaw;
    private final double yawRate;

    public GyroData(SwerveGyro gyro) {
        Objects.requireNonNull(gyro);
        this.yaw = gyro.getYaw();
        this.yawRate = gyro.getYawRate();
    }

    @Override
    public void populateNetworkTable(NetworkTable table) {
        table.getEntry("yaw").setDouble(this.yaw);
        table.getEntry("yawRate").setDouble(this.yawRate);
    }
    
}
