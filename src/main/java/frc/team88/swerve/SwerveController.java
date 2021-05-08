package frc.team88.swerve;

import java.util.Map;
import java.util.Objects;

import com.ctre.phoenix.CANifier;

import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.motion.state.OdomState;

/**
 * The high-level API for controlling a swerve drive with this library.
 */
public class SwerveController {

    // The configuration data
    private final Configuration config;
    
    /**
     * Constructs the SwerveController using the given toml config.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     */
    public SwerveController(final String configPath) {
        Objects.requireNonNull(configPath);
        this.config = new Configuration(configPath);
    }

    /**
     * Constructs the SwerveController using the given toml config.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     * @param gyro The gyro to use instead of instantiating one from the
     *             config.
     */
    public SwerveController(final String configPath, SwerveGyro gyro) {
        Objects.requireNonNull(configPath);
        this.config = new Configuration(configPath, gyro);
    }

    public void update() {

    }
    
    public void setVelocity(double translationDirection, double translationSpeed, double rotationVelocity, boolean fieldCentric) {

    }

    public void holdDirection() {

    }

    public void setCenterOfRotation(double x, double y) {

    }
 
    public VelocityState getTargetVelocity() {

    }
    
    public VelocityState getConstrainedVelocity() {

    }
    
    public OdomState getOdometry() {

    }
    
    /**
     * Gets the gyro being used by this swerve controller.
     * 
     * @return The gyro object.
     */
    public SwerveGyro getGyro() {
        return this.config.getGyro();
    }

    /**
     * Gets the swerve modules on this swerve.
     * 
     * @return An array of the swerve module objects, in no particular order.
     */
    public SwerveModule[] getModules() {
        return this.config.getModules();
    }

    /**
     * Gets the CANifiers used by this swerve for sensing, if any.
     * 
     * @return A mapping from can IDs to Canifiers with that ID.
     */
    public Map<Integer, CANifier> getCanifiers() {
        return this.config.getCanifiers();
    }

    public void enableLoggingOnRIO() {

    }

    public void disableLoggingOnRIO() {

    }

    public void enableNetworkTablesPublishing() {

    }

    public void disableNetworkTablesPublishing() {
        
    }
}
