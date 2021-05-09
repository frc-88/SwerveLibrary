package frc.team88.swerve;

import java.util.Map;
import java.util.Objects;

import com.ctre.phoenix.CANifier;

import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.motion.SwerveChassis;
import frc.team88.swerve.motion.state.OdomState;

/**
 * The high-level API for controlling a swerve drive with this library.
 */
public class SwerveController {

    // The configuration data
    private final Configuration config;

    // The swerve chassis object being controlled
    private final SwerveChassis chassis;
    
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
        this.chassis = new SwerveChassis(this.config);
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
        this.chassis = new SwerveChassis(this.config);
    }

    public void update() {
        this.chassis.update();
    }
    
    public void setVelocity(double translationDirection, double translationSpeed, double rotationVelocity, boolean fieldCentric) {
        VelocityState velocityState = this.getTargetVelocity();
        velocityState = velocityState.changeTranslationDirection(translationDirection).changeTranslationSpeed(translationSpeed).changeRotationVelocity(rotationVelocity).changeIsFieldCentric(fieldCentric);
        this.chassis.setTargetState(velocityState);
        this.chassis.holdAzimuths(false);
    }

    public void holdDirection() {
        VelocityState velocityState = this.getTargetVelocity();
        velocityState = velocityState.changeTranslationSpeed(0).changeRotationVelocity(0);
        this.chassis.setTargetState(velocityState);
        this.chassis.holdAzimuths(true);
    }

    public void setCenterOfRotation(double x, double y) {
        VelocityState velocityState = this.getTargetVelocity();
        velocityState = velocityState.changeCenterOfRotation(x, y);
        this.chassis.setTargetState(velocityState);
    }

    public void setGyroYaw(double yaw) {
        this.getGyro().calibrateYaw(yaw);
    }
 
    public VelocityState getTargetVelocity() {
        return this.chassis.getTargetState();
    }
    
    public VelocityState getConstrainedVelocity() {
        return this.chassis.getConstrainedCommandState();
    }
    
    public OdomState getOdometry() {
        return this.chassis.getOdomState();
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
