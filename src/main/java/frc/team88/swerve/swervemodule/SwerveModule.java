package frc.team88.swerve.swervemodule;

import java.util.Objects;

import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;
import frc.team88.swerve.swervemodule.motorsensor.PositionVelocitySensor;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;

/**
 * Represents a single swerve module that is composed of a PIDMotor for wheel
 * control, a PIDMotor for azimuth control, and a PositionVelocitySensor for
 * absolute azimuth sensing.
 */
public class SwerveModule {

    // The motor for controlling wheel velocity, in feet per second.
    private PIDMotor wheelControl;

    // The motor for contolling azimuth speed, in degrees per second.
    private PIDMotor azimuthControl;

    // Reads the absolute azimuth alsolute angle, in degrees.
    private PositionVelocitySensor absoluteAzimuthSensor;

    // PID controller for azimuth position
    private SyncPIDController azimuthPositionPID;

    /**
     * Constructor.
     * 
     * @param wheelControl                The PIDMotor which controls the wheel
     *                                    velocity, in feet per second
     * @param azimuthControl              The PIDMotor which controls azimuth
     *                                    velocity, in degrees per second
     * @param absoluteAzimuthSensor       The sensor for absolute azimuth angle, in
     *                                    degrees
     * @param azimuthPositionPIDConstants The PID constants for the azimuth position
     *                                    pid control. All PID gains are used
     */
    public SwerveModule(PIDMotor wheelControl, PIDMotor azimuthControl, PositionVelocitySensor absoluteAzimuthSensor,
            PIDPreferenceConstants azimuthPositionPIDConstants) {
        this.wheelControl = Objects.requireNonNull(wheelControl);
        this.azimuthControl = Objects.requireNonNull(azimuthControl);
        this.absoluteAzimuthSensor = Objects.requireNonNull(absoluteAzimuthSensor);
        this.azimuthPositionPID = new SyncPIDController(Objects.requireNonNull(azimuthPositionPIDConstants));
    }

    /**
     * Sets the wheel to the given velocity.
     * 
     * @param velocity The velocity to set, in feet per second
     */
    public void setWheelVelocity(double velocity) {
        this.wheelControl.setVelocity(velocity);
    }

    /**
     * Gets the current wheel velocity
     * 
     * @return The current wheel velocity, in feet per second
     */
    public double getWheelVelocity() {
        return this.wheelControl.getVelocity();
    }

    /**
     * Sets the azimuth to the given velocity.
     * 
     * @param velocity The velocity to set, in degrees per second
     */
    public void setAzimuthVelocity(double velocity) {
        this.azimuthControl.setVelocity(velocity);
    }

    /**
     * Gets the current azimuth velocity
     * 
     * @return The current azimuth velocity, in degrees per second
     */
    public double getAzimuthVelocity() {
        return this.azimuthControl.getVelocity();
    }

    /**
     * Sets the azimuth to the given position.
     * 
     * @param velocity The position to set, in degrees
     */
    public void setAzimuthPosition(double position) {
        this.setAzimuthVelocity(azimuthPositionPID.calculateOutput(this.absoluteAzimuthSensor.getPosition(), position));
    }

    /**
     * Gets the current azimuth position
     * 
     * @return The current azimuth position, in degrees
     */
    public double getAzimuthPosition() {
        return this.absoluteAzimuthSensor.getPosition();
    }

}