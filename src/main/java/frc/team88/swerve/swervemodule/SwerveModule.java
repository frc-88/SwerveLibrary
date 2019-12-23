package frc.team88.swerve.swervemodule;

import java.util.Objects;

import org.javatuples.Pair;

import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;
import frc.team88.swerve.swervemodule.motorsensor.PositionVelocitySensor;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;

/**
 * Represents a single swerve module that is composed of a PIDMotor for wheel
 * control, a PIDMotor for azimuth control, and a PositionVelocitySensor for
 * absolute azimuth sensing.
 */
public class SwerveModule {

    /**
     * Enumerates the possible modes for deciding when to change wheel
     * direction vs. rotating more.
     */
    public enum SwitchingMode {
        /**
         * Never switch wheel direction.
         */
        kNeverSwitch,
        /**
         * Always switch wheel direction if it will reduce rotation.
         */
        kAlwaysSwitch,
        /** 
         * Choose when switch depending on how much rotation would be saved 
         * and how fast the wheel is moving.
         */
        kSmart
    }

    // The motor for controlling wheel velocity, in feet per second.
    private PIDMotor wheelControl;

    // The motor for contolling azimuth speed, in degrees per second.
    private PIDMotor azimuthControl;

    // Reads the absolute azimuth alsolute angle, in degrees.
    private PositionVelocitySensor absoluteAzimuthSensor;

    // PID controller for azimuth position.
    private SyncPIDController azimuthPositionPID;

    // The location of this module relative to the robot's origin.
    private Vector2D location;

    // True if the wheel is currently reversed, false otherwise.
    private boolean isWheelReversed = false;

    // The switching mode being used.
    private SwitchingMode switchingMode = SwitchingMode.kSmart;

    /**
     * Constructor.
     * 
     * @param wheelControl
     *                                        The PIDMotor which controls the wheel
     *                                        velocity, in feet per second
     * @param azimuthControl
     *                                        The PIDMotor which controls azimuth
     *                                        velocity, in degrees per second
     * @param absoluteAzimuthSensor
     *                                        The sensor for absolute azimuth angle,
     *                                        in degrees
     * @param azimuthPositionPIDConstants
     *                                        The PID constants for the azimuth
     *                                        position pid control. All PID gains
     *                                        are used
     */
    public SwerveModule(PIDMotor wheelControl, PIDMotor azimuthControl, PositionVelocitySensor absoluteAzimuthSensor,
            PIDPreferenceConstants azimuthPositionPIDConstants) {
        this.wheelControl = Objects.requireNonNull(wheelControl);
        this.azimuthControl = Objects.requireNonNull(azimuthControl);
        this.absoluteAzimuthSensor = Objects.requireNonNull(absoluteAzimuthSensor);
        this.azimuthPositionPID = new SyncPIDController(Objects.requireNonNull(azimuthPositionPIDConstants));
    }

    /**
     * Sets the wheel to the given speed.
     * 
     * @param speed
     *                  The speed to set, in feet per second
     */
    public void setWheelSpeed(double speed) {
        if (speed < 0) {
            throw new IllegalAccessError("Wheel speed cannot be negative");
        }
        if (isWheelReversed) {
            speed *= -1;
        }
        this.wheelControl.setVelocity(speed);
    }

    /**
     * Gets the current wheel velocity
     * 
     * @return The current wheel velocity, in feet per second
     */
    public double getWheelSpeed() {
        return Math.abs(this.wheelControl.getVelocity());
    }

    /**
     * Sets the azimuth to the given velocity.
     * 
     * @param velocity
     *                     The velocity to set, in degrees per second
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
     * @param position
     *                     The position to set, in degrees
     */
    public void setAzimuthPosition(WrappedAngle position) {
        Pair<Double, Boolean> distanceAndFlip = this.getAzimuthPosition().getSmallestDifferenceWithHalfAngle(position,
                this.getAzimuthWrapBias());
        if (distanceAndFlip.getValue1()) {
            this.isWheelReversed = !this.isWheelReversed;
            this.setWheelSpeed(this.getWheelSpeed());
        }
        double unwrappedAngle = this.absoluteAzimuthSensor.getPosition() + distanceAndFlip.getValue0();
        this.setAzimuthVelocity(
                azimuthPositionPID.calculateOutput(this.absoluteAzimuthSensor.getPosition(), unwrappedAngle));
    }

    /**
     * Gets the current azimuth position
     * 
     * @return The current azimuth position, in degrees
     */
    public WrappedAngle getAzimuthPosition() {
        WrappedAngle azimuth = new WrappedAngle(this.absoluteAzimuthSensor.getPosition());
        if (isWheelReversed) {
            azimuth = azimuth.plus(180.);
        }
        return azimuth;
    }

    /**
     * Sets the location of this module.
     * 
     * @param location
     *                     A position vector from the robot's origin to the location
     *                     of this module
     */
    public void setLocation(Vector2D location) {
        this.location = location;
    }

    /**
     * Gets the location of this module.
     * 
     * @param return A position vector from the robot's origin to the location
     *                     of this module
     */
    public Vector2D getLocation() {
        return Objects.requireNonNull(this.location);
    }
    
    /**
     * Sets the direction current switching mode.
     * @param mode The mode to set
     */
    public void setSwitchingMode(SwitchingMode mode) {
        this.switchingMode = mode;
    }

    /**
     * Gets the direction current switching mode.
     * @return The switching mode
     */
    public SwitchingMode getSwitchingMode() {
        return this.switchingMode;
    }

    /**
     * Gets the curerent biasTo360 to use for determing how to get to the next
     * angle, depending on the swithing mode.
     * 
     * @return The bias to use
     */
    private double getAzimuthWrapBias() {
        switch (getSwitchingMode()) {
        case kAlwaysSwitch:
            return 90;
        case kNeverSwitch:
            return 180;
        case kSmart:
            // TODO: This should be fully paramaterizable when we have configurations
            double currentSpeed = getWheelSpeed();
            if (currentSpeed < 2.5) {
                return 90;
            } else if (currentSpeed < 5.5) {
                return 120;
            } else {
                return 180;
            }
        }
        throw new IllegalStateException("Switching mode is not supported");
    }

}