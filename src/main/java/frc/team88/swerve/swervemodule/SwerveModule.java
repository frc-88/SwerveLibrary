package frc.team88.swerve.swervemodule;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collector;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.javatuples.Pair;

import frc.team88.swerve.configuration.SwerveModuleConfiguration;
import frc.team88.swerve.swervemodule.motorsensor.PIDMotor;
import frc.team88.swerve.swervemodule.motorsensor.PositionSensor;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.util.TrapezoidalProfileController;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.PIDPreferenceConstants;

/**
 * Represents a single swerve module that is composed of a PIDMotor for wheel
 * control, a PIDMotor for azimuth control, and a PositionVelocitySensor for
 * absolute azimuth sensing.
 */
public class SwerveModule {

    // The motors on this module, with units of rotations per second.
    private final PIDMotor[] motors;

    // Reads the absolute azimuth alsolute angle, in degrees.
    private final PositionSensor azimuthSensor;

    // The configuration data for this module.
    private final SwerveModuleConfiguration config;

    // PID controller for azimuth position.
    private TrapezoidalProfileController azimuthPositionController;

    // True if the wheel is currently reversed, false otherwise.
    private boolean isWheelReversed = false;

    // The conversion factor from azimuth rotations to degrees.
    private final double azimuthRotationsToDegrees = 360.;

    // The conversion factor from wheel rotations to feet.
    private final double wheelRotationsToFeet;

    /**
     * Constructor.
     * 
     * @param motors The 2 motors on this module, with units of rotations per
     *               second.
     * @param azimuthSensor The sensor for absolute azimuth angle, in degrees.
     * @param config The configuration data for this module.
     */
    public SwerveModule(final PIDMotor[] motors, final PositionSensor azimuthSensor, final SwerveModuleConfiguration config) {
        if (motors.length != 2) {
            throw new IllegalArgumentException("Must suppy exactly 2 modules");
        }
        this.motors = Objects.requireNonNull(motors);
        this.azimuthSensor = Objects.requireNonNull(azimuthSensor);
        this.config = Objects.requireNonNull(config);

        SyncPIDController azimuthPID = new SyncPIDController(config.getAzimuthControllerConfig().getPIDConfig());
        this.azimuthPositionController = new TrapezoidalProfileController(config.getAzimuthControllerConfig().getMaxSpeed(),
                config.getAzimuthControllerConfig().getMaxAcceleration(), azimuthPID);
        this.azimuthPositionController.reset(this.getAzimuthPosition().asDouble());
        
        this.wheelRotationsToFeet = (config.getWheelDiameter() / 12.) * Math.PI;
    }

    /**
     * Sets the wheel speed and azimuth position. The azimuth velocity is
     * assumed to be 0.
     * 
     * @param wheelSpeed The wheel speed to set, in feet per second.
     * @param azimuthPosition The azimuth position to set, in degrees.
     */
    public void set(double wheelSpeed, WrappedAngle azimuthPosition) {
        set(wheelSpeed, azimuthPosition, 0);
    }

    /**
     * Sets the wheel speed and azimuth position/velocity.
     * 
     * @param wheelSpeed The wheel speed to set, in feet per second.
     * @param azimuthPosition The azimuth position to set, ignoring potential
     *                        wheel reversal, in degrees.
     * @param azimuthVelocity The azimuth velocity to target when the position
     *                        is reached, in degrees per second.
     */
    public void set(double wheelSpeed, WrappedAngle azimuthPosition, double azimuthVelocity) {
        if (wheelSpeed < 0) {
            throw new IllegalAccessError("Wheel speed cannot be negative");
        }

        // Calculate the actual sensor value to target for the azimuth, and
        // flip the wheel direction if necessary.
        Pair<Double, Boolean> distanceAndFlip = this.getAzimuthPosition().getSmallestDifferenceWithHalfAngle(azimuthPosition,
                this.getAzimuthWrapBias());
        if (distanceAndFlip.getValue1()) {
            this.isWheelReversed = !this.isWheelReversed;
        }
        double unwrappedAzimuthAngle = this.azimuthSensor.getPosition() + distanceAndFlip.getValue0();

        // Reverse the wheel if applicable.
        if (this.isWheelReversed) {
            wheelSpeed *= -1;
        }

        // Get the azimuth velocity to command from the trapezoidal profile controller.
        this.azimuthPositionController.setTargetVelocity(azimuthVelocity);
        this.azimuthPositionController.setTargetPosition(unwrappedAzimuthAngle);
        double commandAzimuthVelocity = azimuthPositionController.calculateCommandVelocity(
                this.azimuthSensor.getPosition(), this.getAzimuthVelocity());

        this.setRawWheelVelocities(wheelSpeed, commandAzimuthVelocity);
    }

    /**
     * Sets the raw velocity values for the wheel and azimuth.
     * 
     * @param wheelVelocity The wheel velocity to set, in feet per second.
     * @param azimuthVelocity The azimuth velocity to set, in degrees per second.
     */
    public void setRawWheelVelocities(double wheelVelocity, double azimuthVelocity) {
        double[] rotationsPerSecondVelocities = new double[]{azimuthVelocity / azimuthRotationsToDegrees, wheelVelocity / wheelRotationsToFeet};
        double[] motorVelocities = this.getDifferentialInputs(rotationsPerSecondVelocities);
        for (int motorIndex = 0; motorIndex < this.motors.length; motorIndex++) {
            motors[motorIndex].setVelocity(motorVelocities[motorIndex]);
        }
    }

    /**
     * Gets the current wheel velocity.
     * 
     * @return The current wheel velocity, in feet per second.
     */
    public double getWheelSpeed() {
        return this.getDifferentialOutputs(Stream.of(motors).mapToDouble(PIDMotor::getVelocity).toArray())[1] * wheelRotationsToFeet;
    }

    /**
     * Gets the current azimuth position.
     * 
     * @return The current azimuth position, in degrees.
     */
    public WrappedAngle getAzimuthPosition() {
        WrappedAngle azimuth = new WrappedAngle(this.azimuthSensor.getPosition());
        if (isWheelReversed) {
            azimuth = azimuth.plus(180.);
        }
        return azimuth;
    }

    /**
     * Gets the current azimuth velocity.
     * 
     * @return The current azimuth velocity, in degrees per second.
     */
    public double getAzimuthVelocity() {
        return this.getDifferentialOutputs(Stream.of(motors).mapToDouble(PIDMotor::getVelocity).toArray())[0] * azimuthRotationsToDegrees;
    }

    /**
     * Sets the azimuth to the given position.
     * 
     * @param position The position to set, in degrees
     */
    public void setAzimuthPosition(WrappedAngle position) {
        Pair<Double, Boolean> distanceAndFlip = this.getAzimuthPosition().getSmallestDifferenceWithHalfAngle(position,
                this.getAzimuthWrapBias());
        if (distanceAndFlip.getValue1()) {
            this.isWheelReversed = !this.isWheelReversed;
            this.setWheelSpeed(this.getWheelSpeed());
        }
        double unwrappedAngle = this.absoluteAzimuthSensor.getPosition() + distanceAndFlip.getValue0();
        this.azimuthPositionController.setTargetVelocity(0);
        this.azimuthPositionController.setTargetPosition(unwrappedAngle);
        this.setAzimuthVelocity(azimuthPositionController.calculateCommandVelocity(
                this.absoluteAzimuthSensor.getPosition(), this.azimuthControl.getVelocity()));
    }

    /**
     * Gets the current commanded azimuth position from the trapezoidal profile.
     * 
     * @return The current commanded azimuth position.
     */
    public WrappedAngle getCommandedAzimuthPosition() {
        return new WrappedAngle(this.azimuthPositionController.getLastCommandedPosition());
    }

    /**
     * Sets the location of this module.
     * 
     * @param location A position vector from the robot's origin to the location of
     *                 this module
     */
    public void setLocation(Vector2D location) {
        this.location = location;
    }

    /**
     * Gets the location of this module.
     * 
     * @return A position vector from the robot's origin to the location of
     *               this module
     */
    public Vector2D getLocation() {
        return Objects.requireNonNull(this.location);
    }

    /**
     * Sets the direction current switching mode.
     * 
     * @param mode The mode to set
     */
    public void setSwitchingMode(SwitchingMode mode) {
        this.switchingMode = mode;
    }

    /**
     * Gets the direction current switching mode.
     * 
     * @return The switching mode
     */
    public SwitchingMode getSwitchingMode() {
        return this.switchingMode;
    }

    /**
     * Puts the given differential inputs through the forwards matrix.
     * 
     * @param differentialInputs A length 2 array containing the motor values.
     * @return The differential outputs as a length 2 array with the azimuth
     *         value followed by the wheel value.
     */
    private double[] getDifferentialOutputs(double[] differentialInputs) {
        return this.config.getForwardMatrix().operate(differentialInputs);
    }

    /**
     * Puts the given differential outputs through the inverse matrix.
     * 
     * @param differentialOutputs A length 2 array containing the azimuth
     *                            value followed by the wheel value.
     * @return The differential inputs as a length 2 array.
     */
    private double[] getDifferentialInputs(double[] differentialOutputs) {
        return this.config.getInverseMatrix().operate(differentialOutputs);
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
            if (this.getAzimuthVelocity() > 30) {
                return 180;
            } else if (currentSpeed < 2.5) {
                return 90;
            } else if (currentSpeed < 5.5) {
                return 120;
            } else {
                return 180;
            }
        }
        throw new IllegalStateException("Switching mode is not supported");
    }

    /**
     * Limits the given wheel speed in order to leave enough headroom for the
     * azimuth to have full control.
     * 
     * @param speed The initial wheel speed.
     * @return The limited wheel speed.
     */
    private double limitWheelSpeedForAzimuth(double speed) {
        double limit = this.maxWheelSpeed - this.getAzimuthVelocity() * 0.02;
        return Math.min(limit, Math.max(-limit, speed));
    }

}
