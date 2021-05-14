package frc.team88.swerve.module;

import java.util.Objects;
import java.util.stream.Stream;

import org.apache.commons.math3.linear.RealMatrix;
import org.javatuples.Pair;

import frc.team88.swerve.configuration.SwerveModuleConfiguration;
import frc.team88.swerve.module.sensor.PositionSensor;
import frc.team88.swerve.module.motor.SwerveMotor;
import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.util.TrapezoidalProfileController;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Represents a single swerve module that is composed of a PIDMotor for wheel
 * control, a PIDMotor for azimuth control, and a PositionVelocitySensor for
 * absolute azimuth sensing.
 */
public class SwerveModule {

    // The motors on this module.
    private final SwerveMotor[] motors;

    // Reads the absolute azimuth alsolute angle, in degrees.
    private final PositionSensor azimuthSensor;

    // The configuration data for this module.
    private final SwerveModuleConfiguration config;

    // Controller for azimuth position.
    private TrapezoidalProfileController azimuthPositionController;

    // Controller for the wheel velocity.
    private SyncPIDController wheelVelocityController;

    // True if the wheel is currently reversed, false otherwise.
    private boolean isWheelReversed = false;

    // The conversion factor from azimuth rotations to degrees.
    private final double azimuthRotationsToDegrees = 360.;

    // The conversion factor from wheel rotations to feet.
    private final double wheelRotationsToFeet;

    // The last wheel velocity commanded by the controller.
    private double commandedWheelVelocity = 0;

    // The last wheel velocity set as a target for the controller.
    private double targetWheelVelocity = 0;

    /**
     * Constructor.
     * 
     * @param motors The 2 motors on this module, with units of rotations per
     *               second.
     * @param azimuthSensor The sensor for absolute azimuth angle, in degrees.
     * @param config The configuration data for this module.
     */
    public SwerveModule(final SwerveMotor[] motors, final PositionSensor azimuthSensor, final SwerveModuleConfiguration config) {
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

        this.wheelVelocityController = new SyncPIDController(config.getWheelControllerConfig());
        
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
        double unwrappedAzimuthAngle = this.azimuthSensor.getPosition() + distanceAndFlip.getValue0();

        // Reverse the wheel if applicable.
        this.targetWheelVelocity = distanceAndFlip.getValue1() ? -wheelSpeed : wheelSpeed;

        // Get the azimuth velocity to command from the trapezoidal profile controller.
        this.azimuthPositionController.setTargetVelocity(azimuthVelocity);
        this.azimuthPositionController.setTargetPosition(unwrappedAzimuthAngle);
        double commandAzimuthVelocity = azimuthPositionController.calculateCommandVelocity(
                this.azimuthSensor.getPosition(), this.getAzimuthVelocity());

        // Apply the pid to the wheel velocity.
        this.commandedWheelVelocity = this.targetWheelVelocity + this.wheelVelocityController.calculateOutput(this.getWheelVelocity(), this.targetWheelVelocity);

        this.setRawWheelVelocities(this.commandedWheelVelocity, commandAzimuthVelocity);

        // Set if the wheel direction is flipped
        if (distanceAndFlip.getValue1()) {
            this.isWheelReversed = !this.isWheelReversed;
        }
    }

    /**
     * Sets the raw velocity values for the wheel and azimuth.
     * 
     * @param wheelVelocity The wheel velocity to set, in feet per second.
     * @param azimuthVelocity The azimuth velocity to set, in degrees per second.
     */
    public void setRawWheelVelocities(double wheelVelocity, double azimuthVelocity) {
        double[] rotationsPerSecondVelocities = new double[]{azimuthVelocity / azimuthRotationsToDegrees, wheelVelocity / wheelRotationsToFeet};

        rotationsPerSecondVelocities[1] = this.reduceWheelVelocityForAzimuth(rotationsPerSecondVelocities[1], rotationsPerSecondVelocities[0]);

        double[] motorVelocities = this.getDifferentialInputs(rotationsPerSecondVelocities);
        for (int motorIndex = 0; motorIndex < this.motors.length; motorIndex++) {
            motors[motorIndex].setVelocity(motorVelocities[motorIndex]);
        }
    }

    /**
     * Sets both motors on this module to coast mode.
     */
    public void setCoast() {
        Stream.of(this.motors).forEach(m -> m.setCoast());
    }

    /**
     * Sets both motors on this module to brake mode.
     */
    public void setBrake() {
        Stream.of(this.motors).forEach(m -> m.setCoast());
    }

    /**
     * Gets the current velocity of the wheel.
     * 
     * @return The velocity of the wheel, in feet per second.
     */
    public double getWheelVelocity() {
        return this.getDifferentialOutputs(Stream.of(motors).mapToDouble(SwerveMotor::getVelocity).toArray())[1] * wheelRotationsToFeet;
    }

    /**
     * Gets the current azimuth position.
     * 
     * @return The current azimuth position, in degrees.
     */
    public WrappedAngle getAzimuthPosition() {
        return new WrappedAngle(this.azimuthSensor.getPosition());
    }

    /**
     * Gets the current azimuth position, accounting for wheel reversal.
     * 
     * @return The current azimuth position, in degrees.
     */
    public WrappedAngle getAzimuthPositionFlipped() {
        WrappedAngle azimuth = new WrappedAngle(this.azimuthSensor.getPosition());
        if (this.getWheelVelocity() < 0) {
            azimuth.plus(180);
        }
        return azimuth;
    }

    /**
     * Gets the current azimuth velocity.
     * 
     * @return The current azimuth velocity, in degrees per second.
     */
    public double getAzimuthVelocity() {
        return this.getDifferentialOutputs(Stream.of(motors).mapToDouble(SwerveMotor::getVelocity).toArray())[0] * azimuthRotationsToDegrees;
    }

    /**
     * Gets the current commanded wheel velocity output by the PID controller.
     * 
     * @return The commanded wheel velocity, in feet per second.
     */
    public double getCommandedWheelVelocity() {
        return this.commandedWheelVelocity;
    }

    /**
     * Gets the current commanded azimuth position from the trapezoidal profile.
     * 
     * @return The current commanded azimuth position, in degrees.
     */
    public WrappedAngle getCommandedAzimuthPosition() {
        return new WrappedAngle(this.azimuthPositionController.getLastCommandedPosition());
    }

    /**
     * Gets the current commanded azimuth velocity from the trapezoidal controller.
     * 
     * @return The commanded azimuth velocity, in degrees per second.
     */
    public double getCommandedAzimuthVelocity() {
        return this.azimuthPositionController.getLastCommandedVelocity();
    }

    /**
     * Gets the wheel velocity set as the target for the wheel controller.
     * 
     * @return The target wheel velocity, in feet per second.
     */
    public double getTargetWheelVelocity() {
        return this.targetWheelVelocity;
    }

    /**
     * Gets the azimuth position set as the target for the azimuth controller.
     * 
     * @return The target azimuth position, in degrees.
     */
    public WrappedAngle getTargetAzimuthPosition() {
        return new WrappedAngle(this.azimuthPositionController.getTargetPosition());
    }

    /**
     * Gets the azimuth velocity set as the target for the azimuth controller.
     * 
     * @return The target azimuth velocity, in degrees per second.
     */
    public double getTargetAzimuthVelocity() {
        return this.azimuthPositionController.getTargetVelocity();
    }

    /**
     * Gets the location of this module.
     * 
     * @return A position vector from the robot's origin to the location of
     *               this module, in inches.
     */
    public Vector2D getLocation() {
        return this.config.getLocation();
    }

    /**
     * Gets the motors used by this module.
     * 
     * @return The motors used by this module.
     */
    public SwerveMotor[] getMotors() {
        return this.motors;
    }

    /**
     * Gets the maximum speed that can be commanded to the wheel within the
     * limits of the motor.
     * 
     * @return The max speed, in feet per second.
     */
    public double getMaxWheelSpeed() {
        // Need to check both moving motors in the same direction and moving
        // them in opposite directions, and take the lower of the forwards and
        // backwards wheel speed.
        double positivePositive = this.getDifferentialOutputs(new double[]{motors[0].getMaxVelocity(), motors[1].getMaxVelocity()})[1];
        double positiveNegative = this.getDifferentialOutputs(new double[]{motors[0].getMaxVelocity(), -motors[1].getMaxVelocity()})[1];
        double negativePositive = this.getDifferentialOutputs(new double[]{-motors[0].getMaxVelocity(), motors[1].getMaxVelocity()})[1];
        double negativeNegative = this.getDifferentialOutputs(new double[]{-motors[0].getMaxVelocity(), -motors[1].getMaxVelocity()})[1];
        double maxForwards = Math.max(positivePositive, Math.max(positiveNegative, Math.max(negativePositive, negativeNegative)));
        double maxReverse = -Math.min(positivePositive, Math.min(positiveNegative, Math.min(negativePositive, negativeNegative)));
        return Math.min(maxForwards, maxReverse) * wheelRotationsToFeet;
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
     * Reduces the wheel velocity so that the azimuth can achieve its full
     * velocity.
     * 
     * @param wheelVelocity The desired wheel velocity, in rotations per second.
     * @param azimuthVelocity The azimuth velocity, in rotations per second.
     * @return The reduced wheel velocity, in rotations per second.
     */
    private double reduceWheelVelocityForAzimuth(double wheelVelocity, double azimuthVelocity) {
        /*
        Forward matrix equation:
        | a0  a1 |   | m0 |   | a |
        | w0  w1 | x | m1 | = | w |

        Take the azimuth velocity and the min/max motor 0 velocities, and solve
          for motor 1 velocity:
        m1 = (a - a0*m0) / a1

        Plug both m0 and corresponding m1 values into the equation for wheel
          velocity to find it's limits based on motor 0:
        w = w0*m0 + w1*((a - a0*m0) / a1)

        Repeat for motor 1:
        w = w1*m1 + w0*((a - a1*m1) / a0)

        The wheel velocity must lie within both limits.
        */

        RealMatrix m = this.config.getForwardMatrix();

        double a = azimuthVelocity;
        double w_init = wheelVelocity;
        double a0 = m.getEntry(0, 0);
        double a1 = m.getEntry(0, 1);
        double w0 = m.getEntry(1, 0);
        double w1 = m.getEntry(1, 1);
        double m0_max = motors[0].getMaxVelocity();
        double m0_min = -m0_max;
        double m1_max = motors[1].getMaxVelocity();
        double m1_min = -m1_max;

        double w_m0_minimized = w0*-m0_min + w1*((a - a0*m0_min) / a1);
        double w_m0_maximized = w0*-m0_max + w1*((a - a0*m0_max) / a1);
        double w_m1_minimized = w1*-m1_min + w0*((a - a1*m1_min) / a0);
        double w_m1_maximized = w1*-m1_max + w0*((a - a1*m1_max) / a0);

        // The motor velocity being minimized might mean the wheel velocity
        // is maximized, and vice versa, so can't assume which is min vs. max.
        double w_m0_clamped = MathUtils.clamp(w_init, Math.min(w_m0_minimized, w_m0_maximized), Math.max(w_m0_minimized, w_m0_maximized));
        return MathUtils.clamp(w_m0_clamped, Math.min(w_m1_minimized, w_m1_maximized), Math.max(w_m1_minimized, w_m1_maximized));
    }

    /**
     * Gets the curerent biasTo360 to use for determing how to get to the next
     * angle, depending on the swithing mode.
     * 
     * @return The bias to use
     */
    private double getAzimuthWrapBias() {
        double currentSpeed = Math.abs(this.getWheelVelocity());
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
}
