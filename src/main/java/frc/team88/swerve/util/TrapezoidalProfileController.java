package frc.team88.swerve.util;

import java.util.Objects;

import frc.team88.swerve.wrappers.RobotControllerWrapper;

/**
 * Class that performs the math of a trapezoidal profile controller. Calculates
 * the required speed to follow the profile live. Handles a combined position
 * and velocity target.
 */
public class TrapezoidalProfileController {

    // The maximum speed value that can be both commanded and output, in units per
    // second.
    private double maxSpeed;

    // The maximum acceleration for the trapezoidal profile, in units per second^2.
    private double maxAcceleration;

    // The position controller used to adjust the velocity.
    private SyncPIDController positionPID;

    // The target velocity which is a setpoint for this controller, in units per
    // second.
    private double targetVelocity = 0;

    // The target position which is a setpoint for this controller.
    private double targetPosition = 0;

    // The time at which the last output was calculated, in microseconds.
    private long lastCalculationTime = 0;

    // The commanded position from the last time the output was calculated.
    private double lastCommandedPosition = 0;

    /**
     * Constructor.
     * 
     * @param maxSpeed
     *                            The maximum speed value that can be both commanded
     *                            and output, in units per second
     * @param maxAcceleration
     *                            The maximum acceleration for the trapezoidal
     *                            profile, in units per second^2
     * @param positionPID
     *                            The position controller used to adjust the
     *                            velocity
     */
    public TrapezoidalProfileController(double maxSpeed, double maxAcceleration, SyncPIDController positionPID) {
        if (maxSpeed <= 0) {
            throw new IllegalArgumentException("Max speed must be positive");
        }
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("Max acceleration must be positive");
        }
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.positionPID = Objects.requireNonNull(positionPID);
    }

    /**
     * Sets the max speed limit.
     * 
     * @param maxSpeed
     *                     The maximum speed value that can be both commanded and
     *                     output, in units per second
     */
    public void setMaxSpeed(double maxSpeed) {
        if (maxSpeed <= 0) {
            throw new IllegalArgumentException("Max speed must be positive");
        }
        this.maxSpeed = maxSpeed;
    }

    /**
     * Sets the max acceleration limit.
     * 
     * @param maxAcceleration
     *                            The maximum acceleration for the trapezoidal
     *                            profile, in units per second^2
     */
    public void setMaxAcceleration(double maxAcceleration) {
        if (maxAcceleration <= 0) {
            throw new IllegalArgumentException("Max acceleration must be positive");
        }
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Sets the target velocity.
     * 
     * @param velocity
     *                     The target velocity which is a setpoint for this
     *                     controller, in units per second
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    /**
     * Sets the target position.
     * 
     * @param position
     *                     The target position which is a setpoint for this
     *                     controller
     */
    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    /**
     * Gets the max speed limit.
     * 
     * @return The maximum speed value that can be both commanded and output, in
     *         units per second
     */
    public double getMaxSpeed() {
        return this.maxSpeed;
    }

    /**
     * Gets the max acceleration limit.
     * 
     * @return The maximum acceleration for the trapezoidal profile, in units per
     *         second^2
     */
    public double getMaxAcceleration() {
        return this.maxAcceleration;
    }

    /**
     * Gets the target velocity.
     * 
     * @return The target velocity which is a setpoint for this controller, in units
     *         per second
     */
    public double getTargetVelocity() {
        return this.targetVelocity;
    }

    /**
     * Gets the target position.
     * 
     * @return The target position which is a setpoint for this controller
     */
    public double getTargetPosition() {
        return this.targetPosition;
    }

    /**
     * Gets the position PID controller.
     * 
     * @return The position controller used to adjust the velocity
     */
    public SyncPIDController getPositionPID() {
        return this.positionPID;
    }

    /**
     * Resets the trapezoidal controller, including the associated position PID.
     * Should be called after this controller hasn't been used for a long time.
     * 
     * @param currentPosition
     *                            The current positon
     */
    public void reset(double currentPosition) {
        this.lastCalculationTime = RobotControllerWrapper.getInstance().getFPGATime();
        this.targetPosition = currentPosition;
        this.positionPID.reset();
    }

    /**
     * Calculates the velocity output following the trapezoidal profile.
     * 
     * @param currentPosition
     *                            The current position as an input
     * @param currentVelocity
     *                            The current velocity as an input, in units per
     *                            second
     * @return The velocity value to command as the output, in units per second
     */
    public double calculateCommandVelocity(double currentPosition, double currentVelocity) {
        // Determine if the position target is ahead of or behind the current position
        boolean forwards = targetPosition > currentPosition;

        // Calculate the command velocity based on the current velocity and max
        // acceleration
        double commandVelocity = calculateAcceleratedVelocity(currentVelocity, forwards);

        // Limit the command velocity based on the max speed
        commandVelocity = applyMaxSpeedLimit(commandVelocity);

        // Limit the command velocity such that it has time to deccelerate to hit the
        // target at the right velocity
        commandVelocity = applyDeccelerationLimit(commandVelocity, forwards);

        // Determine the position to command
        double commandPosition = calculateCommandPosition(commandVelocity);

        // Apply the position PID
        commandVelocity += positionPID.calculateOutput(currentPosition, commandPosition);

        // Update the last calculation time and commanded position
        this.lastCalculationTime = RobotControllerWrapper.getInstance().getFPGATime();
        lastCommandedPosition = commandPosition;

        // Return the command velocity
        return commandVelocity;
    }

    /**
     * Calculates the command velocity based on the current velocity and max
     * acceleration.
     * 
     * @param currentVelocity
     *                            The current velocity, in units per second
     * @param forwards
     *                            True if the value returned should be greater than
     *                            the current velocity, false if it should be
     *                            smaller
     * @return The calculated command velocity, in units per second
     */
    protected double calculateAcceleratedVelocity(double currentVelocity, boolean forwards) {
        double sign = forwards ? 1 : -1;
        double addedVelocity = (RobotControllerWrapper.getInstance().getFPGATime() - lastCalculationTime)
                * this.getMaxAcceleration() * sign;
        return currentVelocity + addedVelocity;
    }

    /**
     * Limits the command velocity such that it does not exceed the max speed limit.
     * 
     * @param commandVelocity
     *                            The command velocity to limit, in units per second
     * @return The limited velocity, in units per second
     */
    protected double applyMaxSpeedLimit(double commandVelocity) {
        return Math.min(maxSpeed, Math.max(-maxSpeed, commandVelocity));
    }

    /**
     * Limit the command velocity such that it has time to deccelerate to hit the
     * target at the right velocity.
     * 
     * @param commandVelocity The command velocity to limit, in units per second
     * @return The limited velocity, in units per second
     */
    protected double applyDeccelerationLimit(double commandVelocity, boolean forwards) {
        // v^2 = v0^2 + 2ax -> v0 = sqrt(v^2 - 2ax)
        double limitVelocity = Math
                .sqrt(Math.pow(targetVelocity, 2) - 2 * maxAcceleration * (targetPosition - lastCommandedPosition));
        if (forwards) {
            return Math.min(limitVelocity, commandVelocity);
        } else {
            return Math.max(limitVelocity, commandVelocity);
        }
    }

    /**
     * Determine the command position based on the last command position and the velocity.
     * 
     * @param commandVelocity The currently commanded velocity, in units per second
     * @return The position to command
     */
    protected double calculateCommandPosition(double commandVelocity) {
        return lastCommandedPosition + (RobotControllerWrapper.getInstance().getFPGATime() - lastCalculationTime) * commandVelocity;
    }

}
