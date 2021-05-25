package frc.team88.swerve.util;

import java.util.Objects;

import frc.team88.swerve.configuration.subconfig.TrapezoidalControllerConfiguration;

/**
 * Class that performs the math of a trapezoidal profile controller. Calculates the required speed
 * to follow the profile live. Handles a combined position and velocity target.
 */
public class TrapezoidalProfileController {

  // The config for this trapezoidal controller.
  private final TrapezoidalControllerConfiguration config;

  // The position controller used to adjust the velocity.
  private final SyncPIDController positionPID;

  // The target velocity which is a setpoint for this controller, in units per
  // second.
  private double targetVelocity = 0;

  // The target position which is a setpoint for this controller.
  private double targetPosition = 0;

  // The time at which the last output was calculated, in microseconds.
  private long lastCalculationTime = 0;

  // The commanded position from the last time the output was calculated.
  private double lastCommandedPosition = 0;

  // The commanded velocity from the last time the output was calculated.
  private double lastCommandedVelocity = 0;

  /**
   * Constructor.
   *
   * @param config The configuration for this controller.
   */
  public TrapezoidalProfileController(TrapezoidalControllerConfiguration config) {
    this.config = Objects.requireNonNull(config);
    this.positionPID = new SyncPIDController(config.getPIDConfig());
  }

  /**
   * Sets the target velocity.
   *
   * @param velocity The target velocity which is a setpoint for this controller, in units per
   *     second.
   */
  public void setTargetVelocity(double velocity) {
    this.targetVelocity = velocity;
  }

  /**
   * Sets the target position.
   *
   * @param position The target position which is a setpoint for this controller.
   */
  public void setTargetPosition(double position) {
    this.targetPosition = position;
  }

  /**
   * Gets the max speed limit.
   *
   * @return The maximum speed value that can be both commanded and output, in units per second.
   */
  public double getMaxSpeed() {
    return this.config.getMaxSpeed();
  }

  /**
   * Gets the max acceleration limit.
   *
   * @return The maximum acceleration for the trapezoidal profile, in units per second^2
   */
  public double getMaxAcceleration() {
    return this.config.getMaxAcceleration();
  }

  /**
   * Gets the target velocity.
   *
   * @return The target velocity which is a setpoint for this controller, in units per second.
   */
  public double getTargetVelocity() {
    return this.targetVelocity;
  }

  /**
   * Gets the target position.
   *
   * @return The target position which is a setpoint for this controller.
   */
  public double getTargetPosition() {
    return this.targetPosition;
  }

  /**
   * Gets the last commanded position.
   *
   * @return The last commanded position.
   */
  public double getLastCommandedPosition() {
    return this.lastCommandedPosition;
  }

  /**
   * Gets the last commanded velocity.
   *
   * @return The last commanded velocity.
   */
  public double getLastCommandedVelocity() {
    return this.lastCommandedVelocity;
  }

  /**
   * Gets the position PID controller.
   *
   * @return The position controller used to adjust the velocity.
   */
  public SyncPIDController getPositionPID() {
    return this.positionPID;
  }

  /**
   * Resets the trapezoidal controller, including the associated position PID. Should be called
   * after this controller hasn't been used for a long time.
   *
   * @param currentPosition The current positon.
   */
  public void reset(double currentPosition) {
    this.lastCalculationTime = RobotControllerWrapper.getInstance().getFPGATime();
    this.lastCommandedPosition = currentPosition;
    this.positionPID.reset();
  }

  /**
   * Calculates the velocity output following the trapezoidal profile.
   *
   * @param currentPosition The current position as an input.
   * @param currentVelocity The current velocity as an input, in units per second.
   * @return The velocity value to command as the output, in units per second
   */
  public double calculateCommandVelocity(double currentPosition, double currentVelocity) {
    // Make sure the elapsed time and difference in commanded position is reasonable
    if (RobotControllerWrapper.getInstance().getFPGATime() - lastCalculationTime > 200_000
        || Math.abs(this.lastCommandedPosition - currentPosition) > this.getMaxSpeed() / 20.) {
      this.lastCommandedPosition = currentPosition;
      this.lastCalculationTime = RobotControllerWrapper.getInstance().getFPGATime() - 20_000;
    }

    // Determine if the position target is ahead of or behind the current position
    boolean forwards = targetPosition > lastCommandedPosition;

    // Calculate the command velocity based on the current velocity and max
    // acceleration
    double commandVelocity = calculateAcceleratedVelocity(lastCommandedVelocity, forwards);

    // Limit the command velocity based on the max speed
    commandVelocity = applyMaxSpeedLimit(commandVelocity);

    // Limit the command velocity such that it has time to deccelerate to hit the
    // target at the right velocity
    commandVelocity = applyDeccelerationLimit(commandVelocity, forwards);
    this.lastCommandedVelocity = commandVelocity;

    // Determine the position to command
    double commandPosition = calculateCommandPosition(commandVelocity);

    // Apply the position PID
    commandVelocity += positionPID.calculateOutput(currentPosition, commandPosition);

    // Update the last calculation time and commanded position
    this.lastCalculationTime = RobotControllerWrapper.getInstance().getFPGATime();
    this.lastCommandedPosition = commandPosition;

    // Return the command velocity
    return commandVelocity;
  }

  /**
   * Calculates the command velocity based on the current velocity and max acceleration.
   *
   * @param currentVelocity The current velocity, in units per second.
   * @param forwards True if the value returned should be greater than the current velocity, false
   *     if it should be smaller.
   * @return The calculated command velocity, in units per second.
   */
  protected double calculateAcceleratedVelocity(double currentVelocity, boolean forwards) {
    double sign = forwards ? 1 : -1;
    double addedVelocity =
        ((RobotControllerWrapper.getInstance().getFPGATime() - lastCalculationTime) / 1_000_000.)
            * this.getMaxAcceleration()
            * sign;
    return currentVelocity + addedVelocity;
  }

  /**
   * Limits the command velocity such that it does not exceed the max speed limit.
   *
   * @param commandVelocity The command velocity to limit, in units per second.
   * @return The limited velocity, in units per second.
   */
  protected double applyMaxSpeedLimit(double commandVelocity) {
    return Math.min(getMaxSpeed(), Math.max(-getMaxSpeed(), commandVelocity));
  }

  /**
   * Limit the command velocity such that it has time to deccelerate to hit the target at the right
   * velocity.
   *
   * @param commandVelocity The command velocity to limit, in units per second.
   * @param forwards Whether we are moving forwards or backwards.
   * @return The limited velocity, in units per second.
   */
  protected double applyDeccelerationLimit(double commandVelocity, boolean forwards) {
    // v^2 = v0^2 + 2ax -> v0 = sqrt(v^2 - 2ax)
    double limitVelocity =
        MathUtils.signedPow(targetVelocity, 2)
            + 2 * getMaxAcceleration() * (this.targetPosition - lastCommandedPosition);
    limitVelocity = Math.signum(limitVelocity) * Math.sqrt(Math.abs(limitVelocity));
    if (forwards) {
      return Math.min(limitVelocity, commandVelocity);
    } else {
      return Math.max(limitVelocity, commandVelocity);
    }
  }

  /**
   * Determine the command position based on the last command position and the velocity.
   *
   * @param commandVelocity The currently commanded velocity, in units per second.
   * @return The position to command.
   */
  protected double calculateCommandPosition(double commandVelocity) {
    return lastCommandedPosition
        + ((RobotControllerWrapper.getInstance().getFPGATime() - lastCalculationTime) / 1_000_000.)
            * commandVelocity;
  }
}
