package frc.team88.swerve.util;

import frc.team88.swerve.configuration.subconfig.PIDConfiguration;
import java.util.Objects;

/**
 * Class that performs the math of a PID controller, converting a setpoint and error to an output.
 */
public class SyncPIDController {

  private PIDConfiguration config;

  private double m_accum = 0;
  private double m_prevError = 0;
  private long m_lastLoopTime = 0; // us

  private static final double EPSILON = 0.00000001;

  /**
   * Constructor that uses a PIDConfiguration object.
   *
   * @param constants Contains all of the PID constants for this PID controller
   */
  public SyncPIDController(final PIDConfiguration constants) {
    this.config = Objects.requireNonNull(constants);
  }

  /**
   * Gets the proportial constant for the PID.
   *
   * @return The proportional gain.
   */
  public double getKP() {
    return this.config.getKP();
  }

  /**
   * Gets the integral constant for the PID.
   *
   * @return The integral gain.
   */
  public double getKI() {
    return this.config.getKI();
  }

  /**
   * Gets the derivative constant for the PID.
   *
   * @return The differential gain.
   */
  public double getKD() {
    return this.config.getKD();
  }

  /**
   * Gets the feedforward constant for the PID.
   *
   * @return the feedforward gain.
   */
  public double getKF() {
    return this.config.getKF();
  }

  /**
   * Gets the iZone constant for the PID.
   *
   * @return the maximum error for integral accumulation.
   */
  public double getIZone() {
    return this.config.getIZone();
  }

  /**
   * Gets the iMax constant for the PID.
   *
   * @return The maximum accumulated error.
   */
  public double getIMax() {
    return this.config.getIMax();
  }

  /**
   * Gets the tolerance for the PID.
   *
   * @return The max error to consider the PID on target.
   */
  public double getTolerance() {
    return this.config.getTolerance();
  }

  /**
   * Resets the controller's saved info about integral and derivative. Should be called right before
   * the first use of the controller after it hasn't been used for a while.
   */
  public void reset() {
    this.m_accum = 0;
    this.m_prevError = 0;
    this.m_lastLoopTime = 0;
  }

  /**
   * Gets the output calculated by this PID.
   *
   * @param input The current input value
   * @param setpoint The target input value
   * @return The output value
   */
  public double calculateOutput(double input, double setpoint) {
    double error = setpoint - input;

    if (Math.abs(error) < getTolerance()) {
      return 0;
    }

    double output = 0;
    output += calculateP(error);
    output += calculateI(error);
    output += calculateD(error);
    output += calculateF(setpoint);

    return output;
  }

  /**
   * Calculate the proportional output
   *
   * @param error The current error
   * @return The proportional output
   */
  protected double calculateP(double error) {
    return getKP() * error;
  }

  /**
   * Calculate the integral output
   *
   * @param error The current error
   * @return The integral output
   */
  protected double calculateI(double error) {

    if (getIZone() < EPSILON || Math.abs(error) < getIZone()) {
      m_accum += error;
    } else {
      m_accum = 0;
    }

    if (getIMax() > EPSILON) {
      if (m_accum > 0) {
        m_accum = Math.min(m_accum, getIMax() / getKI());
      } else {
        m_accum = Math.max(m_accum, -getIMax() / getKI());
      }
    }

    return getKI() * m_accum;
  }

  /**
   * Calculate the differential output
   *
   * @param error The current error
   * @return The differential output
   */
  protected double calculateD(double error) {

    long curTime = RobotControllerWrapper.getInstance().getFPGATime();

    double ret = getKD() * (error - m_prevError) / ((curTime - m_lastLoopTime) / 1e6);

    m_prevError = error;
    m_lastLoopTime = curTime;

    return ret;
  }

  /**
   * Calulcate the feedforward output
   *
   * @param setpoint The current setpoint
   * @return The feedforward output
   */
  protected double calculateF(double setpoint) {
    return getKF() * setpoint;
  }
}
