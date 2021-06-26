package frc.team88.swerve.motion.state;

import frc.team88.swerve.configuration.subconfig.ConstraintsConfiguration.OptimizerConfiguration;
import java.util.Iterator;
import java.util.stream.IntStream;

/**
 * Takes a base velocity state and a desired velocity state, and tries to get as close to the
 * desired velocity state as possible. It takes the total required change to get from the base to
 * the desired state as [transDirection, transSpeed, rotationVel, cOfRX, cOfRY] and does a binary
 * search between that and 0 to the desired precision, with all values being scaled proportionally
 * to each other.
 */
public class VelocityStateOptimizer implements Iterator<VelocityState> {

  // For simplicity, velocity states and state changes are represented by arrays in the order:
  // 0) Translation Direction
  // 1) Translation Speed
  // 2) Rotation Direction
  // 3) Center of Rotation X
  // 4) Center of Rotation Y
  // Of these, (2) uses precisionDegrees and the rest use precisionFeet.

  private final double[] desiredChange;
  private final OptimizerConfiguration config;

  private double[] nextChangeToTest;
  private double[] minimumChange;
  private double[] maximumChange;

  private boolean foundState = false;

  /**
   * Creates a velocity state optimizer.
   *
   * @param baseState The baseline state to serve as a bound on our search.
   * @param desiredState The desired state to achieve.
   * @param config The config for this optimizer.
   */
  public VelocityStateOptimizer(
      VelocityState baseState, VelocityState desiredState, OptimizerConfiguration config) {
    this.config = config;

    this.desiredChange =
        new double[] {
          desiredState.getTranslationDirection() - baseState.getTranslationDirection(),
          desiredState.getTranslationSpeed() - baseState.getTranslationSpeed(),
          desiredState.getRotationVelocity() - baseState.getRotationVelocity(),
          desiredState.getCenterOfRotationX() - baseState.getCenterOfRotationX(),
          desiredState.getCenterOfRotationY() - baseState.getCenterOfRotationY()
        };

    this.nextChangeToTest = this.desiredChange.clone();
    this.minimumChange = new double[] {0, 0, 0, 0, 0};
    this.maximumChange = this.desiredChange.clone();
  }

  /**
   * Returns the optimized state. Should only be called once this iterator has finished.
   *
   * @return The optimized state.
   */
  public VelocityState getOptimizedState() {
    return next();
  }

  @Override
  public boolean hasNext() {
    return !foundState;
  }

  @Override
  public VelocityState next() {
    return new VelocityState(
        this.nextChangeToTest[0],
        this.nextChangeToTest[1],
        this.nextChangeToTest[2],
        this.nextChangeToTest[3],
        this.nextChangeToTest[4],
        false);
  }

  /**
   * This method should be called after testing the value returned by next() to tell the iterator if
   * it is valid or not.
   *
   * @param valid True if the last state returned is valid.
   */
  public void setIfLastStateWasValid(boolean valid) {
    if (valid) {
      if (checkInPrecision(this.nextChangeToTest, this.maximumChange)) {
        this.foundState = true;
        return;
      }

      this.minimumChange = this.nextChangeToTest.clone();
      this.nextChangeToTest =
          IntStream.range(0, this.desiredChange.length)
              .mapToDouble((i) -> (this.maximumChange[i] - this.nextChangeToTest[i]) / 2)
              .toArray();
    } else {
      if (checkInPrecision(this.nextChangeToTest, this.minimumChange)) {
        this.foundState = true;
        return;
      }

      this.maximumChange = this.nextChangeToTest.clone();
      this.nextChangeToTest =
          IntStream.range(0, this.desiredChange.length)
              .mapToDouble((i) -> (this.nextChangeToTest[i] - this.minimumChange[i]) / 2)
              .toArray();
    }
  }

  /**
   * Checks if the 2 given states are the same within precision.
   *
   * @param state0 The first state to compare.
   * @param state1 The second state to comare.
   * @return The if the 2 states are the same within precision, falso otherwise.
   */
  private boolean checkInPrecision(double[] state0, double[] state1) {
    return Math.abs(state0[0] - state1[0]) < config.getPrecisionFeet()
        && Math.abs(state0[1] - state1[1]) < config.getPrecisionFeet()
        && Math.abs(state0[2] - state1[2]) < config.getPrecisionDegrees()
        && Math.abs(state0[3] - state1[3]) < config.getPrecisionFeet()
        && Math.abs(state0[4] - state1[4]) < config.getPrecisionFeet();
  }
}
