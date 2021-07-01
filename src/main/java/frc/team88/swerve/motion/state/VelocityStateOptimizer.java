package frc.team88.swerve.motion.state;

import frc.team88.swerve.configuration.subconfig.ConstraintsConfiguration.OptimizerConfiguration;
import java.util.Iterator;
import java.util.Optional;
import java.util.stream.IntStream;

/**
 * Takes a desired velocity state and tries to find a valid state as close it as possible using an
 * algorithm based on binary search.
 */
public class VelocityStateOptimizer implements Iterator<VelocityState> {

  // For simplicity, velocity states and state changes are represented by arrays in the order:
  // 0) Translation Direction
  // 1) Translation Speed
  // 2) Rotation Direction

  private final double[] desiredState;
  private final OptimizerConfiguration config;

  private double[] nextStateToTest;
  private double[] minimumState;
  private double[] maximumState;

  private Optional<double[]> foundState = Optional.empty();

  /**
   * Creates a velocity state optimizer.
   *
   * @param desiredState The desired state to achieve. 
   * @param config The config for this optimizer.
   */
  public VelocityStateOptimizer(VelocityState desiredState, OptimizerConfiguration config) {
    this.config = config;
    
    this.desiredState = this.stateToArray(desiredState);

    this.nextStateToTest = this.desiredState.clone();
    this.minimumState = this.stateToArray(new VelocityState(0, 0, 0, false));
    this.maximumState = this.desiredState.clone();
  }

  /**
   * Returns the optimized state. Should only be called once this iterator has finished.
   *
   * @return The optimized state.
   */
  public VelocityState getOptimizedState() {
    if (this.foundState.isPresent()) {
      System.err.println("The state has not yet been optimized, returning minimum valid state.");
      return this.arrayToState(this.minimumState);
    }

    return this.arrayToState(this.foundState.get());
  }

  @Override
  public boolean hasNext() {
    return this.foundState.isPresent();
  }

  @Override
  public VelocityState next() {
    return this.arrayToState(this.nextStateToTest);
  }

  /**
   * This method should be called after testing the value returned by next() to tell the iterator if
   * it is valid or not.
   *
   * @param valid True if the last state returned is valid.
   */
  public void setIfLastStateWasValid(boolean valid) {
    if (valid) {
      if (checkInPrecision(this.nextStateToTest, this.maximumState)) {
        this.foundState = Optional.of(this.nextStateToTest);
        return;
      }

      this.minimumState = this.nextStateToTest.clone();
      this.nextStateToTest =
          IntStream.range(0, this.desiredState.length)
              .mapToDouble((i) -> (this.maximumState[i] - this.nextStateToTest[i]) / 2)
              .toArray();
    } else {
      if (checkInPrecision(this.nextStateToTest, this.minimumState)) {
        this.foundState = Optional.of(this.minimumState);
        return;
      }

      this.maximumState = this.nextStateToTest.clone();
      this.nextStateToTest =
          IntStream.range(0, this.desiredState.length)
              .mapToDouble((i) -> (this.nextStateToTest[i] - this.minimumState[i]) / 2)
              .toArray();
    }
  }

  /**
   * Converts a velocity state to an array.
   * 
   * @param state The state to convert.
   * @return An array containing {translationDirection, translationSpeed, rotationVelocity}.
   */
  private double[] stateToArray(VelocityState state) {
    return new double[] {state.getTranslationDirection(), state.getTranslationSpeed(), state.getRotationVelocity()};
  }

  /**
   * Converts an array back to a velocity state.
   * 
   * @param arr The array to convert, containing {translationDirection, translationSpeed, rotationVelocity}.
   * @return A velocity state with center of rotation [0, 0] in robot-centric coordinates.
   */
  private VelocityState arrayToState(double[] arr) {
    return new VelocityState(arr[0], arr[1], arr[2], false);
  }

  /**
   * Checks if the 2 given states are the same within precision.
   *
   * @param state0 The first state to compare.
   * @param state1 The second state to comare.
   * @return The if the 2 states are the same within precision, falso otherwise.
   */
  private boolean checkInPrecision(double[] state0, double[] state1) {
    double[] precisions = new double[]{this.config.getTranslationDirectionPrecision(), this.config.getTranslationSpeedPrecision(), this.config.getRotationVelocityPrecision()};
    return IntStream.range(0, state0.length).allMatch((i) -> Math.abs(state0[i] - state1[i]) < precisions[i]);
  }
}
