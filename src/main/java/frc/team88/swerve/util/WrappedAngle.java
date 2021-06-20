package frc.team88.swerve.util;

import java.util.Objects;

/**
 * Represents an angle which is constrained to [-180, 180) degrees. Does the math to find the best
 * direction to get from one angle to another.
 */
public class WrappedAngle {

  private double angle;

  /**
   * Constructor.
   *
   * @param angle The angle that this will represent, in degrees. Not bound to any range.
   */
  public WrappedAngle(double angle) {
    this.angle = wrapAngle(angle);
  }

  /**
   * Gets the angle that this represents, in the range [-180, 180).
   *
   * @return The angle in degrees
   */
  public double asDouble() {
    return angle;
  }

  /**
   * Returns the sum of this angle and the given angle
   *
   * @param addend The angle to add, in degrees
   * @return The sum
   */
  public WrappedAngle plus(double addend) {
    return new WrappedAngle(this.angle + addend);
  }

  /**
   * Return the sum of this angle and the given angle
   *
   * @param addend The angle to add
   * @return The sum
   */
  public WrappedAngle plus(WrappedAngle addend) {
    return this.plus(addend.asDouble());
  }

  /**
   * Gets the smallest difference (by magnitude) between this angle and the given angle, either
   * going clockwise or counter-clockwise.
   *
   * @param that The angle to get the difference from
   * @return The difference, in degrees. When added to this angle, the sum will be the given angle.
   */
  public double getSmallestDifferenceWith(WrappedAngle that) {
    Objects.requireNonNull(that);
    double difference = that.asDouble() - this.asDouble();
    if (difference >= -180 && difference < 180) {
      return difference;
    } else if (difference < -180) {
      return difference + 360;
    } else {
      return difference - 360;
    }
  }

  /**
   * Wraps the angle to be in the range [-180, 180).
   *
   * @param angle The angle to wrap, in degrees
   * @return The wrapped angle, in degrees
   */
  public static double wrapAngle(double angle) {
    angle = (angle + 180) % 360;
    if (angle < 0) {
      angle += 360;
    }
    angle -= 180;
    return angle;
  }
}
