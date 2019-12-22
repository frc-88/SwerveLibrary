package frc.team88.swerve.util;

import java.util.Objects;

import org.javatuples.Pair;

/**
 * Represents an angle which is constrained to [-180, 180) degrees. Does the
 * math to find the best direction to get from one angle to another.
 */
public class WrappedAngle {

    private double angle;

    /**
     * Constructor.
     * 
     * @param angle
     *                  The angle that this will represent, in degrees. Not bound to
     *                  any range.
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
     * @param addend
     *                   The angle to add, in degrees
     * @return The sum
     */
    public WrappedAngle plus(double addend) {
        return new WrappedAngle(this.angle + addend);
    }

    /**
     * Return the sum of this angle and the given angle
     * 
     * @param addend
     *                   The angle to add
     * @return The sum
     */
    public WrappedAngle plus(WrappedAngle addend) {
        return this.plus(addend.asDouble());
    }

    /**
     * Gets the smallest difference (by magnitude) between this angle and the given
     * angle, either going clockwise or counter-clockwise.
     * 
     * @param that
     *                 The angle to get the difference from
     * @return The difference, in degrees. When added to this angle, the sum will be
     *         the given angle.
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
     * Gets the smallest difference (by magnitude) between this angle and either the
     * given angle or the given angle + 180, going etiher clockwise or
     * counter-clockwise.
     * 
     * @param that
     *                      The angle to get the difference from
     * @param biasTo360
     *                      The maximum distance from the given angle where the
     *                      distance returned will be from it and not it + 180.
     *                      Range is [0, 180], where 90 means indifference to where
     *                      the given angle or the given angle + 180 is used, 180
     *                      means never use the given angle + 180, and anything less
     *                      than 0.01 means always use the given angle + 180
     * @return A javatuples pair containing (1) The difference, in degrees. When
     *         added to this angle, the sum will be the given angle or the given
     *         angle + 180. (2) True adding the difference will result in the given
     *         angle + 180, false if it will result in the given angle
     */
    public Pair<Double, Boolean> getSmallestDifferenceWithHalfAngle(WrappedAngle that, double biasTo360) {
        Objects.requireNonNull(that);
        if (biasTo360 < 0 || biasTo360 > 180) {
            throw new IllegalArgumentException("biasTo360 must be in range [0, 180]");
        }
        double differenceToFullAngle = getSmallestDifferenceWith(that);
        if (Math.abs(differenceToFullAngle) > biasTo360 || biasTo360 < 0.01) {
            return new Pair<Double, Boolean>(getSmallestDifferenceWith(new WrappedAngle(that.asDouble() + 180)), true);
        } else {
            return new Pair<Double, Boolean>(differenceToFullAngle, false);
        }
    }

    /**
     * Wraps the angle to be in the range [-180, 180).
     * 
     * @param angle
     *                  The angle to wrap, in degrees
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
