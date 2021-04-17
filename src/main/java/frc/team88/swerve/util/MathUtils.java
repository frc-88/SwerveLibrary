package frc.team88.swerve.util;

/**
 * Contains general math utility functions.
 */
public class MathUtils {

    /**
     * Determines if 2 double values are equal to with a reasonable floating point
     * error
     * 
     * @param a
     *              The expected value
     * @param b
     *              The actual value
     * @return If the 2 values are approximately equal
     */
    public static boolean doubleEquals(double a, double b) {
        final double ZERO_THRESHOLD = 1.E-9;
        final double PERCENT_ERROR_MAX = 0.0001;
        if (Math.abs(a) < ZERO_THRESHOLD || Math.abs(b) < ZERO_THRESHOLD) {
            // If one of the numbers is very close to 0, use absolute error
            return Math.abs(a - b) < ZERO_THRESHOLD;
        }
        // Use percent error
        return Math.abs((a - b) / a) < PERCENT_ERROR_MAX;
    }

    /**
     * Limits the ammount of change when trying to go from the current value to the
     * desired value.
     * 
     * @param current
     *                      The current value
     * @param desired
     *                      The desired value
     * @param maxChange
     *                      The maximum amount that the returned value can differ
     *                      from the current value
     * @return The value that is as close to the desired value as possible without
     *         exceeing the maximum change
     */
    public static double limitChange(double current, double desired, double maxChange) {
        if (desired > current) {
            return Math.min(desired, current + maxChange);
        } else {
            return Math.max(desired, current - maxChange);
        }
    }

    /**
     * Calculates the exponentiation while mantaining sign for even exponents.
     * 
     * @param base
     *               The base value.
     * @param exp
     *               The exponent value.
     * @return
     *               The result of the exponentiation.
     */
    public static double signedPow(double base, int exp){
        double raw = Math.pow(base, exp);
        return (base < 0 && exp % 2 == 0) ? -raw : raw;
    }
}
