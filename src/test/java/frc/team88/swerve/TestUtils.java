package frc.team88.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.Vector2D;

/**
 * Utility class containing static functions which are useful for testing.
 */
public class TestUtils {

    public static final double EPSILON = 0.000001;

    /**
     * Junit assert equals for doubles, with default floating point tolerance.
     * 
     * @param expected
     *                     The expected value
     * @param actual
     *                     The actual value
     */
    public static void assertDoubleEquals(double expected, double actual) {
        assertEquals(expected, actual, EPSILON);
    }

    /**
     * Junit assert equals for Vector2Ds, with default floating point tolerance.
     * Compares magnitude and angle.
     * 
     * @param expected
     *                     The expected value
     * @param actual
     *                     The actual value
     */
    public static void assertVectorEquals(Vector2D expected, Vector2D actual) {
        assertDoubleEquals(expected.getMagnitude(), actual.getMagnitude());
        assertDoubleEquals(expected.getAngle().asDouble(), actual.getAngle().asDouble());
    }

    /**
     * Junit assert equals for velocity states, with default floating point tolerance.
     * Compares field-centricitiy, translation velocity, rotation velocity, and
     * center of rotation
     * 
     * @param expected
     *                     The expected value
     * @param actual
     *                     The actual value
     */
    public static void assertVelocityStateEquals(VelocityState expected, VelocityState actual) {
        assertEquals(expected.isFieldCentric(), actual.isFieldCentric());
        assertDoubleEquals(expected.getTranslationDirection(), actual.getTranslationDirection());
        assertDoubleEquals(expected.getTranslationSpeed(), actual.getTranslationSpeed());
        assertDoubleEquals(expected.getRotationVelocity(), actual.getRotationVelocity());
        assertVectorEquals(expected.getCenterOfRotationVector(), actual.getCenterOfRotationVector());
    }

}
