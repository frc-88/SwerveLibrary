package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class MathUtilsTest {

    @Test
    public void testDoubleEqualsTrue() {
        assertTrue(MathUtils.doubleEquals((1. / 10_000.) * 10., (10. / 1_000_000.) * 100.));
    }

    @Test
    public void testDoubleEqualsFalse() {
        assertFalse(MathUtils.doubleEquals(1., 1.001));
    }

    @Test
    public void testLimitChangeIncreaseUnrestricted() {
        assertEquals(10., MathUtils.limitChange(8., 10., 4.), 0.000001);
    }

    @Test
    public void testLimitChangeIncreaseRestricted() {
        assertEquals(12., MathUtils.limitChange(8., 16., 4.), 0.000001);
    }

    @Test
    public void testLimitChangeDecreaseUnrestricted() {
        assertEquals(8., MathUtils.limitChange(10., 8., 6.), 0.000001);
    }

    @Test
    public void testLimitChangeDecreaseRestricted() {
        assertEquals(4., MathUtils.limitChange(10., 2., 6.), 0.000001);
    }
}
