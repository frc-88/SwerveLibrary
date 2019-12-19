package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class MathUtilsTest {

    @Test
    public void doubleEqualsTrue() {
        assertTrue(MathUtils.doubleEquals((1./10_000.)*10., 
                (10./1_000_000.)*100.));
    }

    @Test
    public void doubleEqualsFalse() {
        assertFalse(MathUtils.doubleEquals(1., 1.001));
    }

    @Test
    public void testReferenceAngle0() {
        assertEquals(0., MathUtils.getReferenceAngle(0), 0.001);
    }

    @Test
    public void testReferenceAngle179() {
        assertEquals(179., MathUtils.getReferenceAngle(179), 0.001);
    }

    @Test
    public void testReferenceAngleNeg180() {
        assertEquals(-180., MathUtils.getReferenceAngle(-180), 0.001);
    }

    @Test
    public void testReferenceAngle270() {
        assertEquals(-90., MathUtils.getReferenceAngle(-90), 0.001);
    }

    @Test
    public void testReferenceAngleNeg270() {
        assertEquals(90., MathUtils.getReferenceAngle(-270), 0.001);
    }

    @Test
    public void testReferenceAngle750() {
        assertEquals(30., MathUtils.getReferenceAngle(750), 0.001);
    }

    @Test
    public void testReferenceAngleNeg750() {
        assertEquals(-30., MathUtils.getReferenceAngle(-750), 0.001);
    }
}