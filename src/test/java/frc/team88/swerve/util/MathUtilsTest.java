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
}
