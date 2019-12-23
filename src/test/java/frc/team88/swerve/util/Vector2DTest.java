package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class Vector2DTest {

    private Vector2D origin;
    private Vector2D up2;
    private Vector2D left1;
    private Vector2D down3;
    private Vector2D rightHalf;
    private Vector2D upRight;
    private Vector2D downLeft;
    private Vector2D deg30;
    private Vector2D deg135;
    private Vector2D degNeg60;
    private Vector2D degNeg135;

    @BeforeEach
    public void setup() {
        origin = Vector2D.createCartesianCoordinates(0, 0);
        up2 = Vector2D.createCartesianCoordinates(0, 2);
        left1 = Vector2D.createCartesianCoordinates(-1, 0);
        down3 = Vector2D.createCartesianCoordinates(0, -3);
        rightHalf = Vector2D.createCartesianCoordinates(0.5, 0);
        upRight = Vector2D.createCartesianCoordinates(1, 3);
        downLeft = Vector2D.createCartesianCoordinates(-2, -1);
        deg30 = Vector2D.createPolarCoordinates(1, new WrappedAngle(30));
        deg135 = Vector2D.createPolarCoordinates(1, new WrappedAngle(135));
        degNeg60 = Vector2D.createPolarCoordinates(1, new WrappedAngle(-60));
        degNeg135 = Vector2D.createPolarCoordinates(2, new WrappedAngle(-135));
    }

    @Test
    public void testCreateCartesian() {
        Vector2D v = Vector2D.createCartesianCoordinates(3, 5);
        assertEquals(3., v.getX(), 0.001);
        assertEquals(5., v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarQ1() {
        Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(30));
        assertEquals(-1., v.getX(), 0.001);
        assertEquals(Math.sqrt(3.), v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarQ2() {
        Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(120));
        assertEquals(-Math.sqrt(3), v.getX(), 0.001);
        assertEquals(-1, v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarQ3() {
        Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(-60));
        assertEquals(Math.sqrt(3), v.getX(), 0.001);
        assertEquals(1, v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarQ4() {
        Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(-150));
        assertEquals(1, v.getX(), 0.001);
        assertEquals(-Math.sqrt(3), v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarOrigin() {
        Vector2D v = Vector2D.createPolarCoordinates(0, new WrappedAngle(90));
        assertEquals(0, v.getX(), 0.001);
        assertEquals(0, v.getY(), 0.001);
    }

    @Test
    public void testCreatePolarNegativeMagnitude() {
        Vector2D v = Vector2D.createPolarCoordinates(-2, new WrappedAngle(90));
        assertEquals(2, v.getX(), 0.001);
        assertEquals(0, v.getY(), 0.001);
    }

    @Test
    public void testGetMagnitudeCardinal() {
        assertEquals(2., up2.getMagnitude(), 0.001);
    }

    @Test
    public void testGetMagnitudeDiagonal() {
        assertEquals(1., deg30.getMagnitude(), 0.001);
    }

    @Test
    public void testGetAngleUp() {
        assertEquals(0., up2.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleLeft() {
        assertEquals(90., left1.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleRight() {
        assertEquals(-180, down3.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleDown() {
        assertEquals(-90, rightHalf.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleQ1() {
        assertEquals(30, deg30.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleQ2() {
        assertEquals(135, deg135.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleQ3() {
        assertEquals(-135, degNeg135.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testGetAngleQ4() {
        assertEquals(-60, degNeg60.getAngle().asDouble(), 0.001);
    }

    @Test
    public void testPlusCardinals() {
        assertTrue(Vector2D.createCartesianCoordinates(-1, 2).approximatelyEquals(up2.plus(left1)));
    }

    @Test
    public void testPlusDiagonals() {
        assertTrue(Vector2D.createCartesianCoordinates(-1, 2).approximatelyEquals(upRight.plus(downLeft)));
    }

    @Test
    public void testTimesCardinal() {
        assertTrue(Vector2D.createCartesianCoordinates(0, 3).approximatelyEquals(up2.times(1.5)));
    }

    @Test
    public void testTimesDiagonal() {
        assertTrue(Vector2D.createCartesianCoordinates(-2, -6).approximatelyEquals(upRight.times(-2)));
    }

    @Test
    public void testRotatePositive() {
        assertTrue(Vector2D.createPolarCoordinates(1, new WrappedAngle(50)).approximatelyEquals(deg30.rotate(20)));
    }

    @Test
    public void testRotateNegative() {
        assertTrue(Vector2D.createPolarCoordinates(1, new WrappedAngle(-15)).approximatelyEquals(deg30.rotate(-45)));
    }

    @Test
    public void testRotateLarge() {
        assertTrue(Vector2D.createPolarCoordinates(1, new WrappedAngle(40)).approximatelyEquals(deg30.rotate(730)));
    }

    @Test
    public void testApproximatelyEqualsTrue() {
        assertTrue(up2.approximatelyEquals(Vector2D.createPolarCoordinates(2, new WrappedAngle(0))));
    }

    @Test
    public void testApproximatelyEqualsTrueZero() {
        assertTrue(Vector2D.ORIGIN.approximatelyEquals(Vector2D.createPolarCoordinates(0., new WrappedAngle(0.))));
    }

    @Test
    public void testApproximatelyEqualsXMismatch() {
        assertFalse(up2.approximatelyEquals(Vector2D.createCartesianCoordinates(0, 2.001)));
    }

    @Test
    public void testApproximatelyEqualsYMismatch() {
        assertFalse(up2.approximatelyEquals(Vector2D.createCartesianCoordinates(0.001, 2)));
    }

    @Test
    public void testLimitChangeUnrestricted() {
        assertTrue(up2.approximatelyEquals(left1.limitChange(up2, 3)));
    }

    @Test
    public void testLimitChangeRestrictedDown() {
        Vector2D current = Vector2D.createCartesianCoordinates(2, 3);
        Vector2D target = Vector2D.createCartesianCoordinates(2, -3);
        assertTrue(Vector2D.createCartesianCoordinates(2, 0).approximatelyEquals(current.limitChange(target, 3)));
    }

    @Test
    public void testLimitChangeRestrictedUp() {
        Vector2D current = Vector2D.createCartesianCoordinates(2, -3);
        Vector2D target = Vector2D.createCartesianCoordinates(2, 3);
        assertTrue(Vector2D.createCartesianCoordinates(2, 0).approximatelyEquals(current.limitChange(target, 3)));
    }

    @Test
    public void testLimitChangeRestrictedSameAngle() {
        Vector2D current = Vector2D.createPolarCoordinates(5, new WrappedAngle(50));
        Vector2D target = Vector2D.createPolarCoordinates(2, new WrappedAngle(50));
        assertTrue(Vector2D.createPolarCoordinates(4, new WrappedAngle(50)).approximatelyEquals(current.limitChange(target, 1)));
    }

    @Test
    public void testLimitChangeRestrictedOppositveAngle() {
        Vector2D current = Vector2D.createPolarCoordinates(2, new WrappedAngle(60));
        Vector2D target = Vector2D.createPolarCoordinates(5, new WrappedAngle(-120));
        assertTrue(Vector2D.createPolarCoordinates(1, new WrappedAngle(-120)).approximatelyEquals(current.limitChange(target, 3)));
    }

}