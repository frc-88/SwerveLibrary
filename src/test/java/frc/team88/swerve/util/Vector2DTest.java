package frc.team88.swerve.util;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;
import static frc.team88.swerve.TestUtils.assertVectorEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class Vector2DTest {

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
    up2 = Vector2D.createCartesianCoordinates(2, 0);
    left1 = Vector2D.createCartesianCoordinates(0, 1);
    down3 = Vector2D.createCartesianCoordinates(-3, 0);
    rightHalf = Vector2D.createCartesianCoordinates(0, -0.5);
    upRight = Vector2D.createCartesianCoordinates(3, -1);
    downLeft = Vector2D.createCartesianCoordinates(-1, 2);
    deg30 = Vector2D.createPolarCoordinates(1, new WrappedAngle(30));
    deg135 = Vector2D.createPolarCoordinates(1, new WrappedAngle(135));
    degNeg60 = Vector2D.createPolarCoordinates(1, new WrappedAngle(-60));
    degNeg135 = Vector2D.createPolarCoordinates(2, new WrappedAngle(-135));
  }

  @Test
  public void testCreateCartesian() {
    Vector2D v = Vector2D.createCartesianCoordinates(3, 5);
    assertDoubleEquals(3., v.getX());
    assertDoubleEquals(5., v.getY());
  }

  @Test
  public void testCreatePolarQ1() {
    Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(30));
    assertDoubleEquals(Math.sqrt(3), v.getX());
    assertDoubleEquals(1, v.getY());
  }

  @Test
  public void testCreatePolarQ2() {
    Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(120));
    assertDoubleEquals(-1, v.getX());
    assertDoubleEquals(Math.sqrt(3), v.getY());
  }

  @Test
  public void testCreatePolarQ3() {
    Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(-60));
    assertDoubleEquals(1, v.getX());
    assertDoubleEquals(-Math.sqrt(3), v.getY());
  }

  @Test
  public void testCreatePolarQ4() {
    Vector2D v = Vector2D.createPolarCoordinates(2, new WrappedAngle(-150));
    assertDoubleEquals(-Math.sqrt(3), v.getX());
    assertDoubleEquals(-1, v.getY());
  }

  @Test
  public void testCreatePolarOrigin() {
    Vector2D v = Vector2D.createPolarCoordinates(0, new WrappedAngle(90));
    assertDoubleEquals(0, v.getX());
    assertDoubleEquals(0, v.getY());
  }

  @Test
  public void testCreatePolarNegativeMagnitude() {
    Vector2D v = Vector2D.createPolarCoordinates(-2, new WrappedAngle(90));
    assertDoubleEquals(0, v.getX());
    assertDoubleEquals(-2, v.getY());
  }

  @Test
  public void testGetMagnitudeCardinal() {
    assertDoubleEquals(2., up2.getMagnitude());
  }

  @Test
  public void testGetMagnitudeDiagonal() {
    assertDoubleEquals(1., deg30.getMagnitude());
  }

  @Test
  public void testGetAngleUp() {
    assertDoubleEquals(0., up2.getAngle().asDouble());
  }

  @Test
  public void testGetAngleLeft() {
    assertDoubleEquals(90., left1.getAngle().asDouble());
  }

  @Test
  public void testGetAngleRight() {
    assertDoubleEquals(-180, down3.getAngle().asDouble());
  }

  @Test
  public void testGetAngleDown() {
    assertDoubleEquals(-90, rightHalf.getAngle().asDouble());
  }

  @Test
  public void testGetAngleQ1() {
    assertDoubleEquals(30, deg30.getAngle().asDouble());
  }

  @Test
  public void testGetAngleQ2() {
    assertDoubleEquals(135, deg135.getAngle().asDouble());
  }

  @Test
  public void testGetAngleQ3() {
    assertDoubleEquals(-135, degNeg135.getAngle().asDouble());
  }

  @Test
  public void testGetAngleQ4() {
    assertDoubleEquals(-60, degNeg60.getAngle().asDouble());
  }

  @Test
  public void testPlusCardinals() {
    assertVectorEquals(Vector2D.createCartesianCoordinates(2, 1), up2.plus(left1));
  }

  @Test
  public void testPlusDiagonals() {
    assertVectorEquals(Vector2D.createCartesianCoordinates(2, 1), upRight.plus(downLeft));
  }

  @Test
  public void testTimesCardinal() {
    assertVectorEquals(Vector2D.createCartesianCoordinates(3, 0), up2.times(1.5));
  }

  @Test
  public void testTimesDiagonal() {
    assertVectorEquals(Vector2D.createCartesianCoordinates(-6, 2), upRight.times(-2));
  }

  @Test
  public void testRotatePositive() {
    assertVectorEquals(Vector2D.createPolarCoordinates(1, new WrappedAngle(50)), deg30.rotate(20));
  }

  @Test
  public void testRotateNegative() {
    assertVectorEquals(
        Vector2D.createPolarCoordinates(1, new WrappedAngle(-15)), deg30.rotate(-45));
  }

  @Test
  public void testRotateLarge() {
    assertVectorEquals(Vector2D.createPolarCoordinates(1, new WrappedAngle(40)), deg30.rotate(730));
  }

  @Test
  public void testLimitChangeUnrestricted() {
    assertVectorEquals(up2, left1.limitChange(up2, 3));
  }

  @Test
  public void testLimitChangeRestrictedDown() {
    Vector2D current = Vector2D.createCartesianCoordinates(2, 3);
    Vector2D target = Vector2D.createCartesianCoordinates(2, -3);
    assertVectorEquals(Vector2D.createCartesianCoordinates(2, 0), current.limitChange(target, 3));
  }

  @Test
  public void testLimitChangeRestrictedUp() {
    Vector2D current = Vector2D.createCartesianCoordinates(2, -3);
    Vector2D target = Vector2D.createCartesianCoordinates(2, 3);
    assertVectorEquals(Vector2D.createCartesianCoordinates(2, 0), current.limitChange(target, 3));
  }

  @Test
  public void testLimitChangeRestrictedSameAngle() {
    Vector2D current = Vector2D.createPolarCoordinates(5, new WrappedAngle(50));
    Vector2D target = Vector2D.createPolarCoordinates(2, new WrappedAngle(50));
    assertVectorEquals(
        Vector2D.createPolarCoordinates(4, new WrappedAngle(50)), current.limitChange(target, 1));
  }

  @Test
  public void testLimitChangeRestrictedOppositeAngle() {
    Vector2D current = Vector2D.createPolarCoordinates(2, new WrappedAngle(60));
    Vector2D target = Vector2D.createPolarCoordinates(5, new WrappedAngle(-120));
    assertVectorEquals(
        Vector2D.createPolarCoordinates(1, new WrappedAngle(-120)), current.limitChange(target, 3));
  }
}
