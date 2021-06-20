package frc.team88.swerve.util;

import static frc.team88.swerve.TestUtils.EPSILON;
import static frc.team88.swerve.TestUtils.assertDoubleEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class WrappedAngleTest {

  private WrappedAngle angleNeg180;
  private WrappedAngle angleNeg120;
  private WrappedAngle angleNeg90;
  private WrappedAngle angleNeg60;
  private WrappedAngle angle0;
  private WrappedAngle angle60;
  private WrappedAngle angle90;
  private WrappedAngle angle120;

  private List<WrappedAngle> allAngles;

  @BeforeEach
  public void setup() {
    this.angleNeg180 = new WrappedAngle(-180);
    this.angleNeg120 = new WrappedAngle(-120);
    this.angleNeg90 = new WrappedAngle(-90);
    this.angleNeg60 = new WrappedAngle(-60);
    this.angle0 = new WrappedAngle(0);
    this.angle60 = new WrappedAngle(60);
    this.angle90 = new WrappedAngle(90);
    this.angle120 = new WrappedAngle(120);

    allAngles = new ArrayList<>();
    allAngles.add(angleNeg180);
    allAngles.add(angleNeg120);
    allAngles.add(angleNeg90);
    allAngles.add(angleNeg60);
    allAngles.add(angle0);
    allAngles.add(angle60);
    allAngles.add(angle90);
    allAngles.add(angle120);
  }

  @Test
  public void testWrapAngle0() {
    assertDoubleEquals(0, WrappedAngle.wrapAngle(0));
  }

  @Test
  public void testWrapAngleNeg() {
    assertDoubleEquals(-90, WrappedAngle.wrapAngle(-90));
  }

  @Test
  public void testWrapAnglePos() {
    assertDoubleEquals(120, WrappedAngle.wrapAngle(120));
  }

  @Test
  public void testWrapAngleNeg180() {
    assertDoubleEquals(-180, WrappedAngle.wrapAngle(-180));
  }

  @Test
  public void testWrapAngle180() {
    assertDoubleEquals(-180, WrappedAngle.wrapAngle(180));
  }

  @Test
  public void testWrapAngleOver180() {
    assertDoubleEquals(-90, WrappedAngle.wrapAngle(270));
  }

  @Test
  public void testWrapAngleUnderNeg180() {
    assertDoubleEquals(60, WrappedAngle.wrapAngle(-300));
  }

  @Test
  public void testWrapAngleFarOver180() {
    assertDoubleEquals(20, WrappedAngle.wrapAngle(1100));
  }

  @Test
  public void testWrapAngleFarUnderNeg180() {
    assertDoubleEquals(-20, WrappedAngle.wrapAngle(-1100));
  }

  @Test
  public void testAngleDistances() {
    for (WrappedAngle currentAngle : this.allAngles) {
      for (WrappedAngle targetAngle : this.allAngles) {
        // Test smallest distance
        double distanceNoHalf = currentAngle.getSmallestDifferenceWith(targetAngle);
        assertEquals(
            targetAngle.asDouble(),
            currentAngle.plus(distanceNoHalf).asDouble(),
            EPSILON,
            distanceNoHalf
                + " is not a valid distance from "
                + currentAngle.asDouble()
                + " to "
                + targetAngle.asDouble());
        assertTrue(
            distanceNoHalf >= -180. && distanceNoHalf < 180.,
            distanceNoHalf
                + " is not the shortest distance from "
                + currentAngle.asDouble()
                + " to "
                + targetAngle.asDouble());
      }
    }
  }
}
