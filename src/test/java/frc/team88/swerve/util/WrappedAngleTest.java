package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import static frc.team88.swerve.TestUtils.EPSILON;
import static frc.team88.swerve.TestUtils.assertDoubleEquals;

import java.util.ArrayList;
import java.util.List;

import org.javatuples.Pair;
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
                assertEquals(targetAngle.asDouble(), currentAngle.plus(distanceNoHalf).asDouble(), EPSILON,
                        distanceNoHalf + " is not a valid distance from " + currentAngle.asDouble() + " to "
                                + targetAngle.asDouble());
                assertTrue(distanceNoHalf >= -180. && distanceNoHalf < 180.,
                        distanceNoHalf + " is not the shortest distance from " + currentAngle.asDouble() + " to "
                                + targetAngle.asDouble());

                // Test smallest distance to half angle with a bias of 180
                Pair<Double, Boolean> distanceBias180 = currentAngle.getSmallestDifferenceWithHalfAngle(targetAngle,
                        180);
                assertEquals(distanceNoHalf, distanceBias180.getValue0(), EPSILON, distanceBias180.getValue0()
                        + " with bias 180 does not match shortest distance with no half-angle, " + distanceNoHalf
                        + ", from " + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                assertFalse(distanceBias180.getValue1(), "The half angle should not be used when the bias is 180 from "
                        + currentAngle.asDouble() + " to " + targetAngle.asDouble());

                // Test smallest distance to half angle with a bias of 0
                Pair<Double, Boolean> distanceBias0 = currentAngle.getSmallestDifferenceWithHalfAngle(targetAngle, 0);
                assertEquals(WrappedAngle.wrapAngle(distanceNoHalf + 180), distanceBias0.getValue0(), EPSILON,
                        distanceBias0.getValue0() + " with bias 0 does not match shortest distance with no half-angle, "
                                + distanceNoHalf + ", + 180 from " + currentAngle.asDouble() + " to "
                                + targetAngle.asDouble());
                assertTrue(distanceBias0.getValue1(), "The half angle should always be used when the bias is 0 from "
                        + currentAngle.asDouble() + " to " + targetAngle.asDouble());

                // Test smallest distance to half angle with a bias of 90
                Pair<Double, Boolean> distanceBias90 = currentAngle.getSmallestDifferenceWithHalfAngle(targetAngle, 90);
                if (distanceBias90.getValue1()) {
                    assertEquals(targetAngle.plus(180.).asDouble(),
                            currentAngle.plus(distanceBias90.getValue0()).asDouble(), EPSILON,
                            distanceBias90.getValue0()
                                    + " with bias 90 is supposed to be a half-angle, but is not a valid distance from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble() + " + 180");
                } else {
                    assertEquals(distanceNoHalf, distanceBias90.getValue0(), EPSILON, distanceBias90.getValue0()
                            + " with bias 90 is not half-angle, but does not match shortest distance with no half-angle, "
                            + distanceNoHalf + ", from " + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                }
                assertTrue(Math.abs(distanceBias90.getValue0()) <= 90,
                        distanceBias90.getValue0() + " with bias 90 is not the shortest distance from "
                                + currentAngle.asDouble() + " to " + targetAngle.asDouble() + " or it's half-angle");

                // Test smallest distance to half angle with a bias of 135
                Pair<Double, Boolean> distanceBias135 = currentAngle.getSmallestDifferenceWithHalfAngle(targetAngle,
                        135);
                if (distanceBias135.getValue1()) {
                    assertEquals(targetAngle.plus(180.).asDouble(),
                            currentAngle.plus(distanceBias135.getValue0()).asDouble(), EPSILON,
                            distanceBias135.getValue0()
                                    + " with bias 135 is supposed to be a half-angle, but is not a valid distance from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble() + " + 180");
                    assertTrue(Math.abs(distanceBias135.getValue0()) <= 45,
                            distanceBias135.getValue0()
                                    + " with bias 135 should not have gone to the half-angle to go from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                } else {
                    assertEquals(distanceNoHalf, distanceBias135.getValue0(), EPSILON, distanceBias135.getValue0()
                            + " with bias 135 is not half-angle, but does not match shortest distance with no half-angle, "
                            + distanceNoHalf + ", from " + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                    assertTrue(Math.abs(distanceBias135.getValue0()) <= 135,
                            distanceBias135.getValue0()
                                    + " with bias 135 should not have gone to the full-angle to go from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                }

                // Test smallest distance to half angle with a bias of 45
                Pair<Double, Boolean> distanceBias45 = currentAngle.getSmallestDifferenceWithHalfAngle(targetAngle, 45);
                if (distanceBias45.getValue1()) {
                    assertEquals(targetAngle.plus(180.).asDouble(),
                            currentAngle.plus(distanceBias45.getValue0()).asDouble(), EPSILON,
                            distanceBias45.getValue0()
                                    + " with bias 45 is supposed to be a half-angle, but is not a valid distance from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble() + " + 180");
                    assertTrue(Math.abs(distanceBias45.getValue0()) <= 135,
                            distanceBias45.getValue0()
                                    + " with bias 45 should not have gone to the half-angle to go from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                } else {
                    assertEquals(distanceNoHalf, distanceBias45.getValue0(), EPSILON, distanceBias45.getValue0()
                            + " with bias 45 is not half-angle, but does not match shortest distance with no half-angle, "
                            + distanceNoHalf + ", from " + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                    assertTrue(Math.abs(distanceBias45.getValue0()) <= 45,
                            distanceBias45.getValue0()
                                    + " with bias 45 should not have gone to the full-angle to go from "
                                    + currentAngle.asDouble() + " to " + targetAngle.asDouble());
                }
            }
        }
    }

}
