package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.wrappers.RobotControllerWrapper;

public class SyncPIDControllerTest {

    private SyncPIDController controller;

    private final double kP = 100;
    private final double kI = 10;
    private final double kD = 1;
    private final double kF = 1000;
    private final double iZone = 10;
    private final double iMax = 100;
    private final double tolerance = 2;

    @Mock
    RobotControllerWrapper robotController;

    @BeforeEach
    public void setup() {
        controller = new SyncPIDController(kP, kI, kD, kF, iZone, iMax, tolerance);

        MockitoAnnotations.initMocks(this);

        RobotControllerWrapper.setInstance(robotController);
    }

    @Test
    public void testSetKP() {
        controller.setKP(2 * kP);
        assertDoubleEquals(2 * kP, controller.getKP());
    }

    @Test
    public void testSetKI() {
        controller.setKP(2 * kP);
        assertDoubleEquals(2 * kP, controller.getKP());
    }

    @Test
    public void testSetKD() {
        controller.setKP(2 * kP);
        assertDoubleEquals(2 * kP, controller.getKP());
    }

    @Test
    public void testSetKF() {
        controller.setKP(2 * kP);
        assertDoubleEquals(2 * kP, controller.getKP());
    }

    @Test
    public void testDisableIZone() {
        controller.disableIZone();
        assertFalse(controller.isIZoneEnabled());
    }

    @Test
    public void testDisableIMax() {
        controller.disableIMax();
        assertFalse(controller.isIMaxEnabled());
    }

    @Test
    public void testSetIZone() {
        controller.disableIZone();
        controller.setIZone(2 * iZone);
        assertDoubleEquals(2 * iZone, controller.getIZone());
        assertTrue(controller.isIZoneEnabled());
    }

    @Test
    public void testSetIMax() {
        controller.disableIMax();
        controller.setIMax(2 * iMax);
        assertDoubleEquals(2 * iMax, controller.getIMax());
        assertTrue(controller.isIMaxEnabled());
    }

    @Test
    public void testSetTolerance() {
        controller.setTolerance(2. * tolerance);
        assertDoubleEquals(2. * tolerance, controller.getTolerance());
    }

    @Test
    public void testCalculateP() {
        assertDoubleEquals(500., controller.calculateP(5.));
    }

    @Test
    public void testCalculateISimple() {
        assertDoubleEquals(50., controller.calculateI(5.));
    }

    @Test
    public void testCalculateIMultipleTimes() {
        assertDoubleEquals(50., controller.calculateI(5.));
        assertDoubleEquals(90., controller.calculateI(4.));
    }

    @Test
    public void testCalculateIIZoneDisabled() {
        controller.disableIZone();
        assertDoubleEquals(60., controller.calculateI(6.));
        assertDoubleEquals(90., controller.calculateI(3.));
        assertDoubleEquals(70., controller.calculateI(-2.));
        assertDoubleEquals(-60., controller.calculateI(-13.));
    }

    @Test
    public void testCalculateIIZoneEnabled() {
        assertDoubleEquals(0., controller.calculateI(11.));
        assertDoubleEquals(30., controller.calculateI(3.));
        assertDoubleEquals(10., controller.calculateI(-2.));
        assertDoubleEquals(10., controller.calculateI(-13.));
    }

    @Test
    public void testCalculateIIMaxDisabled() {
        controller.disableIMax();
        assertDoubleEquals(90., controller.calculateI(9.));
        assertDoubleEquals(120., controller.calculateI(3.));
        assertDoubleEquals(140., controller.calculateI(2.));
        assertDoubleEquals(50., controller.calculateI(-9.));
        assertDoubleEquals(-40., controller.calculateI(-9.));
        assertDoubleEquals(-130., controller.calculateI(-9.));
    }

    @Test
    public void testCalculateIIMaxEnabled() {
        assertDoubleEquals(90., controller.calculateI(9.));
        assertDoubleEquals(100., controller.calculateI(3.));
        assertDoubleEquals(100., controller.calculateI(2.));
        assertDoubleEquals(10., controller.calculateI(-9.));
        assertDoubleEquals(-80., controller.calculateI(-9.));
        assertDoubleEquals(-100., controller.calculateI(-5.));
    }

    @Test
    public void testCalculateD() {
        when(robotController.getFPGATime()).thenReturn(1_000_000l).thenReturn(1_100_000l);
        assertDoubleEquals(5., controller.calculateD(5.));
        assertDoubleEquals(50., controller.calculateD(10.));
    }

    @Test
    public void testCalculateF() {
        assertDoubleEquals(5000., controller.calculateF(5.));
    }

    @Test
    public void testCalculateOutputNormal() {
        when(robotController.getFPGATime()).thenReturn(1_000_000l).thenReturn(1_100_000l);
        assertEquals(500. + 50. + 5. + 0., controller.calculateOutput(-5., 0.));
        assertEquals(300. + 80. - 20. + 1000., controller.calculateOutput(-2., 1.));
    }

    @Test
    public void testCalculateOutputWithTolerance() {
        when(robotController.getFPGATime()).thenReturn(1_000_000l).thenReturn(2_000_000l).thenReturn(3_000_000l);
        assertNotEquals(0., controller.calculateOutput(-5., 0.));
        assertNotEquals(0., controller.calculateOutput(5., 0.));
        assertEquals(0., controller.calculateOutput(1., 0.));
        assertEquals(0., controller.calculateOutput(1., 0.));
    }

}
