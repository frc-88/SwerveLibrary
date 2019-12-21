package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.util.wpilibwrappers.RobotControllerWrapper;

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
        controller.setKP(2*kP);
        assertEquals(2*kP, controller.getKP(), 0.0001);
    }

    @Test
    public void testSetKI() {
        controller.setKP(2*kP);
        assertEquals(2*kP, controller.getKP(), 0.0001);
    }

    @Test
    public void testSetKD() {
        controller.setKP(2*kP);
        assertEquals(2*kP, controller.getKP(), 0.0001);
    }

    @Test
    public void testSetKF() {
        controller.setKP(2*kP);
        assertEquals(2*kP, controller.getKP(), 0.0001);
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
        controller.setIZone(2*iZone);
        assertEquals(2*iZone, controller.getIZone(), 0.0001);
        assertTrue(controller.isIZoneEnabled());
    }

    @Test
    public void testSetIMax() {
        controller.disableIMax();
        controller.setIMax(2*iMax);
        assertEquals(2*iMax, controller.getIMax(), 0.0001);
        assertTrue(controller.isIMaxEnabled());
    }

    @Test
    public void testSetTolerance() {
        controller.setTolerance(2*tolerance);
        assertEquals(2*tolerance, controller.getTolerance(), 0.0001);
    }

    @Test
    public void testCalculateP() {
        assertEquals(500, controller.calculateP(5));
    }

    @Test
    public void testCalculateISimple() {
        assertEquals(50, controller.calculateI(5));
    }

    @Test
    public void testCalculateIMultipleTimes() {
        assertEquals(50, controller.calculateI(5));
        assertEquals(90, controller.calculateI(4));
    }

    @Test
    public void testCalculateIIZoneDisabled() {
        controller.disableIZone();
        assertEquals(60, controller.calculateI(6));
        assertEquals(90, controller.calculateI(3));
        assertEquals(70, controller.calculateI(-2));
        assertEquals(-60, controller.calculateI(-13));
    }

    @Test
    public void testCalculateIIZoneEnabled() {
        assertEquals(0, controller.calculateI(11));
        assertEquals(30, controller.calculateI(3));
        assertEquals(10, controller.calculateI(-2));
        assertEquals(10, controller.calculateI(-13));
    }

    @Test
    public void testCalculateIIMaxDisabled() {
        controller.disableIMax();
        assertEquals(90, controller.calculateI(9));
        assertEquals(120, controller.calculateI(3));
        assertEquals(140, controller.calculateI(2));
        assertEquals(50, controller.calculateI(-9));
        assertEquals(-40, controller.calculateI(-9));
        assertEquals(-130, controller.calculateI(-9));
    }

    @Test
    public void testCalculateIIMaxEnabled() {
        assertEquals(90, controller.calculateI(9));
        assertEquals(100, controller.calculateI(3));
        assertEquals(100, controller.calculateI(2));
        assertEquals(10, controller.calculateI(-9));
        assertEquals(-80, controller.calculateI(-9));
        assertEquals(-100, controller.calculateI(-5));
    }

    @Test
    public void testCalculateD() {
        when(robotController.getFPGATime())
                .thenReturn(1_000_000l).thenReturn(1_100_000l);
        assertEquals(5, controller.calculateD(5));
        assertEquals(50, controller.calculateD(10));
    }

    @Test
    public void testCalculateF() {
        assertEquals(5000, controller.calculateF(5));
    }

    @Test
    public void testCalculateOutputNormal() {
        when(robotController.getFPGATime())
                .thenReturn(1_000_000l).thenReturn(1_100_000l);
        assertEquals(500 + 50 + 5 + 0, controller.calculateOutput(-5, 0));
        assertEquals(300 + 80 - 20 + 1000, controller.calculateOutput(-2, 1));
    }

    @Test
    public void testCalculateOutputWithTolerance() {
        when(robotController.getFPGATime())
                .thenReturn(1_000_000l).thenReturn(2_000_000l)
                .thenReturn(3_000_000l);
        assertNotEquals(0, controller.calculateOutput(-5, 0));
        assertNotEquals(0, controller.calculateOutput(5, 0));
        assertEquals(0, controller.calculateOutput(1, 0));
        assertEquals(0, controller.calculateOutput(1, 0));
    }

}