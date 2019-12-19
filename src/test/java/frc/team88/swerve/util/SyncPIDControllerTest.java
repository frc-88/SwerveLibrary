package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SyncPIDControllerTest {

    private SyncPIDController controller;

    private final double kP = 100;
    private final double kI = 1;
    private final double kD = 10;
    private final double kF = 1000;
    private final double iZone = 10;
    private final double iMax = 5;
    private final double tolerance = 2;

    @BeforeEach
    public void setup() {
        controller = new SyncPIDController(kP, kI, kD, kF, iZone, iMax);
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
        controller.setIZone(2*iMax);
        assertEquals(2*iMax, controller.getIMax(), 0.0001);
        assertTrue(controller.isIMaxEnabled());
    }

    @Test
    public void testSetTolerance() {
        controller.setTolerance(tolerance);
        assertEquals(tolerance, controller.getTolerance(), 0.0001);
    }

    

}