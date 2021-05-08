package frc.team88.swerve.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class TrapezoidalProfileControllerTest {

    @Mock
    private SyncPIDController positionPID;

    @Mock
    private RobotControllerWrapper robotController;

    private TrapezoidalProfileController controller;

    private final double MAX_SPEED = 10;
    private final double MAX_ACCELERATION = 100;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);
        RobotControllerWrapper.setInstance(robotController);

        this.controller = new TrapezoidalProfileController(MAX_SPEED, MAX_ACCELERATION, positionPID);
        when(robotController.getFPGATime()).thenReturn(0l);
        this.controller.reset(0);
        this.controller.setTargetPosition(0);
        this.controller.setTargetVelocity(0);
   }

    @Test
    public void testCalculateAcceleratedVelocityForwards() {
        when(robotController.getFPGATime()).thenReturn(10_000l);
        assertEquals(2., this.controller.calculateAcceleratedVelocity(1, true));
    }

    @Test
    public void testCalculateAcceleratedVelocityReverse() {
        when(robotController.getFPGATime()).thenReturn(10_000l);
        assertEquals(1., this.controller.calculateAcceleratedVelocity(2, false));
    }

    @Test
    public void testApplyMaxSpeedLimitForwardsUnlimited() {
        assertEquals(8., this.controller.applyMaxSpeedLimit(8.));
    }

    @Test
    public void testApplyMaxSpeedLimitForwardsLimited() {
        assertEquals(10., this.controller.applyMaxSpeedLimit(10.1));
    }

    @Test
    public void testApplyMaxSpeedLimitReverseUnlimited() {
        assertEquals(-8., this.controller.applyMaxSpeedLimit(-8.));
    }

    @Test
    public void testApplyMaxSpeedLimitReverseLimited() {
        assertEquals(-10., this.controller.applyMaxSpeedLimit(-10.1));
        
    }

    @Test
    public void testApplyDeccelerationLimitForwardsBeforeDecceleration() {
        this.controller.setTargetPosition(10.);
        this.controller.setTargetVelocity(1);
        assertEquals(5., this.controller.applyDeccelerationLimit(5., true));
    }

    @Test
    public void testApplyDeccelerationLimitForwardsDuringDecceleration() {
        this.controller.setTargetPosition(.1);
        this.controller.setTargetVelocity(1);
        // Calculated with https://www.calculatorsoup.com/calculators/physics/uniformly-accelerated-motion-calculator.php
        assertEquals(4.58257569, this.controller.applyDeccelerationLimit(5., true), 0.0001);
    }

    @Test
    public void testApplyDeccelerationLimitForwardsBeforeTargetVelocityFar() {
        this.controller.setTargetPosition(10);
        this.controller.setTargetVelocity(1);
        assertEquals(-1., this.controller.applyDeccelerationLimit(-1., true));
    }

    @Test
    public void testApplyDeccelerationLimitForwardsBeforeTargetVelocityNear() {
        this.controller.setTargetPosition(.1);
        this.controller.setTargetVelocity(1);
        assertEquals(-10., this.controller.applyDeccelerationLimit(-10., true));
    }

    @Test
    public void testApplyDeccelerationLimitReverseBeforeDecceleration() {
        this.controller.setTargetPosition(-10.);
        this.controller.setTargetVelocity(-1);
        assertEquals(-5., this.controller.applyDeccelerationLimit(-5., false));
    }

    @Test
    public void testApplyDeccelerationLimitReverseDuringDecceleration() {
        this.controller.setTargetPosition(-.1);
        this.controller.setTargetVelocity(-1);
        // Calculated with https://www.calculatorsoup.com/calculators/physics/uniformly-accelerated-motion-calculator.php
        assertEquals(-4.58257569, this.controller.applyDeccelerationLimit(-5., false), 0.0001);
    }

    @Test
    public void testApplyDeccelerationLimitReverseBeforeTargetVelocityFar() {
        this.controller.setTargetPosition(-10);
        this.controller.setTargetVelocity(-1);
        assertEquals(1., this.controller.applyDeccelerationLimit(1., false));
    }

    @Test
    public void testApplyDeccelerationLimitReverseBeforeTargetVelocityNear() {
        this.controller.setTargetPosition(-.1);
        this.controller.setTargetVelocity(-1);
        assertEquals(10., this.controller.applyDeccelerationLimit(10., false));
    }

    @Test
    public void testCalculateCommandPosition() {
        when(robotController.getFPGATime()).thenReturn(100_000l);
        assertEquals(1., this.controller.calculateCommandPosition(10.));
    }
}
