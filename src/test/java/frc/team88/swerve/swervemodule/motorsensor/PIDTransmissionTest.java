package frc.team88.swerve.swervemodule.motorsensor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.AdditionalMatchers.eq;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

public class PIDTransmissionTest {

    @Mock
    private SwerveMotor inputMotor;

    private PIDTransmission reductionTransmission;
    private PIDTransmission speedUpTransmission;

    private double reductionGearRatio = 10.;
    private double speedUpGearRatio = 1. / 10.;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);

        reductionTransmission = new PIDTransmission(inputMotor, reductionGearRatio);
        speedUpTransmission = new PIDTransmission(inputMotor, speedUpGearRatio);
    }

    @Test
    public void testReductionGetPosition() {
        when(inputMotor.getPosition()).thenReturn(2.);
        assertEquals(0.2, reductionTransmission.getPosition(), 0.000001);
    }

    @Test
    public void testSpeedUpGetPosition() {
        when(inputMotor.getPosition()).thenReturn(2.);
        assertEquals(20., speedUpTransmission.getPosition(), 0.000001);
    }

    @Test
    public void testReductionCalibratePosition() {
        reductionTransmission.calibratePosition(0.1);
        verify(inputMotor).calibratePosition(eq(1., 0.000001));
    }

    @Test
    public void testSpeedUpCalibratePosition() {
        speedUpTransmission.calibratePosition(10.);
        verify(inputMotor).calibratePosition(eq(1., 0.000001));
    }

    @Test
    public void testReductionGetVelocity() {
        when(inputMotor.getVelocity()).thenReturn(5.);
        assertEquals(0.5, reductionTransmission.getVelocity(), 0.000001);
    }

    @Test
    public void testSpeedUpGetVelocity() {
        when(inputMotor.getVelocity()).thenReturn(5.);
        assertEquals(50., speedUpTransmission.getVelocity(), 0.000001);
    }
}