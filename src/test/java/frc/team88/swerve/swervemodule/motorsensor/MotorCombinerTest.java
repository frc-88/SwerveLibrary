package frc.team88.swerve.swervemodule.motorsensor;

import static org.mockito.AdditionalMatchers.eq;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.module.motor.SwerveMotor;

public class MotorCombinerTest {

    private MotorCombiner combiner;
    private SwerveMotor output1;
    private SwerveMotor output2;

    private final double[] input1ForwardCoefficients = { 1, 10 };
    private final double[] input2ForwardCoefficients = { 1, -8 };

    @Mock
    SwerveMotor input1;

    @Mock
    SwerveMotor input2;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);

        combiner = new MotorCombiner.Builder(2).addInput(input1, input1ForwardCoefficients)
                .addInput(input2, input2ForwardCoefficients).build();

        output1 = combiner.getOutput(0);
        output2 = combiner.getOutput(1);
    }

    @Test
    public void testGetPositionAdditive() {
        when(input1.getPosition()).thenReturn(2.);
        when(input2.getPosition()).thenReturn(5.);
        assertDoubleEquals(7., output1.getPosition());
    }

    @Test
    public void testGetPositionDifferential() {
        when(input1.getPosition()).thenReturn(2.);
        when(input2.getPosition()).thenReturn(5.);
        assertDoubleEquals(-20., output2.getPosition());
    }

    @Test
    public void testCalibratePositionAdditive() {
        when(input1.getPosition()).thenReturn(2.).thenReturn(10.);
        when(input2.getPosition()).thenReturn(5.).thenReturn(0.);
        output1.calibratePosition(1.);
        assertDoubleEquals(4., output1.getPosition());
    }

    @Test
    public void testCalibratePositionDifferential() {
        when(input1.getPosition()).thenReturn(2.).thenReturn(10.);
        when(input2.getPosition()).thenReturn(5.).thenReturn(0.);
        output2.calibratePosition(20.);
        assertDoubleEquals(140., output2.getPosition());
    }

    @Test
    public void testGetVelocityAdditive() {
        when(input1.getVelocity()).thenReturn(2.);
        when(input2.getVelocity()).thenReturn(5.);
        assertDoubleEquals(7., output1.getVelocity());
    }

    @Test
    public void testGetVelocityDifferential() {
        when(input1.getVelocity()).thenReturn(2.);
        when(input2.getVelocity()).thenReturn(5.);
        assertDoubleEquals(-20., output2.getVelocity());
    }

    @Test
    public void testSetVelocity() {
        // Inverse matrix is {{4/9, 5/9}, {1/18, -1/18}}
        output1.setVelocity(9.);
        verify(input1).setVelocity(eq(4., 0.000001));
        verify(input2).setVelocity(eq(5., 0.000001));

        output2.setVelocity(36.);
        verify(input1).setVelocity(eq(6., 0.000001));
        verify(input2).setVelocity(eq(3., 0.000001));

        output1.setVelocity(-18.);
        verify(input1).setVelocity(eq(-6., 0.000001));
        verify(input2).setVelocity(eq(-12., 0.000001));
    }
}
