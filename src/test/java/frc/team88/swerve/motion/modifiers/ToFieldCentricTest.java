package frc.team88.swerve.motion.modifiers;

import static org.mockito.Mockito.when;

import static frc.team88.swerve.TestUtils.assertMotionStateEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.wrappers.gyro.Gyro;

public class ToFieldCentricTest {

    @Mock
    private Gyro gyro;

    private ToFieldCentric modifier;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);

        modifier = new ToFieldCentric(gyro);
    }

    @Test
    public void testApplyFullVelocityFieldCentric() {
        when(gyro.getYaw()).thenReturn(20.);
        FullVelocityMotionState initialState = FullVelocityMotionState.createFieldCentricDefault()
                .changeTranslationVelocity(Vector2D.createPolarCoordinates(5, new WrappedAngle(50.)))
                .changeRotationVelocity(90.).changeCenterOfRotation(Vector2D.createCartesianCoordinates(1, 2));
        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(initialState);
        FullVelocityMotionState expectedState = initialState;
        assertMotionStateEquals(expectedState, modifiedState);
    }

    @Test
    public void testApplyFullVelocityRobotCentric() {
        when(gyro.getYaw()).thenReturn(20.);
        FullVelocityMotionState initialState = FullVelocityMotionState.createRobotCentricDefault()
                .changeTranslationVelocity(Vector2D.createPolarCoordinates(5, new WrappedAngle(50.)))
                .changeRotationVelocity(90.).changeCenterOfRotation(Vector2D.createCartesianCoordinates(1, 2));
        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(initialState);
        FullVelocityMotionState expectedState = initialState.makeFieldCentric()
                .changeTranslationVelocity(initialState.getTranslationVelocity().changeAngle(new WrappedAngle(70.)));
        assertMotionStateEquals(expectedState, modifiedState);
    }

}
