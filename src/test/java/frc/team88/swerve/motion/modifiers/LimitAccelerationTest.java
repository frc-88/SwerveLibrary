package frc.team88.swerve.motion.modifiers;

import static org.mockito.Mockito.when;

import static frc.team88.swerve.TestUtils.assertDoubleEquals;
import static frc.team88.swerve.TestUtils.assertVectorEquals;
import static frc.team88.swerve.TestUtils.assertMotionStateEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;

public class LimitAccelerationTest {

    @Mock
    private DoublePreferenceConstant translationLimit;

    @Mock
    private DoublePreferenceConstant rotationLimit;

    private LimitAcceleration modifier;

    private Vector2D previousTranslation = Vector2D.ORIGIN;
    private double previousRotation = 0.;

    private final double EXPECTED_UPDATE_RATE = 10.; // Hz

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);

        modifier = new LimitAcceleration(translationLimit, rotationLimit,
                () -> FullVelocityMotionState.createRobotCentricDefault()
                        .changeTranslationVelocity(this.previousTranslation)
                        .changeRotationVelocity(this.previousRotation)
                        .changeCenterOfRotation(Vector2D.createCartesianCoordinates(1, 2)),
                EXPECTED_UPDATE_RATE);

        when(translationLimit.getValue()).thenReturn(10. * EXPECTED_UPDATE_RATE);
        when(rotationLimit.getValue()).thenReturn(45. * EXPECTED_UPDATE_RATE);
    }

    @Test
    public void testLimitTranslationVelocityUnrestricted() {
        this.previousTranslation = Vector2D.createPolarCoordinates(5, new WrappedAngle(45));
        Vector2D target = Vector2D.createPolarCoordinates(4, new WrappedAngle(30));
        assertVectorEquals(target, modifier.limitTranslationVelocity(target));
    }

    @Test
    public void testLimitTranslationVelocityRestrictedDown() {
        this.previousTranslation = Vector2D.createCartesianCoordinates(2, 10);
        Vector2D target = Vector2D.createCartesianCoordinates(2, -5);
        assertVectorEquals(Vector2D.createCartesianCoordinates(2, 0), modifier.limitTranslationVelocity(target));
    }

    @Test
    public void testLimitTranslationVelocityRestrictedUp() {
        this.previousTranslation = Vector2D.createCartesianCoordinates(2, -5);
        Vector2D target = Vector2D.createCartesianCoordinates(2, 10);
        assertVectorEquals(Vector2D.createCartesianCoordinates(2, 5), modifier.limitTranslationVelocity(target));
    }

    @Test
    public void testLimitTranslationVelocityRestrictedSameAngle() {
        this.previousTranslation = Vector2D.createPolarCoordinates(15, new WrappedAngle(50));
        Vector2D target = Vector2D.createPolarCoordinates(2, new WrappedAngle(50));
        assertVectorEquals(Vector2D.createPolarCoordinates(5, new WrappedAngle(50)),
                modifier.limitTranslationVelocity(target));
    }

    @Test
    public void testLimitTranslationVelocityRestrictedOppositeAngle() {
        this.previousTranslation = Vector2D.createPolarCoordinates(8, new WrappedAngle(60));
        Vector2D target = Vector2D.createPolarCoordinates(5, new WrappedAngle(-120));
        assertVectorEquals(Vector2D.createPolarCoordinates(2, new WrappedAngle(-120)),
                modifier.limitTranslationVelocity(target));
    }

    @Test
    public void testLimitRotationVelocityIncreaseUnrestricted() {
        this.previousRotation = 90.;
        assertDoubleEquals(120., modifier.limitRotationVelocity(120.));
    }

    @Test
    public void testLimitRotationVelocityIncreaseRestricted() {
        this.previousRotation = 90.;
        assertDoubleEquals(135., modifier.limitRotationVelocity(180.));
    }

    @Test
    public void testLimitRotationVelocityDecreaseUnrestricted() {
        this.previousRotation = 90.;
        assertDoubleEquals(60., modifier.limitRotationVelocity(60.));
    }

    @Test
    public void testLimitRotationVelocityDecreaseRestricted() {
        this.previousRotation = 90.;
        assertDoubleEquals(45., modifier.limitRotationVelocity(0.));
    }

    @Test
    public void testFullVelocityLimiting() {
        this.previousTranslation = Vector2D.createCartesianCoordinates(15, 4);
        this.previousRotation = -180;

        FullVelocityMotionState target = FullVelocityMotionState.createRobotCentricDefault()
                .changeTranslationVelocity(Vector2D.createCartesianCoordinates(0, 4)).changeRotationVelocity(90)
                .changeCenterOfRotation(Vector2D.createCartesianCoordinates(3, -5));
        FullVelocityMotionState limited = modifier.applyToFullVelocityState(target);
        FullVelocityMotionState expected = target.changeTranslationVelocity(Vector2D.createCartesianCoordinates(5, 4))
                .changeRotationVelocity(-135);

        assertMotionStateEquals(expected, limited);
    }
}
