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
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.LongPreferenceConstant;
import frc.team88.swerve.wrappers.RobotControllerWrapper;

public class ToHammerModeTest {

    @Mock
    private RobotControllerWrapper robotController;

    @Mock
    private DoublePreferenceConstant hammerAngle;

    @Mock
    private LongPreferenceConstant switchTime;

    private ToHammerMode modifier;

    private final double defaultHammerAngle = 45.;
    private final FullVelocityMotionState inputFullVelocity = FullVelocityMotionState.createRobotCentricDefault()
            .changeTranslationVelocity(Vector2D.createPolarCoordinates(5, new WrappedAngle(90)))
            .changeRotationVelocity(180.).changeCenterOfRotation(Vector2D.createCartesianCoordinates(1, 2));
    private final FullVelocityMotionState evenFullVelocity = inputFullVelocity
            .changeTranslationVelocity(inputFullVelocity.getTranslationVelocity().changeAngle(new WrappedAngle(135.)));
    private final FullVelocityMotionState oddFullVelocity = inputFullVelocity
            .changeTranslationVelocity(inputFullVelocity.getTranslationVelocity().changeAngle(new WrappedAngle(45.)));

    @BeforeEach
    public void setup() {
        MockitoAnnotations.initMocks(this);
        RobotControllerWrapper.setInstance(robotController);

        this.modifier = new ToHammerMode(hammerAngle, switchTime);

        when(hammerAngle.getValue()).thenReturn(defaultHammerAngle);
        when(switchTime.getValue()).thenReturn(1_000_000l);
    }

    @Test
    public void testTime0FullVelocity() {
        // Reset at time 0, modify at time 0
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(0l);
        modifier.reset();

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(evenFullVelocity, modifiedState);
    }

    @Test
    public void testPreSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l);
        modifier.reset();

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(evenFullVelocity, modifiedState);
    }

    @Test
    public void testFirstSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(oddFullVelocity, modifiedState);
    }

    @Test
    public void testPostFirstSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch), modify at time 1,250,000
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l);// .thenReturn(1_250_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);
        // modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(oddFullVelocity, modifiedState);
    }

    @Test
    public void testSecondSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch), modify at time 1,500,000 (twice for switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l).thenReturn(1_500_000l).thenReturn(1_500_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(evenFullVelocity, modifiedState);
    }

    @Test
    public void testPostSecondSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch), modify at time 1,250,000, modify at time 2,250,000 (twice for
        // switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l).thenReturn(1_250_000l).thenReturn(2_250_000l).thenReturn(2_250_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(evenFullVelocity, modifiedState);
    }

    @Test
    public void testThirdSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch), modify at time 2,000,000 (twice for switch), modify at time 3,00,000
        // (twice for switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l).thenReturn(2_000_000l).thenReturn(2_000_000l).thenReturn(3_000_000l)
                .thenReturn(3_000_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(oddFullVelocity, modifiedState);
    }

    @Test
    public void testPostThirdSwitchFullVelocity() {
        // Reset at time 0, modify at time 250,000, modify at time 500,000 (twice for
        // switch), modify at time 2,000,000 (twice for switch), modify at time
        // 2,250,000, modify at time 3,250,000 (twice for switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(250_000l).thenReturn(500_000l)
                .thenReturn(500_000l).thenReturn(2_000_000l).thenReturn(2_000_000l).thenReturn(2_250_000l)
                .thenReturn(3_250_000l).thenReturn(3_250_000l);
        modifier.reset();

        // Build up modifier state
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.applyToFullVelocityState(inputFullVelocity);

        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);

        assertMotionStateEquals(oddFullVelocity, modifiedState);
    }

    @Test
    public void testResetFullVelocity() {
        // Reset at time 0, modify at time 500,000 (twice for switch), reset at
        // 2,000,000, modify at 2,000_000, modify at 2,750,000 (twice for switch)
        when(robotController.getFPGATime()).thenReturn(0l).thenReturn(500_000l).thenReturn(500_000l)
                .thenReturn(2_000_000l).thenReturn(2_000_000l).thenReturn(2_750_000l).thenReturn(2_750_000l);
        modifier.reset();

        // First modification and reset
        modifier.applyToFullVelocityState(inputFullVelocity);
        modifier.reset();

        // Modify immediately
        FullVelocityMotionState modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);
        assertMotionStateEquals(evenFullVelocity, modifiedState);

        // Modify after a switch
        modifiedState = modifier.applyToFullVelocityState(inputFullVelocity);
        assertMotionStateEquals(oddFullVelocity, modifiedState);
    }

}
