package frc.team88.swerve.motion.modifiers;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.LongPreferenceConstant;
import frc.team88.swerve.wrappers.RobotControllerWrapper;

/**
 * Changes the motion state into hammer mode, which uses a side to side motion
 * to push opponents at an angle, such that we can always be traction limited
 * even with a very high geared drivetrain. Reset() should be called before
 * every first use.
 */
public class ToHammerMode implements MotionModifier<MotionState> {

    // The angle to offset from the target translation angle
    private DoublePreferenceConstant hammerAngle;

    // The time to spend hammering in each direction
    private LongPreferenceConstant switchTime;

    // The time when the hammer direction was last changed, in microseconds.
    private long lastHammerModeChangeTime = 0;

    // Count of how many times hammer mode has changed direction since the last
    // reset.
    private int hammerModeChangeCount = 0;

    /**
     * Constructor.
     * 
     * @param hammerAngle
     *                        The angle to offset from the target translation angle
     * @param switchTime
     *                        The time to spend hammering in each direction
     */
    public ToHammerMode(DoublePreferenceConstant hammerAngle, LongPreferenceConstant switchTime) {
        this.hammerAngle = hammerAngle;
        this.switchTime = switchTime;
    }

    @Override
    public MotionState apply(MotionState state) {
        return state.acceptModifier(this);
    }

    @Override
    public MotionState applyToFullVelocityState(FullVelocityMotionState state) {
        // Halve the time to change if this is the first hammer
        long minTimeToChange = switchTime.getValue();
        if (hammerModeChangeCount == 0) {
            minTimeToChange /= 2;
        }

        // Check if it is time to change
        if ((RobotControllerWrapper.getInstance().getFPGATime() - lastHammerModeChangeTime) > minTimeToChange) {
            hammerModeChangeCount++;
            lastHammerModeChangeTime = RobotControllerWrapper.getInstance().getFPGATime();
        }

        // Rotate the vector by the offset in the right direction
        double angleToRotateVector = hammerAngle.getValue();
        if (hammerModeChangeCount % 2 == 1) {
            angleToRotateVector *= -1;
        }
        return state.changeTranslationVelocity(state.getTranslationVelocity().rotate(angleToRotateVector));
    }

    /**
     * Reset hammer mode to it's initial state. Should be called right before the
     * first call to apply in an application of hammer mode, because it handles
     * timing.
     */
    public void reset() {
        this.hammerModeChangeCount = 0;
        this.lastHammerModeChangeTime = RobotControllerWrapper.getInstance().getFPGATime();
    }
}