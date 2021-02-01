package frc.team88.swerve.motion.modifiers;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.wrappers.gyro.Gyro;

/**
 * Converts a field-centric motion state to a robot-centric motion state using
 * the supplied gyro. Will not modify a robot-centric motion state.
 */
public class ToRobotCentric implements MotionModifier<MotionState> {

    // The gyro to use for the conversion
    private Gyro gyro;

    /**
     * Constructor.
     * 
     * @param gyro
     *                 The gyro to use for conversion
     */
    public ToRobotCentric(Gyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public MotionState apply(MotionState state) {
        return state.acceptModifier(this);
    }

    @Override
    public FullVelocityMotionState applyToFullVelocityState(FullVelocityMotionState state) {
        // if (!state.isFieldCentric()) {
        //     // Already robot-centric, nothing to do
        //     return state;
        // }

        Vector2D currentTranslationVelocity = state.getTranslationVelocity();
        WrappedAngle newAngle = currentTranslationVelocity.getAngle().plus(-gyro.getYaw());
        return state.changeTranslationVelocity(currentTranslationVelocity.changeAngle(newAngle)).makeRobotCentric();
    }

}