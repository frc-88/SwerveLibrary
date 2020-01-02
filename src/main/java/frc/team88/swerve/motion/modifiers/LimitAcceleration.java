package frc.team88.swerve.motion.modifiers;

import java.util.function.Supplier;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;

/**
 * Applies an acceleration limit to both the translation and rotaton of a motion
 * state.
 */
public class LimitAcceleration implements MotionModifier<MotionState> {

    // The translation acceleration limit, in feet per second
    private final DoublePreferenceConstant translationLimit;

    // The rotation acceleration limit, in degrees per second
    private final DoublePreferenceConstant rotationLimit;

    // A function to get the previous full velocity motion state.
    private final Supplier<MotionState> previousMotionStateSupplier;

    // The expected rate at which this will be called, in Hz.
    private final double expectedUpdateRate;

    public LimitAcceleration(DoublePreferenceConstant translationLimit, DoublePreferenceConstant rotationLimit,
            Supplier<MotionState> previousMotionStateSupplier, double expectedUpdateRate) {
        this.translationLimit = translationLimit;
        this.rotationLimit = rotationLimit;
        this.previousMotionStateSupplier = previousMotionStateSupplier;
        this.expectedUpdateRate = expectedUpdateRate;
    }

    @Override
    public MotionState apply(MotionState state) {
        return state.acceptModifier(this);
    }

    @Override
    public FullVelocityMotionState applyToFullVelocityState(FullVelocityMotionState state) {
        FullVelocityMotionState newState = state;

        // Apply the translation limit
        newState = newState.changeTranslationVelocity(this.limitTranslationVelocity(newState.getTranslationVelocity()));

        // Apply the rotation limit
        newState = newState.changeRotationVelocity(this.limitRotationVelocity(newState.getRotationVelocity()));

        return newState;
    }

    /**
     * Limits the translation velocity of the given target vector such that the
     * difference between it and the vector from the previous motion state have some
     * maximum difference.
     * 
     * @param targetVelocity
     *                           The target velocity
     * @return The target velocity or capped velocity
     */
    protected Vector2D limitTranslationVelocity(Vector2D targetVelocity) {
        // Calculate the change limit for translation
        double translationChangeLimit = translationLimit.getValue() / this.expectedUpdateRate;
        // Calculate the acceleration limited translation
        return previousMotionStateSupplier.get().getTranslationVelocity().limitChange(targetVelocity,
                translationChangeLimit);
    }

    /**
     * Limits the rotation of the given value such that the difference between it
     * and the rotation from the previous motion state have some maximum difference.
     * 
     * @param targetVelocity
     *                           The target velocity
     * @return The target velocity or capped velocity
     */
    protected double limitRotationVelocity(double targetVelocity) {
        // Calculate the change limit for rotation
        double rotationChangeLimit = this.rotationLimit.getValue() / this.expectedUpdateRate;
        // Calculate the acceleration limited rotation
        return MathUtils.limitChange(previousMotionStateSupplier.get().getRotationVelocity(), targetVelocity,
                rotationChangeLimit);
    }

}