package frc.team88.swerve.motion.modifiers;

import java.util.function.Function;

import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;

public interface MotionModifier<R> extends Function<MotionState, R> {

    public R applyToFullVelocityState(FullVelocityMotionState state);

}