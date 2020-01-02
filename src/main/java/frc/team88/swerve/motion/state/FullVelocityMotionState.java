package frc.team88.swerve.motion.state;

import java.util.Objects;

import frc.team88.swerve.motion.modifiers.MotionModifier;
import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.Vector2D;

/**
 * Represents a motion state that is specified by velocity for both translation
 * and heading. Abstract because additional type information such as
 * field-centric or robot-centric is needed, even though all functionaily is
 * fully implemented other than the factory method.
 */
public class FullVelocityMotionState implements MotionState {

    private final boolean isFieldCentric;

    // The translation velocity, as a velocity vector in feet per
    // second.
    private final Vector2D translationVelocity;

    // The rotation velocity, in degrees per second.
    private final double rotationVelocity;

    // The center of rotation, as a position vector from the robot's origin,
    // in feet.
    private final Vector2D centerOfRotation;

    /**
     * Constructor. Protected to force use of factory methods.
     * 
     * @param isFieldCentric
     *                                True if this motion state is field-centric,
     *                                false if it is robot-centric
     * @param translationVelocity
     *                                The translation velocity, as a velocity vector
     *                                in feet per second
     * @param rotationVelocity
     *                                The rotation velocity, in degrees per second
     * @param centerOfRotation
     *                                The center of rotation, as a position vector
     *                                from the robot's origin, in feet
     */
    protected FullVelocityMotionState(boolean isFieldCentric, Vector2D translationVelocity, double rotationVelocity,
            Vector2D centerOfRotation) {
        this.isFieldCentric = isFieldCentric;
        this.translationVelocity = Objects.requireNonNull(translationVelocity);
        this.rotationVelocity = rotationVelocity;
        this.centerOfRotation = Objects.requireNonNull(centerOfRotation);
    }

    /**
     * Factory method for creating a robot-centric motion state with 0 velocities, a
     * center of rotation at the robot's origin, and all other specifications at
     * default.
     * 
     * @return The robot-centric still motion state
     */
    public static FullVelocityMotionState createRobotCentricDefault() {
        return new FullVelocityMotionState(false, Vector2D.ORIGIN, 0, Vector2D.ORIGIN);
    }

    /**
     * Factory method for creating a field-centric motion state with 0 velocities, a
     * center of rotation at the robot's origin, and all other specifications at
     * default.
     * 
     * @return The field-centric default motion state
     */
    public static FullVelocityMotionState createFieldCentricDefault() {
        return new FullVelocityMotionState(true, Vector2D.ORIGIN, 0, Vector2D.ORIGIN);
    }

    @Override
    public <R> R acceptModifier(MotionModifier<R> modifier) {
        return modifier.applyToFullVelocityState(this);
    }

    @Override
    public boolean isFieldCentric() {
        return this.isFieldCentric;
    }

    @Override
    public Vector2D getTranslationVelocity() {
        return this.translationVelocity;
    }

    @Override
    public double getRotationVelocity() {
        return this.rotationVelocity;
    }

    @Override
    public Vector2D getCenterOfRotation() {
        return this.centerOfRotation;
    }

    @Override
    public FullVelocityMotionState changeTranslationVelocity(Vector2D newTranslationVelocity) {
        return new FullVelocityMotionState(isFieldCentric, Objects.requireNonNull(newTranslationVelocity),
                this.rotationVelocity, this.centerOfRotation);
    }

    @Override
    public FullVelocityMotionState changeRotationVelocity(double newRotationVelocity) {
        return new FullVelocityMotionState(isFieldCentric, this.translationVelocity, newRotationVelocity,
                this.centerOfRotation);
    }

    @Override
    public FullVelocityMotionState changeCenterOfRotation(Vector2D newCenterOfRotation) {
        return new FullVelocityMotionState(isFieldCentric, this.translationVelocity, this.rotationVelocity,
                Objects.requireNonNull(newCenterOfRotation));
    }

    @Override
    public FullVelocityMotionState makeRobotCentric() {
        return new FullVelocityMotionState(false, this.translationVelocity, this.rotationVelocity,
                this.centerOfRotation);
    }

    @Override
    public FullVelocityMotionState makeFieldCentric() {
        return new FullVelocityMotionState(true, this.translationVelocity, this.rotationVelocity,
                this.centerOfRotation);
    }
 }
