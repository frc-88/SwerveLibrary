package frc.team88.swerve.motion.state;

import frc.team88.swerve.motion.modifiers.MotionModifier;
import frc.team88.swerve.util.Vector2D;

/**
 * Represents a state of motion for the overall swerve chassis, with varying
 * parameters actually specified.
 */
public interface MotionState {

    /**
     * Visitor accept method for motion modifiers. Should dispatch to the
     * appropriate visitor method for the implementing class.
     * 
     * @return The result of applying the modifier on this motion state
     */
    public <R> R acceptModifier(MotionModifier<R> modifier);

    /**
     * Are the parameters of this motion state field-centric, when applicable?
     * 
     * @return True is field-centric, false if robot-centric
     */
    public boolean isFieldCentric();

    /**
     * Gets the translation velocity.
     * 
     * @return The translation velocity, as a velocity vector in feet per second
     */
    public Vector2D getTranslationVelocity();

    /**
     * Gets the rotation velocity.
     * 
     * @return The rotation velocity, in degrees per second
     */
    public double getRotationVelocity();

    /**
     * Gets the center of rotation.
     * 
     * @return The center of rotation, as a position vector from the robot's origin,
     *         in feet
     * @return The new motion state
     */
    public Vector2D getCenterOfRotation();

    /**
     * Creates a new MotionState that is the same as this one, except with the given
     * translation velocity.
     * 
     * @param newTranslationVelocity
     *                                   The translation velocity to set, as a
     *                                   velocity vector in feet per second
     * @return The new motion state
     */
    public MotionState changeTranslationVelocity(Vector2D newTranslationVelocity);

    /**
     * Creates a new MotionState that is the same as this one, except with the given
     * rotaton velocity.
     * 
     * @param newRotationVelocity
     *                                The rotation velocity to set, in degrees per
     *                                second
     * @return The new motion state
     */
    public MotionState changeRotationVelocity(double newRotationVelocity);

    /**
     * Creates a new MotionState that is the same as this one, except with the given
     * center of rotaton.
     * 
     * @param newCenterOfRotation
     *                                The center of rotation, as a position vector
     *                                from the robot's origin, in feet
     * @return The new motion state
     */
    public MotionState changeCenterOfRotation(Vector2D newCenterOfRotation);

    /**
     * Creates a new MotionState that is the same as this one, except it is denoted
     * as being robot-centric. No actual changes are made to the parameters.
     * 
     * @return The new motion state
     */
    public MotionState makeRobotCentric();

    /**
     * Creates a new MotionState that is the same as this one, except it is denoted
     * as being field-centric. No actual changes are made to the parameters.
     * 
     * @return The new motion state
     */
    public MotionState makeFieldCentric();
}
