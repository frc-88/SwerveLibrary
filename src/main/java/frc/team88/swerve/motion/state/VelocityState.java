package frc.team88.swerve.motion.state;

import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Represents a desired state of motion, including the translational velocity.
 */
public class VelocityState {
    
    private final double translationDirection;
    private final double translationSpeed;
    private final double rotationVelocity;
    private final double centerOfRotationX;
    private final double centerOfRotationY;
    private final boolean isFieldCentric;

    
    /**
     * Constructs this velocity state with all possible parameters.
     * 
     * @param translationDirection The direction to translate, in degrees
     *                             counting counterclockise from forwards.
     * @param translationSpeed The speed to translate, in feet per second.
     * @param rotationVelocity The angular velocity to rotate, in degrees
     *                         per second.
     * @param centerOfRotationX The x component of the point to rotate about
     *                          relative to the robot's origin, in feet.
     * @param centerOfRotationY The y component of the point to rotate about
     *                          relative to the robot's origin, in feet.
     * @param isFieldCentric True if this state is field-centric, false if it
     *                       is robot-centric.
     */
    public VelocityState(double translationDirection, double translationSpeed, double rotationVelocity, double centerOfRotationX, double centerOfRotationY, boolean isFieldCentric) {
        this.translationDirection = translationDirection;
        this.translationSpeed = translationSpeed;
        this.rotationVelocity = rotationVelocity;
        this.centerOfRotationX = centerOfRotationX;
        this.centerOfRotationY = centerOfRotationY;
        this.isFieldCentric = isFieldCentric;
    }

    /**
     * Constructs this velocity state assuming a center of rotation of <0, 0>.
     * 
     * @param translationDirection The direction to translate, in degrees
     *                             counting counterclockise from forwards.
     * @param translationSpeed The speed to translate, in feet per second.
     * @param rotationVelocity The angular velocity to rotate, in degrees
     *                         per second.
     * @param isFieldCentric True if this state is field-centric, false if it
     *                       is robot-centric.
     */
    public VelocityState(double translationDirection, double translationSpeed, double rotationVelocity, boolean isFieldCentric) {
        this(translationDirection, translationSpeed, rotationVelocity, 0, 0, isFieldCentric);
    }
    
    /**
     * Gets the direction of translation.
     * 
     * @return The translation direction, in degrees counterclockwise from
     *         forwards.
     */
    public double getTranslationDirection() {
        return this.translationDirection;
    }

    /**
     * Gets the speed of translation.
     * 
     * @return The translation speed, in feet per second.
     */
    public double getTranslationSpeed() {
        return this.translationSpeed;
    }
    
    /**
     * Gets a vector representing translation.
     * 
     * @return The translation velocity vector, in feet per second.
     */
    public Vector2D getTranslationVector() {
        return Vector2D.createPolarCoordinates(this.getTranslationSpeed(), new WrappedAngle(this.getTranslationDirection()));
    }

    /**
     * Gets the rotational velocity.
     * 
     * @return The angular velocity, in degrees per second.
     */
    public double getRotationVelocity() {
        return this.rotationVelocity;
    }

    /**
     * Gets the x component of the center of rotation.
     * 
     * @return The x component of the center of rotation, in feet.
     */
    public double getCenterOfRotationX() {
        return this.centerOfRotationX;
    }

    /**
     * Gets the y component of the center of rotation.
     * 
     * @return The y component of the center of rotation, in feet.
     */
    public double getCenterOfRotationY() {
        return this.centerOfRotationY;
    }

    /**
     * Gets the center of rotation as a vector.
     * 
     * @return The center of rotation, in feet.
     */
    public Vector2D getCenterOfRotationVector() {
        return Vector2D.createCartesianCoordinates(this.centerOfRotationX, this.centerOfRotationY);
    }

    /**
     * Gets if this velocity state is field-centric.
     * 
     * @return True if this velocity state is field-centric, false otherwise.
     */
    public boolean isFieldCentric() {
        return this.isFieldCentric;
    }

    /**
     * Creates a new velocity state that is the same as this one, except with
     * the given translation direction.
     * 
     * @param direction The translation direction, in degrees counterclockwise
     *                  from forwards.
     * @return The new velocity state.
     */
    public VelocityState changeTranslationDirection(double direction) {
        return new VelocityState(direction, this.translationSpeed, this.rotationVelocity, this.centerOfRotationX, this.centerOfRotationY, this.isFieldCentric);
    }

    /**
     * Creates a new velocity state that is the same as this one, except with
     * the given translation speed.
     * 
     * @param speed The translation speed, in feet per second.
     * @return The new velocity state.
     */
    public VelocityState changeTranslationSpeed(double speed) {
        return new VelocityState(this.translationDirection, speed, this.rotationVelocity, this.centerOfRotationX, this.centerOfRotationY, this.isFieldCentric);
    }

    /**
     * Creates a new velocity state that is the same as this one, except with
     * the given rotation velocity.
     * 
     * @param velocity The rotation velocity, in degrees per second.
     * @return The new velocity state.
     */
    public VelocityState changeRotationVelocity(double velocity) {
        return new VelocityState(this.translationDirection, this.translationSpeed, velocity, this.centerOfRotationX, this.centerOfRotationY, this.isFieldCentric);
    }

    /**
     * Creates a new velocity state that is the same as this one, except with
     * the given center of rotation.
     * 
     * @param x The x component of the center of rotation, in feet.
     * @param y The y component of the center of rotation, in feet.
     * @return The new velocity state.
     */
    public VelocityState changeCenterOfRotation(double x, double y) {
        return new VelocityState(this.translationDirection, this.translationSpeed, this.rotationVelocity, x, y, this.isFieldCentric);
    }

    /**
     * Creates a new velocity state that is the same as this one, except with
     * the given field-centric setting.
     * 
     * @param isFieldCentric True if the velocity state should be
     *                       field-centric, false if it should be
     *                       robot-centric.
     * @return The new velocity state.
     */
    public VelocityState changeIsFieldCentric(boolean isFieldCentric) {
        return new VelocityState(this.translationDirection, this.translationSpeed, this.rotationVelocity, this.centerOfRotationX, this.centerOfRotationY, isFieldCentric);
    }
}
