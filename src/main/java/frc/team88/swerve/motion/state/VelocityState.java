package frc.team88.swerve.motion.state;

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

    public VelocityState(double translationDirection, double translationSpeed, double rotationVelocity, boolean isFieldCentric) {
        this(translationDirection, translationSpeed, rotationVelocity, 0, 0, isFieldCentric);
    }
    

    public double getTranslationDirection() {
        return this.translationDirection;
    }


    public double getTranslationSpeed() {
        return this.translationSpeed;
    }


    public double getRotationVelocity() {
        return this.rotationVelocity;
    }


    public double getCenterOfRotationX() {
        return this.centerOfRotationX;
    }


    public double getCenterOfRotationY() {
        return this.centerOfRotationY;
    }


    public boolean isIsFieldCentric() {
        return this.isFieldCentric;
    }

    public boolean getIsFieldCentric() {
        return this.isFieldCentric;
    }


}
