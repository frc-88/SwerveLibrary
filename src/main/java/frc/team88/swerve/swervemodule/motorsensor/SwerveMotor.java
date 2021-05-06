package frc.team88.swerve.swervemodule.motorsensor;

/**
 * Motor with associated sensor that can be set to a velocity.
 */
public interface SwerveMotor extends PositionSensor {

    /**
     * Get the velocity from the sensor, in rotations per second.
     * 
     * @return The velocity from the sensor, in rotations per second
     */
    public double getVelocity();

    /**
     * Set the velocity to the given value.
     * 
     * @param velocity
     *                     The velocity to set, in rotations per second.
     */
    public void setVelocity(double velocity);

    /**
     * Gets the max velocity of this motor.
     * 
     * @return The max velocity, in rotations per second.
     */
    public double getMaxVelocity();

}
