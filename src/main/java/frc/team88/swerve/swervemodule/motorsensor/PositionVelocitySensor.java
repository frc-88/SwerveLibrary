package frc.team88.swerve.swervemodule.motorsensor;

/**
 * Represents a sensor which can provide both position and velocity in one
 * dimension, such as an encoder or potentiometer. Can also be set to make the
 * current position correspond to a given value.
 */
public interface PositionVelocitySensor {

    /**
     * Get the position value of the sensor.
     * 
     * @return The position value
     */
    public double getPosition();

    /**
     * Get the velocity value of the sensor.
     * 
     * @return The velocity value
     */
    public double getVelocity();

    /**
     * Calibrate the sensor such that the current position will read as the given
     * value.
     * 
     * @param position The position to set
     */
    public void calibratePosition(double position);

}