package frc.team88.swerve.swervemodule;

/**
 * Interface for setting and getting the physical attributes of a single swerve module.
 */
public interface SwerveModule {

    /**
     * Sets the commanded speed and azimuith of the swerve module.
     * 
     * @param wheelSpeed The wheel speed commanded, in feet/second
     * @param azimuth The azimuth commanded, in degrees (0 being forwards)
     */
    public void set(double wheelSpeed, double azimuth);

    /**
     * Gets the speed of the wheel.
     * 
     * @return The speed in feet/second
     */
    public double getWheelSpeed();

    /**
     * Gets the azimuth of the module.
     * @return The azimuth of the module in degrees
     */
    public double getAzimuth();

    /**
     * Gets the angular speed of the module's azimuth.
     * 
     * @return the angular speed of the module in degrees per second
     */
    public double getAzimuthVelocity();

}