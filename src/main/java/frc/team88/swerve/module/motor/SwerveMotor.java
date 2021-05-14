package frc.team88.swerve.module.motor;

import frc.team88.swerve.module.sensor.PositionSensor;

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

    /**
     * Get the current draw from this motor.
     * 
     * @return The current draw, in amps.
     */
    public double getCurrentDraw();

    /**
     * Get the voltage commanded to this motor.
     * 
     * @return The command voltage, in volts.
     */
    public double getCommandVoltage();

    /**
     * The velocity commanded to this motor.
     * 
     * @return The command velocity, in rotations per second.
     */
    public double getCommandVelocity();

    /**
     * Sets the motor to coast mode.
     */
    public void setCoast();

    /**
     * Sets the motor to brake mode.
     */
    public void setBrake();

}
