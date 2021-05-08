package frc.team88.swerve.module.sensor;

/**
 * Represents a sensor which can provide position in one dimension, such as an
 * encoder or potentiometer. Can also be set to make the current position 
 * correspond to a given value.
 */
public interface PositionSensor {

    /**
     * Get the position value of the sensor.
     * 
     * @return The position value in rotations.
     */
    public double getPosition();
}
