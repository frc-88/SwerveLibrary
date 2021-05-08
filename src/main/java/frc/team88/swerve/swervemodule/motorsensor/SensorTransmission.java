package frc.team88.swerve.swervemodule.motorsensor;

import java.util.Objects;

import frc.team88.swerve.configuration.SensorTransmissionConfiguration;

/**
 * Composed of a single sensor, and applies a gear ratio to it.
 */
public class SensorTransmission implements PositionSensor {

    // The sensor on the input of the transmission
    private PositionSensor inputSensor;

    // The configuration data for this sensor.
    private final SensorTransmissionConfiguration config;

    /**
     * Constructor.
     * 
     * @param inputSensor The sensor on the input of the transmission.
     * @param config The config data for this sensor transmission.
     */
    public SensorTransmission(PositionSensor inputSensor, SensorTransmissionConfiguration config) {
        this.inputSensor = Objects.requireNonNull(inputSensor);
        this.config = Objects.requireNonNull(config);
    }

    @Override
    public double getPosition() {
        return applyTransmissionForwards(inputSensor.getPosition()) - this.config.getOffset();
    }

    /**
     * Applies the gear ratio, inversion, and offset to get from an input value
     * to an output value.
     * 
     * @param value The input value.
     * @return The output value.
     */
    protected double applyTransmissionForwards(double value) {
        return value / this.config.getRatio() * this.getInversionConstant() - this.config.getOffset();
    }

    /**
     * Applies the gear ratio, inversion, and offset to get from an output
     * value to an input value.
     * 
     * @param value The output value.
     * @return The input value.
     */
    protected double applyTransmissionReverse(double value) {
        return (value + this.config.getOffset()) * this.config.getRatio() * this.getInversionConstant();
    }

    /**
     * Returns -1. if the sensor is inverted, or 1. otherwise.
     * 
     * @return The constant to multiple positions by for inversion.
     */
    protected double getInversionConstant() {
        return this.config.isInverted() ? -1. : 1.;
    }

}