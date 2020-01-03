package frc.team88.swerve.swervemodule.motorsensor;

import java.util.Objects;

/**
 * Composed of a single sensor, and applies a gear ratio to it.
 */
public class SensorTransmission implements PositionVelocitySensor {

    // The sensor on the input of the transmission
    private PositionVelocitySensor inputSensor;

    // The gear ratio being applied to the sensor
    private double gearRatio;

    /**
     * Constructor.
     * 
     * @param inputSensor The sensor on the input of the transmission
     * @param gearRatio   The gear ratio being applied to the sensor
     */
    public SensorTransmission(PositionVelocitySensor inputSensor, double gearRatio) {
        this.inputSensor = Objects.requireNonNull(inputSensor);
        if (gearRatio != 0.) {
            this.gearRatio = gearRatio;
        } else {
            throw new IllegalArgumentException("Gear ratio must not be zero");
        }
    }

    @Override
    public double getPosition() {
        return applyForwardGearRatio(inputSensor.getPosition());
    }

    @Override
    public double getVelocity() {
        return applyForwardGearRatio(inputSensor.getVelocity());
    }

    @Override
    public void calibratePosition(double position) {
        inputSensor.calibratePosition(applyReverseGearRatio(position));
    }

    /**
     * Applies the gear ratio to get from an input value to an output value.
     * 
     * @param value The input value
     * @return The output value
     */
    protected double applyForwardGearRatio(double value) {
        return value / gearRatio;
    }

    /**
     * Applies the gear ratio to get from an output value to an input value.
     * 
     * @param value The output value
     * @return The input value
     */
    protected double applyReverseGearRatio(double value) {
        return value * gearRatio;
    }

}