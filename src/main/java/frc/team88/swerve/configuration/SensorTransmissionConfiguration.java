package frc.team88.swerve.configuration;

import java.util.Objects;

import com.electronwill.nightconfig.core.Config;

/**
 * Captures all of the configuration information about a position sensor.
 */
public class SensorTransmissionConfiguration {

    // Configuration values. See getters for documentation.
    private final boolean inverted;
    private final double ratio;
    private final double offset;

    /**
     * Constructs this configuration from a position sensor config.
     * 
     * @param config The instantiated position sensor template.
     */
    public SensorTransmissionConfiguration(Config config) {
        Objects.requireNonNull(config);
        this.inverted = config.get("inverted");
        this.ratio = config.get("ratio");
        this.offset = config.get("offset");
    }

    /**
     * Gets if the sensor is inverted.
     * 
     * @return If the sensor is inverted.
     */
    public boolean isInverted() {
        return this.inverted;
    }

    /**
     * Gets if the sensor is inverted.
     * 
     * @return If the sensor is inverted.
     */
    public boolean getInverted() {
        return this.inverted;
    }

    /**
     * Gets the sensor's gear ratio.
     * 
     * @return The sensors gear ratio, which multiplied by the output value
     * gives the input value.
     */
    public double getRatio() {
        return this.ratio;
    }

    /**
     * Gets the sensor's offset.
     * 
     * @return The offset of the sensor, which is subtracted from the position
     * after applying the gear ratio and inversion.
     */
    public double getOffset() {
        return this.offset;
    }
}
