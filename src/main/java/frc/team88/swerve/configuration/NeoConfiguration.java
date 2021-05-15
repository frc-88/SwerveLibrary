package frc.team88.swerve.configuration;

import java.util.Objects;

import com.electronwill.nightconfig.core.Config;

/**
 * Captures all of the configuration information about a Neo.
 */
public class NeoConfiguration {

    // Configuration values. See getters for documentation.
    private final boolean inverted;
    private final double maxSpeed;

    /**
     * Constructs this configuration from a NEO config.
     * 
     * @param config The instantiated NEO template.
     */
    public NeoConfiguration(Config config) {
        Objects.requireNonNull(config);
        this.inverted = config.get("inverted");
        this.maxSpeed = (double) config.get("max-speed-rps");
    }
    
    /**
     * Gets if the motor is inverted.
     * 
     * @return If the motor is inverted.
     */
    public boolean isInverted() {
        return this.inverted;
    }

    /**
     * Gets if the motor is inverted.
     * 
     * @return If the motor is inverted.
     */
    public boolean getInverted() {
        return this.inverted;
    }

    /**
     * Gets the max speed of the motor.
     * 
     * @return The measured max speed of the motor, including inefficiencies.
     */
    public double getMaxSpeed() {
        return this.maxSpeed;
    }
}
