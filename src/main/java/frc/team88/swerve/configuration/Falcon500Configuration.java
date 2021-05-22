package frc.team88.swerve.configuration;

import java.util.Objects;

import com.electronwill.nightconfig.core.Config;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;

/**
 * Captures all of the configuration information about a Falcon 500.
 */
public class Falcon500Configuration implements NetworkTablePopulator {

    // Configuration values. See getters for documentation.
    private boolean inverted;
    private double maxSpeed;

    private transient boolean firstNetworkTableCall = true;

    /**
     * Constructs this configuration from a falcon 500 config.
     * 
     * @param config The instantiated falcon 500 template.
     */
    public Falcon500Configuration(Config config) {
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

    @Override
    public void populateNetworkTable(NetworkTable table) {
        if (this.firstNetworkTableCall) {
            this.firstNetworkTableCall = false;
            table.getEntry("inverted").setBoolean(this.inverted);
            table.getEntry("maxSpeed").setDouble(this.maxSpeed);
        } else {
            this.inverted = table.getEntry("inverted").getBoolean(this.inverted);
            this.maxSpeed = table.getEntry("maxSpeed").getDouble(this.maxSpeed);
        }
    }
}
