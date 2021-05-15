package frc.team88.swerve.configuration;

import java.util.Objects;

import com.electronwill.nightconfig.core.Config;

/**
 * Contains PID values pulled from a config file.
 */
public class PIDConfiguration {

    // Configuration values. See getters for documentation.
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;
    private final double iZone;
    private final double iMax;
    private final double tolerance;
    
    /**
     * Constructs from a raw configuration containing some of the appropriate
     * fields.
     * 
     * @param config The raw configuration.
     */
    public PIDConfiguration(Config config) {
        Objects.requireNonNull(config);
        this.kP = config.getOrElse("kP", 0.);
        this.kI = config.getOrElse("kI", 0.);
        this.kD = config.getOrElse("kD", 0.);
        this.kF = config.getOrElse("kF", 0.);
        this.iZone = config.getOrElse("i-zone", 0.);
        this.iMax = config.getOrElse("i-max", 0.);
        this.tolerance = config.getOrElse("tolerance", 0.);
    }

    /**
     * Gets the kP value.
     * 
     * @return The proportional constant.
     */
    public double getKP() {
        return this.kP;
    }

    /**
     * Gets the kI value.
     * 
     * @return The integral constant.
     */
    public double getKI() {
        return this.kI;
    }

    /**
     * Gets the kD value.
     * 
     * @return The differential constant.
     */
    public double getKD() {
        return this.kD;
    }

    /**
     * Gets the kF value.
     * 
     * @return The feedforward constant.
     */
    public double getKF() {
        return this.kF;
    }

    /**
     * Gets the iZone value.
     * 
     * @return The max error for which integral will be accumulated.
     */
    public double getIZone() {
        return this.iZone;
    }

    /**
     * Gets the iMax value.
     * 
     * @return The max accumulated integral value.
     */
    public double getIMax() {
        return this.iMax;
    }

    /**
     * Gets the tolerance value.
     * 
     * @return The minimum error for the PID output to be non-zero.
     */
    public double getTolerance() {
        return this.tolerance;
    }

    
}
