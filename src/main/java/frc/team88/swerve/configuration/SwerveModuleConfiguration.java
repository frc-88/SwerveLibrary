package frc.team88.swerve.configuration;

import java.util.Objects;

import com.electronwill.nightconfig.core.Config;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import frc.team88.swerve.util.Vector2D;

/**
 * Captures all of the configuration information about a SwerveModule.
 */
public class SwerveModuleConfiguration {

    // Configuration values. See getters for documentation.
    private final Vector2D location;
    private final RealMatrix forwardMatrix;
    private final RealMatrix inverseMatrix;
    private final double wheelDiameter;
    private final AzimuthControllerConfiguration azimuthControllerConfig;

    /**
     * Constructs this configuration from an instantiated module design.
     * 
     * @param instantiatedConfig The instantiated module design.
     */
    public SwerveModuleConfiguration(Config instantiatedConfig) {
        Objects.requireNonNull(instantiatedConfig);
        this.location = Vector2D.createCartesianCoordinates((double) instantiatedConfig.get("location-inches.x"),
                (double) instantiatedConfig.get("location-inches.x"));
        this.forwardMatrix = new Array2DRowRealMatrix(instantiatedConfig.get("differential-matrix"), true);
        this.inverseMatrix = MatrixUtils.inverse(this.forwardMatrix);
        this.wheelDiameter = instantiatedConfig.get("wheel-diameter-inches");
        this.azimuthControllerConfig = new AzimuthControllerConfiguration(instantiatedConfig.get("azimuth-controller"));
    }

    /**
     * Gets the location of the swerve module.
     * 
     * @return The location of the swerve module in inches.
     */
    public Vector2D getLocation() {
        return this.location;
    }

    /**
     * Gets the forward differential matrix.
     * 
     * @return The 2x2 forward differential matrix for the module, which converts
     *         from inputs [motor0, motor1] to outputs [azimuthSpeed, wheelSpeed].
     *         Unitless.
     */
    public RealMatrix getForwardMatrix() {
        return this.forwardMatrix;
    }

    /**
     * Gets the inverse differential matrix.
     * 
     * @return The 2x2 inverse differential matrix for the module, which converts
     *         from outputs [azimuthSpeed, wheelSpeed] to inputs [motor0, motor1].
     *         Unitless.
     */
    public RealMatrix getInverseMatrix() {
        return this.inverseMatrix;
    }

    /**
     * Gets the wheel size.
     * 
     * @return The diameter of the wheel on the module, in inches.
     */
    public double getWheelDiameter() {
        return this.wheelDiameter;
    }

    /**
     * Gets the azimuth controller configuration.
     * 
     * @return The configuration info for the azimuth controller.
     */
    public AzimuthControllerConfiguration getAzimuthControllerConfig() {
        return this.azimuthControllerConfig;
    }

    /**
     * Captures all of the configuration information about a SwerveModule's azimuth
     * controller.
     */
    public class AzimuthControllerConfiguration {

        // Configuration values. See getters for documentation.
        private final PIDConfiguration pidConfig;
        private final double maxSpeed;
        private final double maxAcceleration;

        /**
         * Constructs from a raw configuration.
         * 
         * @param config The raw configuration.
         */
        public AzimuthControllerConfiguration(Config config) {
            Objects.requireNonNull(config);
            this.pidConfig = new PIDConfiguration(config);
            this.maxSpeed = config.get("max-speed");
            this.maxAcceleration = config.get("max-acceleration");
        }

        /**
         * Gets the PID configuration.
         * 
         * @return The PID configuration for the azimuth controller.
         */
        public PIDConfiguration getPIDConfig() {
            return this.pidConfig;
        }

        /**
         * Gets the max speed.
         * 
         * @return The max speed for the trapezoidal controller.
         */
        public double getMaxSpeed() {
            return this.maxSpeed;
        }

        /**
         * Gets the max acceleration.
         * 
         * @return The max acceleration for the trapezoidal controller.
         */
        public double getMaxAcceleration() {
            return this.maxAcceleration;
        }
    }
}
