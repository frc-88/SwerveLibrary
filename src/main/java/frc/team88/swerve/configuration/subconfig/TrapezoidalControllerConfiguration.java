package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Captures all of the configuration information about a SwerveModule's azimuth controller. */
public class TrapezoidalControllerConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private final PIDConfiguration pidConfig;
  private double maxSpeed;
  private double maxAcceleration;

  private transient boolean firstNetworkTableCall = true;

  /**
   * Constructs from a raw configuration.
   *
   * @param config The raw configuration.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public TrapezoidalControllerConfiguration(Config config) {
    Objects.requireNonNull(config);
    this.pidConfig = new PIDConfiguration(config);
    this.maxSpeed = Configuration.configCheckAndGetDouble(config, "max-speed");
    if (this.maxSpeed <= 0) {
      throw new InvalidConfigValueException(
          String.format("Max speed is %d, but it should be positive.", this.maxSpeed));
    }
    this.maxAcceleration = Configuration.configCheckAndGetDouble(config, "max-acceleration");
    if (this.maxAcceleration <= 0) {
      throw new InvalidConfigValueException(
          String.format(
              "Max acceleration is %d, but it should be positive.", this.maxAcceleration));
    }
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

  @Override
  public void populateNetworkTable(NetworkTable table) {
    this.pidConfig.populateNetworkTable(table);
    if (this.firstNetworkTableCall) {
      this.firstNetworkTableCall = false;
      table.getEntry("maxSpeed").setDouble(this.maxSpeed);
      table.getEntry("maxAcceleration").setDouble(this.maxAcceleration);
    } else {
      this.maxSpeed = table.getEntry("maxSpeed").getDouble(this.maxSpeed);
      this.maxAcceleration = table.getEntry("maxAcceleration").getDouble(this.maxAcceleration);
    }
  }
}
