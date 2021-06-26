package frc.team88.swerve.configuration.subconfig;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Captures all of the configuration information about a SwerveModule's azimuth controller. */
public class ConstraintsConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private final OptimizerConfiguration optimizerConfig;

  /**
   * Constructs from a raw configuration.
   *
   * @param config The raw configuration.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public ConstraintsConfiguration(Config config) {
    Objects.requireNonNull(config);
    this.optimizerConfig =
        new OptimizerConfiguration(
            Configuration.configCheckAndGet(config, "optimizer", Config.class));
  }

  /**
   * Gets the optimizer configuration.
   *
   * @return The optimizer configuration.
   */
  public OptimizerConfiguration getOptimizerConfig() {
    return this.optimizerConfig;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    this.optimizerConfig.populateNetworkTable(table);
  }

  public static class OptimizerConfiguration implements NetworkTablePopulator {

    // Configuration values. See getters for documentation.
    private double precisionFeet;
    private double precisionDegrees;

    private transient boolean firstNetworkTableCall = true;

    /**
     * Constructs from a raw configuration.
     *
     * @param config The raw configuration.
     * @throws SwerveConfigException If the user provided config is incorrect.
     */
    public OptimizerConfiguration(Config config) {
      Objects.requireNonNull(config);
      this.precisionFeet = Configuration.configCheckAndGetDouble(config, "feet-precision");
      if (this.precisionFeet <= 0) {
        throw new InvalidConfigValueException(
            String.format("Feet precision is %d, but it should be positive.", this.precisionFeet));
      }
      this.precisionDegrees = Configuration.configCheckAndGetDouble(config, "degrees-precision");
      if (this.precisionDegrees <= 0) {
        throw new InvalidConfigValueException(
            String.format(
                "Degrees precision is %d, but it should be positive.", this.precisionDegrees));
      }
    }

    /**
     * Gets the feet precision.
     *
     * @return The precision for optimizing measurements in feet.
     */
    public double getPrecisionFeet() {
      return this.precisionFeet;
    }

    /**
     * Gets the degrees precision.
     *
     * @return The precision for optimizing measurements in degrees.
     */
    public double getPrecisionDegrees() {
      return this.precisionDegrees;
    }

    @Override
    public void populateNetworkTable(NetworkTable table) {
      if (this.firstNetworkTableCall) {
        this.firstNetworkTableCall = false;
        table.getEntry("precisionFeet").setDouble(this.precisionFeet);
        table.getEntry("precisionDegrees").setDouble(this.precisionDegrees);
      } else {
        this.precisionFeet = table.getEntry("precisionFeet").getDouble(this.precisionFeet);
        this.precisionDegrees = table.getEntry("precisionDegrees").getDouble(this.precisionDegrees);
      }
    }
  }
}
