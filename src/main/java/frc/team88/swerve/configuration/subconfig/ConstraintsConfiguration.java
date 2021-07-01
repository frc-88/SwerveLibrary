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
    private double translationDirectionPrecision;
    private double translationSpeedPrecision;
    private double rotationVelocityPrecision;

    private transient boolean firstNetworkTableCall = true;

    /**
     * Constructs from a raw configuration.
     *
     * @param config The raw configuration.
     * @throws SwerveConfigException If the user provided config is incorrect.
     */
    public OptimizerConfiguration(Config config) {
      Objects.requireNonNull(config);

      this.translationDirectionPrecision = Configuration.configCheckAndGetDouble(config, "translation-direction-precision");
      if (this.translationDirectionPrecision <= 0) {
        throw new InvalidConfigValueException(
            String.format("Translation direction precision is %d, but it should be positive.", this.translationDirectionPrecision));
      }

      this.translationSpeedPrecision = Configuration.configCheckAndGetDouble(config, "translation-speed-precision");
      if (this.translationSpeedPrecision <= 0) {
        throw new InvalidConfigValueException(
            String.format("Translation speed precision is %d, but it should be positive.", this.translationSpeedPrecision));
      }

      this.rotationVelocityPrecision = Configuration.configCheckAndGetDouble(config, "rotation-velocity-precision");
      if (this.rotationVelocityPrecision <= 0) {
        throw new InvalidConfigValueException(
            String.format("Rotation velocity precision is %d, but it should be positive.", this.rotationVelocityPrecision));
      }
    }

    /**
     * Gets the translation direction precision.
     *
     * @return The precision for optimizing the translation direction.
     */
    public double getTranslationDirectionPrecision() {
      return this.translationDirectionPrecision;
    }

    /**
     * Gets the translation speed precision.
     *
     * @return The precision for optimizing the translation speed.
     */
    public double getTranslationSpeedPrecision() {
      return this.translationSpeedPrecision;
    }

    /**
     * Gets the rotation velocity precision.
     *
     * @return The precision for optimizing the rotation velocity.
     */
    public double getRotationVelocityPrecision() {
      return this.rotationVelocityPrecision;
    }


    @Override
    public void populateNetworkTable(NetworkTable table) {
      if (this.firstNetworkTableCall) {
        this.firstNetworkTableCall = false;
        table.getEntry("translationDirectionPrecision").setDouble(this.translationDirectionPrecision);
        table.getEntry("translationSpeedPrecision").setDouble(this.translationSpeedPrecision);
        table.getEntry("rotationVelocityPrecision").setDouble(this.rotationVelocityPrecision);
      } else {
        this.translationDirectionPrecision = table.getEntry("translationDirectionPrecision").getDouble(this.translationDirectionPrecision);
        this.translationSpeedPrecision = table.getEntry("translationSpeedPrecision").getDouble(this.translationSpeedPrecision);
        this.rotationVelocityPrecision = table.getEntry("rotationVelocityPrecision").getDouble(this.rotationVelocityPrecision);
      }
    }
  }
}
