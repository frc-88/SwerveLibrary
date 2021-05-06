package frc.team88.swerve.configuration;

import java.util.List;
import java.util.Objects;

import com.electronwill.nightconfig.core.Config;
import com.electronwill.nightconfig.core.Config.Entry;
import com.electronwill.nightconfig.core.file.FileNotFoundAction;
import com.electronwill.nightconfig.core.io.ConfigParser;
import com.electronwill.nightconfig.core.io.ParsingMode;
import com.electronwill.nightconfig.toml.TomlFormat;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.wrappers.gyro.Gyro;
import frc.team88.swerve.wrappers.gyro.NavX;

/**
 * 
 */

/**
 * Parses a swerve configuration file, generates objects from it, and provides
 * access to all of its contents.
 */
public class Configuration {

    // The loaded config data
    private final Config configData;

    // The gyro from this configuration
    private Gyro gyro;

    // The swerve modules from this configuration
    private SwerveModule[] modules;
    
    /**
     * Loads the base config and user config from the filesystem.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     */
    public Configuration(final String configPath) {
        Objects.requireNonNull(configPath);

        ConfigParser<?> tomlParser = TomlFormat.instance().createParser();

        // Parse the base config file first
        this.configData = tomlParser.parse(getClass().getResourceAsStream("data/base_config.toml"));

        // Append and overwrite with the user-supplied config file
        tomlParser.parse(Filesystem.getDeployDirectory().toPath().resolve(configPath), this.configData, ParsingMode.MERGE, FileNotFoundAction.THROW_ERROR);

        // Create all of the objects and configs
        this.instantiateGyro();
        this.instantiateModules();
    }

    /**
     * Gets the gyro object specified by this config.
     * 
     * @return The gyro object.
     */
    public Gyro getGyro() {
        return this.gyro;
    }

    /**
     * Instantiates a template's config by apppending and overwritting the design
     * with an instance-specific config.
     */
    private Config instantiateTemplateConfig(Config templateConfig, Config instanceConfig) {
        instanceConfig = Config.copy(instanceConfig, TomlFormat.instance());
        Config instantiatedConfig = Config.copy(templateConfig, TomlFormat.instance());

        for (Entry entry : instanceConfig.entrySet()) {
            instantiatedConfig.set(entry.getKey(), entry.getValue());
        }

        return instantiatedConfig;
    }

    /**
     * Instatiates a NavX object.
     * 
     * @param gyroConfig The config for the gyro
     * @return The navx object
     */
    private NavX instantiateNavX(Config gyroConfig) {
        String portType = gyroConfig.get("port-type");
        switch(portType) {
            case "SPI":
                return new NavX(gyroConfig.getEnum("port", SPI.Port.class));
            case "I2C":
                return new NavX(gyroConfig.getEnum("port", I2C.Port.class));
            case "Serial":
                return new NavX(gyroConfig.getEnum("port", SerialPort.Port.class));
            default:
                throw new IllegalArgumentException(String.format("Invalid port type %s given in configuration.", portType));
        }
    }

    /**
     * Instantiates the gyro object from the config.
     */
    private void instantiateGyro() {
        String template = this.configData.get("gyro.template");
        Config gyroConfig = instantiateTemplateConfig(configData.get("gyro-templates." + template), configData.get("gyro"));

        switch (template) {
            case "navx":
                this.gyro = this.instantiateNavX(gyroConfig);
                break;
            default:
                throw new IllegalArgumentException(String.format("The template %s does not have a corresponding gyro class.", template));
        }
    }

    /**
     * Instantiates an individual module.
     * 
     * @param instanceConfig The instance config for this module.
     * @return The module object.
     */
    private SwerveModule instantiateModule(Config instanceConfig) {
        String template = instanceConfig.get("template");
        Config moduleConfig = instantiateTemplateConfig(configData.get("module-templates." + template), instanceConfig);

        
    }

    /**
     * Instantiates the module objects from the config.
     */
    private void instantiateModules() {
        List<Config> moduleConfigs = this.configData.get("modules");
        this.modules = new SwerveModule[moduleConfigs.size()];

        for (int moduleIndex = 0; moduleIndex < moduleConfigs.size(); moduleIndex++) {
            this.modules[moduleIndex] = this.instantiateModule(moduleConfigs.get(moduleIndex));
        }
    }
}
