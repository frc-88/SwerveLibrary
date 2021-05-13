package frc.team88.swerve.configuration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
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
import frc.team88.swerve.gyro.NavX;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.module.motor.Falcon500;
import frc.team88.swerve.module.motor.Neo;
import frc.team88.swerve.module.motor.SwerveMotor;
import frc.team88.swerve.module.sensor.PositionSensor;
import frc.team88.swerve.module.sensor.CANifiedPWMEncoder;
import frc.team88.swerve.module.sensor.SensorTransmission;


/**
 * Parses a swerve configuration file, generates objects from it, and provides
 * access to all of its contents.
 */
public class Configuration {

    // The loaded config data
    private final Config configData;

    // The gyro from this configuration
    private SwerveGyro gyro;

    // The swerve modules from this configuration
    private SwerveModule[] modules;

    // The canifiers used by sensors in this configuration
    private final Map<Integer, CANifier> canifiers;
    
    /**
     * Loads the base config and user config from the filesystem.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     * @param gyro       The gyro to use. Will instantiate from config if null.
     */
    public Configuration(final String configPath, SwerveGyro gyro) {
        Objects.requireNonNull(configPath);
        this.canifiers = new HashMap<>();
        ConfigParser<?> tomlParser = TomlFormat.instance().createParser();
        
        // Parse the base config file first
        this.configData = tomlParser.parse(getClass().getResourceAsStream("/base_config.toml"));

        // Append and overwrite with the user-supplied config file
        tomlParser.parse(Filesystem.getDeployDirectory().toPath().resolve(configPath), this.configData, ParsingMode.MERGE, FileNotFoundAction.THROW_ERROR);

        // Create all of the objects and configs
        this.instantiateModules();
        if (Objects.nonNull(gyro)) {
            this.gyro = gyro;
        } else {
            this.instantiateGyro();
        }
    }

    /**
     * Loads the base config and user config from the filesystem. Instantiates
     * the gyro from the config.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     */
    public Configuration(final String configPath) {
        this(configPath, null);
    }

    /**
     * Gets the gyro object specified by this config.
     * 
     * @return The gyro object.
     */
    public SwerveGyro getGyro() {
        return this.gyro;
    }

    /**
     * Gets the swerve modules specified by this config.
     * 
     * @return The modules array.
     */
    public SwerveModule[] getModules() {
        return this.modules;
    }

    /**
     * Gets a mapping from can IDs to canifiers instantiated by this
     * configuration.
     * 
     * @return The mapping from can IDs to CANifiers.
     */
    public Map<Integer, CANifier> getCanifiers() {
        return this.canifiers;
    }

    /**
     * Instantiates a template's config by apppending and overwritting the design
     * with an instance-specific config.
     */
    private Config instantiateTemplateConfig(Config templateConfig, Config instanceConfig) {
        instanceConfig = Config.copy(instanceConfig, TomlFormat.instance());
        Config instantiatedConfig = Config.copy(templateConfig, TomlFormat.instance());

        for (Entry entry : instanceConfig.entrySet()) {
            Object value = entry.getValue();
            if (value instanceof List<?>) {
                List<Object> newList = new ArrayList<Object>();
                for (Object subvalue : (List<?>)value) {
                    if (subvalue instanceof Config) {
                        newList.add(this.instantiateTemplateConfig(templateConfig.get(entry.getKey()), (Config)value));
                    } else {
                        newList.add(subvalue);
                    }
                }
                value = newList;
            } else if (value instanceof Config) {
                value = this.instantiateTemplateConfig(templateConfig.get(entry.getKey()), (Config)value);
            }
            instantiatedConfig.set(entry.getKey(), value);
        }

        return instantiatedConfig;
    }

    /**
     * Instatiates a NavX object.
     * 
     * @param gyroConfig The config for the gyro.
     * @return The navx object.
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
        Config gyroConfig = this.instantiateTemplateConfig(configData.get("gyro-templates." + template), configData.get("gyro"));

        switch (template) {
            case "navx":
                this.gyro = this.instantiateNavX(gyroConfig);
                break;
            default:
                throw new IllegalArgumentException(String.format("The template %s does not have a corresponding gyro class.", template));
        }
    }

    /**
     * Instantiates a PWM position sensor on a Canifier. Will create the
     * Canifier object if it does not already exist.
     * 
     * @param instanceConfig The config for the canified pwm sensor.
     * @return The canified pwm sensor object.
     */
    private SensorTransmission instantiateCanifiedPWM(Config instanceConfig) {
        int canID = instanceConfig.getInt("can-id");
        if (!this.canifiers.containsKey(canID)) {
            this.canifiers.put(canID, new CANifier(canID));
        }

        int pwmChannel = instanceConfig.getInt("pwm-channel");
        PWMChannel channel;
        switch (pwmChannel) {
            case 0:
                channel = PWMChannel.PWMChannel0;
                break;
            case 1:
                channel = PWMChannel.PWMChannel1;
                break;
            case 2:
                channel = PWMChannel.PWMChannel2;
                break;
            case 3:
                channel = PWMChannel.PWMChannel3;
                break;
            default:
                throw new IllegalArgumentException(String.format("%d is not a valid PWM channel. It must be between 0 and 3, inclusive", pwmChannel));
        }

        CANifiedPWMEncoder rawSensor = new CANifiedPWMEncoder(this.canifiers.get(canID), channel);
        return new SensorTransmission(rawSensor, new SensorTransmissionConfiguration(instanceConfig));
    }

    /**
     * Instantiates a position sensor.
     * 
     * @param instanceConfig The config for the sensor.
     * @return The sensor object.
     */
    private SensorTransmission instantiateSensor(Config instanceConfig) {
        String template = instanceConfig.get("template");
        Config sensorConfig = this.instantiateTemplateConfig(configData.get("sensor-templates." + template), instanceConfig);

        switch (template) {
            case "canified-pwm":
                return this.instantiateCanifiedPWM(sensorConfig);
            default:
                throw new IllegalArgumentException(String.format("The template %s does not have a corresponding sensor class.", template));
        }
    }

    /**
     * Instantiates a Falcon 500.
     * 
     * @param instanceConfig The config for the Falcon 500.
     * @return The falcon object.
     */
    private Falcon500 instantiateFalcon500(Config instanceConfig) {
        return new Falcon500(instanceConfig.getInt("can-id"), new Falcon500Configuration(instanceConfig));
    }

    /**
     * Instantiates a Neo.
     * 
     * @param instanceConfig The config for the Neo.
     * @return The neo object.
     */
    private Neo instantiateNeo(Config instanceConfig) {
        return new Neo(instanceConfig.getInt("can-id"), new NeoConfiguration(instanceConfig));
    }

    /**
     * Instantiates a motor.
     * 
     * @param instanceConfig The instance config for this motor.
     * @return The motor object.
     */
    private SwerveMotor instantiateMotor(Config instanceConfig) {
        String template = instanceConfig.get("template");
        Config motorConfig = this.instantiateTemplateConfig(configData.get("motor-templates." + template), instanceConfig);

        switch (template) {
            case "falcon500":
                return this.instantiateFalcon500(motorConfig);
            case "neo":
                return this.instantiateNeo(motorConfig);
            default:
                throw new IllegalArgumentException(String.format("The template %s does not have a corresponding motor class.", template));
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
        Config moduleConfig = this.instantiateTemplateConfig(configData.get("module-templates." + template), instanceConfig);

        SwerveMotor motors[] = new SwerveMotor[]{this.instantiateMotor(moduleConfig.get("motors.0")), this.instantiateMotor(moduleConfig.get("motors.1"))};
        PositionSensor azimuthSensor = this.instantiateSensor(moduleConfig.get("azimuth-sensor"));

        return new SwerveModule(motors, azimuthSensor, new SwerveModuleConfiguration(moduleConfig));
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
