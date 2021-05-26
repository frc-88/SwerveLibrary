package frc.team88.swerve.configuration;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.electronwill.nightconfig.core.Config;
import com.electronwill.nightconfig.core.file.FileNotFoundAction;
import com.electronwill.nightconfig.core.io.ConfigParser;
import com.electronwill.nightconfig.core.io.ParsingMode;
import com.electronwill.nightconfig.toml.TomlFormat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team88.swerve.configuration.exceptions.ConfigFieldNotFoundException;
import frc.team88.swerve.configuration.exceptions.ConfigNotFoundException;
import frc.team88.swerve.configuration.exceptions.IncorrectConfigTypeException;
import frc.team88.swerve.configuration.exceptions.InvalidConfigFormatException;
import frc.team88.swerve.configuration.exceptions.InvalidConfigValueException;
import frc.team88.swerve.configuration.exceptions.InvalidTemplateException;
import frc.team88.swerve.configuration.exceptions.SwerveConfigException;
import frc.team88.swerve.configuration.exceptions.TemplateInstantiationException;
import frc.team88.swerve.configuration.subconfig.Falcon500Configuration;
import frc.team88.swerve.configuration.subconfig.NeoConfiguration;
import frc.team88.swerve.configuration.subconfig.SensorTransmissionConfiguration;
import frc.team88.swerve.configuration.subconfig.SwerveModuleConfiguration;
import frc.team88.swerve.data.NetworkTablePopulator;
import frc.team88.swerve.gyro.NavX;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.module.motor.Falcon500;
import frc.team88.swerve.module.motor.Neo;
import frc.team88.swerve.module.motor.SwerveMotor;
import frc.team88.swerve.module.sensor.CANifiedPWMEncoder;
import frc.team88.swerve.module.sensor.PositionSensor;
import frc.team88.swerve.module.sensor.SensorTransmission;
import frc.team88.swerve.module.sensor.SwerveCANcoder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Parses a swerve configuration file, generates objects from it, and provides access to all of its
 * contents.
 */
public class Configuration implements NetworkTablePopulator {

  // The loaded config data
  private final Config configData;

  // The gyro from this configuration
  private SwerveGyro gyro;

  // The swerve modules from this configuration
  private SwerveModule[] modules;

  // The canifiers used by sensors in this configuration
  private final Map<Integer, CANifier> canifiers;

  // The configurations that will be placed in NetworkTables to allow for
  // modification, with the keys being the table key.
  private final Map<String, NetworkTablePopulator> networkTableConfigs;

  /**
   * Loads the base config and user config from the filesystem.
   *
   * @param configPath The file path of the toml config. It can be a relative path inside of the
   *     deploy directory or an absolute path.
   * @param gyro The gyro to use. Will instantiate from config if null.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  public Configuration(final String configPath, SwerveGyro gyro) {
    Objects.requireNonNull(configPath);
    this.canifiers = new HashMap<>();
    this.networkTableConfigs = new HashMap<>();

    ConfigParser<?> tomlParser = TomlFormat.instance().createParser();

    // Parse the base config file first
    this.configData = tomlParser.parse(getClass().getResourceAsStream("base_config.toml"));

    // Append and overwrite with the user-supplied config file
    Path userConfigPath;
    try {
      userConfigPath = Filesystem.getDeployDirectory().toPath().resolve(configPath);
    } catch (Exception e) {
      throw new ConfigNotFoundException(
          "Error encountered getting path of file " + configPath + " in deploy directory.", e);
    }
    if (!Files.exists(userConfigPath)) {
      throw new ConfigNotFoundException(
          "The given config file " + configPath + " does not exist in the deploy directory.");
    } else if (Files.isDirectory(userConfigPath)) {
      throw new ConfigNotFoundException(
          "The given config file " + configPath + " is actually a directory.");
    }
    try {
      tomlParser.parse(
          userConfigPath, this.configData, ParsingMode.MERGE, FileNotFoundAction.THROW_ERROR);
    } catch (Exception e) {
      throw new InvalidConfigFormatException(
          "The given config file " + configPath + " is not a valid TOML file.", e);
    }

    // Create all of the objects and configs
    this.instantiateModules();
    if (Objects.nonNull(gyro)) {
      this.gyro = gyro;
    } else {
      this.instantiateGyro();
    }
  }

  /**
   * Loads the base config and user config from the filesystem. Instantiates the gyro from the
   * config.
   *
   * @param configPath The file path of the toml config. It can be a relative path inside of the
   *     deploy directory or an absolute path.
   * @throws SwerveConfigException If the user provided config is incorrect.
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
   * Gets a mapping from can IDs to canifiers instantiated by this configuration.
   *
   * @return The mapping from can IDs to CANifiers.
   */
  public Map<Integer, CANifier> getCanifiers() {
    return this.canifiers;
  }

  /**
   * Does a deep copy on the given config, to the level of making new inner config and list objects,
   * into the target config.
   *
   * @param configToCopy The config to copy from.
   * @param targetConfig The config to copy into. If it isn't empty, it will retain its contents,
   *     unless they are overwritten by a value in the config being copied.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private void deepCopyConfig(Config configToCopy, Config targetConfig) {
    for (Config.Entry entry : configToCopy.entrySet()) {
      Object value = entry.getValue();
      String key = entry.getKey();
      if (value instanceof List<?>) {
        List<?> valueAsList = (List<?>) value;
        List<Object> newList;
        if (targetConfig.contains(key)) {
          if (!(targetConfig.get(key) instanceof List)) {
            throw new TemplateInstantiationException(
                String.format(
                    "Cannot instantiate template because the value of %s is a list in the template config but not in"
                        + " the user config.",
                    key));
          }
          List<?> targetList = (List<?>) targetConfig.get(key);
          if (valueAsList.size() != targetList.size()) {
            throw new TemplateInstantiationException(
                String.format(
                    "Cannot instantiate template because %s contains lists of different lengths",
                    key));
          }
          newList = new ArrayList<Object>(targetList);
        } else {
          newList = new ArrayList<Object>();
        }
        for (Object subValue : valueAsList) {
          if (subValue instanceof Config) {
            Config subConfig = Config.inMemory();
            deepCopyConfig((Config) subValue, subConfig);
            newList.add(subConfig);
          } else {
            newList.add(subValue);
          }
        }
        targetConfig.set(key, newList);
      } else if (value instanceof Config) {
        Config subConfig;
        if (targetConfig.contains(key)) {
          if (!(targetConfig.get(key) instanceof Config)) {
            throw new TemplateInstantiationException(
                String.format(
                    "Cannot instantiate template because the value of %s is a table in the template config but not the"
                        + " user config.",
                    key));
          }
          subConfig = targetConfig.get(key);
        } else {
          subConfig = Config.inMemory();
        }
        deepCopyConfig((Config) value, subConfig);
        targetConfig.set(entry.getKey(), subConfig);
      } else {
        targetConfig.set(entry.getKey(), value);
      }
    }
  }

  /**
   * Instantiates a template's config by apppending and overwritting the design with an
   * instance-specific config.
   *
   * @param templateConfig The config of the template being used as a base.
   * @param instanceConfig The config of the instance implementing the template.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private Config instantiateTemplateConfig(Config templateConfig, Config instanceConfig) {
    Config instantiatedConfig = Config.inMemory();
    // By copying the instance config second, it will overwrite values in
    // the template config.
    this.deepCopyConfig(templateConfig, instantiatedConfig);
    this.deepCopyConfig(instanceConfig, instantiatedConfig);
    return instantiatedConfig;
  }

  /**
   * Instatiates a NavX object.
   *
   * @param gyroConfig The config for the gyro.
   * @return The navx object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private NavX instantiateNavX(Config gyroConfig) {
    String portType = configCheckAndGet(gyroConfig, "port-type", String.class);
    switch (portType) {
      case "SPI":
        return new NavX(configCheckAndGetEnum(gyroConfig, "port", SPI.Port.class));
      case "I2C":
        return new NavX(configCheckAndGetEnum(gyroConfig, "port", I2C.Port.class));
      case "Serial":
        return new NavX(configCheckAndGetEnum(gyroConfig, "port", SerialPort.Port.class));
      default:
        throw new InvalidConfigValueException(
            String.format("Invalid port type %s given in configuration.", portType));
    }
  }

  /**
   * Instantiates the gyro object from the config.
   *
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private void instantiateGyro() {
    Config instanceConfig = configCheckAndGet(configData, "gyro", Config.class);
    String template = configCheckAndGet(instanceConfig, "template", String.class);
    if (!this.configData.contains("gyro-templates." + template)) {
      throw new InvalidTemplateException("The gyro template " + template + " does not exist.");
    }
    Config gyroConfig =
        this.instantiateTemplateConfig(
            configData.get("gyro-templates." + template), instanceConfig);

    switch (template) {
      case "navx":
        this.gyro = this.instantiateNavX(gyroConfig);
        break;
      default:
        throw new IllegalArgumentException(
            String.format(
                "The template %s does not have a corresponding gyro class. Please contact the library developer.",
                template));
    }
  }

  /**
   * Instantiates a PWM position sensor on a Canifier. Will create the Canifier object if it does
   * not already exist.
   *
   * @param instanceConfig The config for the canified pwm sensor.
   * @param networkTable The key to use for the network table.
   * @return The canified pwm sensor object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private SensorTransmission instantiateCanifiedPWM(Config instanceConfig, String networkTable) {
    int canID = configCheckAndGet(instanceConfig, "can-id", Integer.class);
    if (canID < 0 || canID >= 64) {
      throw new InvalidConfigValueException(
          String.format("CAN ID %d is not in range [0, 63]", canID));
    }
    if (!this.canifiers.containsKey(canID)) {
      this.canifiers.put(canID, new CANifier(canID));
    }

    int pwmChannel = configCheckAndGet(instanceConfig, "pwm-channel", Integer.class);
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
        throw new InvalidConfigValueException(
            String.format(
                "%d is not a valid PWM channel. It must be between 0 and 3, inclusive",
                pwmChannel));
    }

    SensorTransmissionConfiguration sensorConfig =
        new SensorTransmissionConfiguration(instanceConfig);
    this.networkTableConfigs.put(networkTable, sensorConfig);

    CANifiedPWMEncoder rawSensor = new CANifiedPWMEncoder(this.canifiers.get(canID), channel);
    return new SensorTransmission(rawSensor, sensorConfig);
  }

  /**
   * Instantiates a CANCoder.
   *
   * @param instanceConfig The config for the CANCoder.
   * @param networkTable The key to use for the network table.
   * @return The CANCoder object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private SensorTransmission instantiateCANCoder(Config instanceConfig, String networkTable) {

    SensorTransmissionConfiguration sensorConfig =
        new SensorTransmissionConfiguration(instanceConfig);
    this.networkTableConfigs.put(networkTable, sensorConfig);

    SwerveCANcoder rawSensor =
        new SwerveCANcoder(configCheckAndGet(instanceConfig, "can-id", Integer.class));
    return new SensorTransmission(rawSensor, sensorConfig);
  }

  /**
   * Instantiates a position sensor.
   *
   * @param instanceConfig The config for the sensor.
   * @param networkTable The key to use for the network table.
   * @return The sensor object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private SensorTransmission instantiateSensor(Config instanceConfig, String networkTable) {
    String template = configCheckAndGet(instanceConfig, "template", String.class);
    if (!this.configData.contains("sensor-templates." + template)) {
      throw new InvalidTemplateException("The sensor template " + template + " does not exist.");
    }
    Config sensorConfig =
        this.instantiateTemplateConfig(
            configData.get("sensor-templates." + template), instanceConfig);

    switch (template) {
      case "canified-pwm":
        return this.instantiateCanifiedPWM(sensorConfig, networkTable);
      case "cancoder":
        return this.instantiateCANCoder(sensorConfig, networkTable);
      default:
        throw new IllegalArgumentException(
            String.format(
                "The template %s does not have a corresponding sensor class. Please contact the library developer.",
                template));
    }
  }

  /**
   * Instantiates a Falcon 500.
   *
   * @param instanceConfig The config for the Falcon 500.
   * @param networkTable The key to use for the network table.
   * @return The falcon object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private Falcon500 instantiateFalcon500(Config instanceConfig, String networkTable) {
    Falcon500Configuration falconConfig = new Falcon500Configuration(instanceConfig);
    this.networkTableConfigs.put(networkTable, falconConfig);

    int canID = configCheckAndGet(instanceConfig, "can-id", Integer.class);
    if (canID < 0 || canID >= 64) {
      throw new InvalidConfigValueException(
          String.format("CAN ID %d is not in range [0, 63]", canID));
    }
    return new Falcon500(canID, falconConfig);
  }

  /**
   * Instantiates a Neo.
   *
   * @param instanceConfig The config for the Neo.
   * @param networkTable The key to use for the network table.
   * @return The neo object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private Neo instantiateNeo(Config instanceConfig, String networkTable) {
    NeoConfiguration neoConfig = new NeoConfiguration(instanceConfig);
    this.networkTableConfigs.put(networkTable, neoConfig);

    int canID = configCheckAndGet(instanceConfig, "can-id", Integer.class);
    if (canID < 0 || canID >= 64) {
      throw new InvalidConfigValueException(
          String.format("CAN ID %d is not in range [0, 63]", canID));
    }
    return new Neo(canID, neoConfig);
  }

  /**
   * Instantiates a motor.
   *
   * @param instanceConfig The instance config for this motor.
   * @param networkTable The key to use for the network table.
   * @return The motor object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private SwerveMotor instantiateMotor(Config instanceConfig, String networkTable) {
    String template = configCheckAndGet(instanceConfig, "template", String.class);
    if (!this.configData.contains("motor-templates." + template)) {
      throw new InvalidTemplateException("The motor template " + template + " does not exist.");
    }
    Config motorConfig =
        this.instantiateTemplateConfig(
            configData.get("motor-templates." + template), instanceConfig);

    switch (template) {
      case "falcon500":
        return this.instantiateFalcon500(motorConfig, networkTable);
      case "neo":
        return this.instantiateNeo(motorConfig, networkTable);
      default:
        throw new IllegalArgumentException(
            String.format(
                "The template %s does not have a corresponding motor class. Please contact the library developer.",
                template));
    }
  }

  /**
   * Finds the config for the motor in the given config. If it has an assigned name, that will be
   * used, otherwise the defaults of [0, 1] will be used.
   *
   * @param moduleConfig The config to be searched.
   * @param motorIdx The index of the motor, either 0 or 1.
   * @return The subconfig for the motor in the provided config.
   * @throw SwerveConfigException If the motor is not in the config or is not a table.
   */
  private Config findMotorConfig(Config moduleConfig, int motorIdx) {
    String motorKey = "";

    String nameKey = "motor-" + motorIdx + "-name";
    if (moduleConfig.contains(nameKey)) {
      if (!(moduleConfig.get(nameKey) instanceof String)) {
        throw new IncorrectConfigTypeException(nameKey + " must be a string.");
      }
      motorKey = "motors." + moduleConfig.get(nameKey);
    } else {
      motorKey = "motors." + motorIdx;
    }

    if (!moduleConfig.contains(motorKey)) {
      throw new ConfigFieldNotFoundException(
          "The module config is missing motor at " + motorKey + ".");
    } else if (!(moduleConfig.get(motorKey) instanceof Config)) {
      throw new IncorrectConfigTypeException("The motor " + motorKey + " must be a table.");
    } else {
      return moduleConfig.get(motorKey);
    }
  }

  /**
   * Instantiates an individual module.
   *
   * @param instanceConfig The instance config for this module.
   * @param networkTable The key to use for the network table.
   * @return The module object.
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private SwerveModule instantiateModule(Config instanceConfig, String networkTable) {
    Config moduleConfig;
    if (instanceConfig.contains("template")) {
      if (!(instanceConfig.get("template") instanceof String)) {
        throw new IncorrectConfigTypeException("Field 'template' must be of type String");
      }
      String template = instanceConfig.get("template");
      if (!this.configData.contains("module-templates." + template)) {
        throw new InvalidTemplateException("The module template " + template + " does not exist.");
      }
      moduleConfig =
          this.instantiateTemplateConfig(
              this.configData.get("module-templates." + template), instanceConfig);
    } else {
      moduleConfig = instanceConfig;
    }

    SwerveMotor motors[] =
        new SwerveMotor[] {
          this.instantiateMotor(findMotorConfig(moduleConfig, 0), networkTable + "/motors/0"),
          this.instantiateMotor(findMotorConfig(moduleConfig, 1), networkTable + "/motors/1")
        };
    PositionSensor azimuthSensor =
        this.instantiateSensor(
            configCheckAndGet(moduleConfig, "azimuth-sensor", Config.class),
            networkTable + "/sensor");

    SwerveModuleConfiguration swerveModuleConfig = new SwerveModuleConfiguration(moduleConfig);
    this.networkTableConfigs.put(networkTable, swerveModuleConfig);

    return new SwerveModule(motors, azimuthSensor, swerveModuleConfig);
  }

  /**
   * Instantiates the module objects from the config.
   *
   * @throws SwerveConfigException If the user provided config is incorrect.
   */
  private void instantiateModules() {
    if (!this.configData.contains("modules")) {
      throw new ConfigFieldNotFoundException("No modules specified in config.");
    }
    List<Config> moduleConfigs = this.configData.get("modules");
    if (moduleConfigs.size() < 2) {
      throw new InvalidConfigValueException(
          String.format(
              "Only %d modules foundd in config, at least 2 required.", moduleConfigs.size()));
    }
    this.modules = new SwerveModule[moduleConfigs.size()];

    for (int moduleIndex = 0; moduleIndex < moduleConfigs.size(); moduleIndex++) {
      this.modules[moduleIndex] =
          this.instantiateModule(moduleConfigs.get(moduleIndex), "/modules/" + moduleIndex);
    }
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    for (Map.Entry<String, NetworkTablePopulator> entry : this.networkTableConfigs.entrySet()) {
      entry.getValue().populateNetworkTable(table.getSubTable(entry.getKey()));
    }
  }

  /**
   * Checks the existence and type of the given config field, then returns the value.
   *
   * @param <T> The type of the field.
   * @param config The config to get the field from.
   * @param key The key of the field to get.
   * @param type The expected type of the field.
   * @return The value of the field.
   * @throws SwerveConfigException If the key does not exist or is not the right type.
   */
  public static <T> T configCheckAndGet(Config config, String key, Class<T> type) {
    if (!config.contains(key)) {
      throw new ConfigFieldNotFoundException("Field '" + key + "' not found in config.");
    }
    Object value = config.get(key);
    if (!type.isInstance(value)) {
      throw new IncorrectConfigTypeException(
          "Field '"
              + key
              + "' is of type "
              + value.getClass().getSimpleName()
              + ", but should be of type "
              + type.getSimpleName()
              + ".");
    }
    return type.cast(value);
  }

  /**
   * Checks that the given field exists and is a number, then returns it as a double.
   *
   * @param config The config to get the field from.
   * @param key The key of the field to get.
   * @return The value of the field.
   * @throws SwerveConfigException If the key does not exist or is not a number.
   */
  public static double configCheckAndGetDouble(Config config, String key) {
    Number value = configCheckAndGet(config, key, Number.class);
    return value.doubleValue();
  }

  /**
   * Checks that the given field exists and is an enum, then returns the value.
   *
   * @param <T> The type of the field.
   * @param config The config to get the field from.
   * @param key The key of the field to get.
   * @param enumType The expected enum type of the field.
   * @return The value of the field.
   * @throws SwerveConfigException If the key does not exist, is not a string, or is not in the
   *     enum.
   */
  public static <T extends Enum<T>> T configCheckAndGetEnum(
      Config config, String key, Class<T> enumType) {
    String value = configCheckAndGet(config, key, String.class);
    try {
      return Enum.valueOf(enumType, value);
    } catch (IllegalArgumentException e) {
      throw new InvalidConfigValueException(
          "'" + value + "' is not a valid value of enum " + enumType.getSimpleName() + ".", e);
    }
  }

  /**
   * If the given field exists, check it's type then return it's value. Otherwise, return the
   * default.
   *
   * @param <T> The type of the field.
   * @param config The config to get the field from.
   * @param key The key of the field to get.
   * @param type The expected type of the field.
   * @param defaultValue The value to return if the field doesn't exist.
   * @return The value of the field.
   * @throws SwerveConfigException If the key is not the right type.
   */
  public static <T> T configCheckAndGetOrElse(
      Config config, String key, Class<T> type, T defaultValue) {
    if (!config.contains(key)) {
      return defaultValue;
    }
    Object value = config.get(key);
    if (!type.isInstance(value)) {
      throw new IncorrectConfigTypeException(
          "Field '"
              + key
              + "' is of type "
              + value.getClass().getSimpleName()
              + ", but should be of type "
              + type.getSimpleName()
              + ".");
    }
    return type.cast(value);
  }

  /**
   * If the given field exists, check that it's a number then return it as a double. Otherwise,
   * return the default.
   *
   * @param config The config to get the field from.
   * @param key The key of the field to get.
   * @param defaultValue The value to return if the field doesn't exist.
   * @return The value of the field.
   * @throws SwerveConfigException If the key is not a number.
   */
  public static double configCheckAndGetDoubleOrElse(
      Config config, String key, double defaultValue) {
    Number value = configCheckAndGetOrElse(config, key, Number.class, 0.);
    return value.doubleValue();
  }
}
