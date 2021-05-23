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
import frc.team88.swerve.commandmux.CommandMux;
import frc.team88.swerve.commandmux.CommandMuxEntry;
import frc.team88.swerve.module.sensor.SwerveCANcoder;

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

  // The command multiplexers from this configuration
  private CommandMux commandMux;

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
   */
  public Configuration(final String configPath, SwerveGyro gyro) {
    Objects.requireNonNull(configPath);
    this.canifiers = new HashMap<>();
    this.networkTableConfigs = new HashMap<>();

    ConfigParser<?> tomlParser = TomlFormat.instance().createParser();

    // Parse the base config file first
    this.configData = tomlParser.parse(getClass().getResourceAsStream("base_config.toml"));

    // Append and overwrite with the user-supplied config file
    tomlParser.parse(
        Filesystem.getDeployDirectory().toPath().resolve(configPath),
        this.configData,
        ParsingMode.MERGE,
        FileNotFoundAction.THROW_ERROR);

    // Create all of the objects and configs
    this.instantiateModules();
    if (Objects.nonNull(gyro)) {
      this.gyro = gyro;
    } else {
      this.instantiateGyro();
    }
    instantiateCommandMuxs();
  }

  /**
   * Loads the base config and user config from the filesystem. Instantiates the gyro from the
   * config.
   *
   * @param configPath The file path of the toml config. It can be a relative path inside of the
   *     deploy directory or an absolute path.
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
   * Gets the command mux specified by this config.
   *
   * @return The command mux.
   */
  public CommandMux getCommandMux() {
    return this.commandMux;
  }

  /**
   * Does a deep copy on the given config, to the level of making new inner config and list objects,
   * into the target config.
   *
   * @param configToCopy The config to copy from.
   * @param targetConfig The config to copy into. If it isn't empty, it will retain its contents,
   *     unless they are overwritten by a value in the config being copied.
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
            throw new IllegalArgumentException(
                String.format(
                    "Cannot combine configs because the value of %s is a list in one but not the"
                        + " other.",
                    key));
          }
          List<?> targetList = (List<?>) targetConfig.get(key);
          if (valueAsList.size() != targetList.size()) {
            throw new IllegalArgumentException(
                String.format(
                    "Cannot combine configs because %s contains lists of different lengths", key));
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
            throw new IllegalArgumentException(
                String.format(
                    "Cannot combine configs because the value of %s is a config in one but not the"
                        + " other.",
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
   * Instantiates a template's config by appending and overwritting the design with an
   * instance-specific config.
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
   */
  private NavX instantiateNavX(Config gyroConfig) {
    String portType = gyroConfig.get("port-type");
    switch (portType) {
      case "SPI":
        return new NavX(gyroConfig.getEnum("port", SPI.Port.class));
      case "I2C":
        return new NavX(gyroConfig.getEnum("port", I2C.Port.class));
      case "Serial":
        return new NavX(gyroConfig.getEnum("port", SerialPort.Port.class));
      default:
        throw new IllegalArgumentException(
            String.format("Invalid port type %s given in configuration.", portType));
    }
  }

  /** Instantiates the gyro object from the config. */
  private void instantiateGyro() {
    String template = this.configData.get("gyro.template");
    Config gyroConfig =
        this.instantiateTemplateConfig(
            configData.get("gyro-templates." + template), configData.get("gyro"));

    switch (template) {
      case "navx":
        this.gyro = this.instantiateNavX(gyroConfig);
        break;
      default:
        throw new IllegalArgumentException(
            String.format("The template %s does not have a corresponding gyro class.", template));
    }
  }

  private CommandMuxEntry instantiateCommandMux(Config instanceConfig, String networkTable) {
    String template = instanceConfig.get("template");
    Config config =
        this.instantiateTemplateConfig(
            configData.get("command-mux-templates." + template), instanceConfig);

    networkTable += "/mux/" + config.get("id");
    CommandMuxConfiguration muxConfig = new CommandMuxConfiguration(config);
    this.networkTableConfigs.put(networkTable, muxConfig);
    return new CommandMuxEntry(muxConfig);
  }

  /** Instantiates the command mux objects from the config. */
  private void instantiateCommandMuxs() {
    List<Config> muxConfigs = this.configData.get("command-mux");
    this.commandMux = new CommandMux(muxConfigs);
    for (int index = 0; index < muxConfigs.size(); index++) {
      this.commandMux.instantiateMux(index, this.instantiateCommandMux(muxConfigs.get(index), "/commands"));
    }
  }

  /**
   * Instantiates a PWM position sensor on a Canifier. Will create the Canifier object if it does
   * not already exist.
   *
   * @param instanceConfig The config for the canified pwm sensor.
   * @param networkTable The key to use for the network table.
   * @return The canified pwm sensor object.
   */
  private SensorTransmission instantiateCanifiedPWM(Config instanceConfig, String networkTable) {
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
        throw new IllegalArgumentException(
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
   */
  private SensorTransmission instantiateCANCoder(Config instanceConfig, String networkTable) {

    SensorTransmissionConfiguration sensorConfig =
        new SensorTransmissionConfiguration(instanceConfig);
    this.networkTableConfigs.put(networkTable, sensorConfig);

    SwerveCANcoder rawSensor = new SwerveCANcoder(instanceConfig.getInt("can-id"));
    return new SensorTransmission(rawSensor, sensorConfig);
  }

  /**
   * Instantiates a position sensor.
   *
   * @param instanceConfig The config for the sensor.
   * @param networkTable The key to use for the network table.
   * @return The sensor object.
   */
  private SensorTransmission instantiateSensor(Config instanceConfig, String networkTable) {
    String template = instanceConfig.get("template");
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
            String.format("The template %s does not have a corresponding sensor class.", template));
    }
  }

  /**
   * Instantiates a Falcon 500.
   *
   * @param instanceConfig The config for the Falcon 500.
   * @param networkTable The key to use for the network table.
   * @return The falcon object.
   */
  private Falcon500 instantiateFalcon500(Config instanceConfig, String networkTable) {
    Falcon500Configuration falconConfig = new Falcon500Configuration(instanceConfig);
    this.networkTableConfigs.put(networkTable, falconConfig);
    return new Falcon500(instanceConfig.getInt("can-id"), falconConfig);
  }

  /**
   * Instantiates a Neo.
   *
   * @param instanceConfig The config for the Neo.
   * @param networkTable The key to use for the network table.
   * @return The neo object.
   */
  private Neo instantiateNeo(Config instanceConfig, String networkTable) {
    NeoConfiguration neoConfig = new NeoConfiguration(instanceConfig);
    this.networkTableConfigs.put(networkTable, neoConfig);
    return new Neo(instanceConfig.getInt("can-id"), neoConfig);
  }

  /**
   * Instantiates a motor.
   *
   * @param instanceConfig The instance config for this motor.
   * @param networkTable The key to use for the network table.
   * @return The motor object.
   */
  private SwerveMotor instantiateMotor(Config instanceConfig, String networkTable) {
    String template = instanceConfig.get("template");
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
            String.format("The template %s does not have a corresponding motor class.", template));
    }
  }

  /**
   * Instantiates an individual module.
   *
   * @param instanceConfig The instance config for this module.
   * @param networkTable The key to use for the network table.
   * @return The module object.
   */
  private SwerveModule instantiateModule(Config instanceConfig, String networkTable) {
    String template = instanceConfig.get("template");
    Config moduleConfig =
        this.instantiateTemplateConfig(
            configData.get("module-templates." + template), instanceConfig);

    SwerveMotor motors[] =
        new SwerveMotor[] {
          this.instantiateMotor(moduleConfig.get("motors.0"), networkTable + "/motors/0"),
          this.instantiateMotor(moduleConfig.get("motors.1"), networkTable + "/motors/1")
        };
    PositionSensor azimuthSensor =
        this.instantiateSensor(moduleConfig.get("azimuth-sensor"), networkTable + "/sensor");

    SwerveModuleConfiguration swerveModuleConfig = new SwerveModuleConfiguration(moduleConfig);
    this.networkTableConfigs.put(networkTable, swerveModuleConfig);

    return new SwerveModule(motors, azimuthSensor, swerveModuleConfig);
  }

  /** Instantiates the module objects from the config. */
  private void instantiateModules() {
    List<Config> moduleConfigs = this.configData.get("modules");
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
}
