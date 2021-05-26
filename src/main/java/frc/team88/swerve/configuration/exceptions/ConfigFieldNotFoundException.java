package frc.team88.swerve.configuration.exceptions;

/** Thrown when a required field is not found in the config. */
public class ConfigFieldNotFoundException extends SwerveConfigException {

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public ConfigFieldNotFoundException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public ConfigFieldNotFoundException(String message, Throwable cause) {
    super(message, cause);
  }
}
