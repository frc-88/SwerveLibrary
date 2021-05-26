package frc.team88.swerve.configuration.exceptions;

/** Thrown when the user-specified config cannot be found. */
public class ConfigNotFoundException extends SwerveConfigException {

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public ConfigNotFoundException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public ConfigNotFoundException(String message, Throwable cause) {
    super(message, cause);
  }
}
