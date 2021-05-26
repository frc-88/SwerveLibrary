package frc.team88.swerve.configuration.exceptions;

/** Thrown when the value of a field in the config is invalid. */
public class InvalidConfigValueException extends SwerveConfigException {

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public InvalidConfigValueException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public InvalidConfigValueException(String message, Throwable cause) {
    super(message, cause);
  }
}
