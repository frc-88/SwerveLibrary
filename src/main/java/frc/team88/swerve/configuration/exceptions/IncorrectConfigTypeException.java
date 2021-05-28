package frc.team88.swerve.configuration.exceptions;

/** Thrown when the type of a field is not the expected type. */
public class IncorrectConfigTypeException extends SwerveConfigException {

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public IncorrectConfigTypeException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public IncorrectConfigTypeException(String message, Throwable cause) {
    super(message, cause);
  }
}
