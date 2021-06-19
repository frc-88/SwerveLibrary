package frc.team88.swerve.configuration.exceptions;

/** Thrown when an invalid template is given in a config. */
public class InvalidTemplateException extends SwerveConfigException {

  private static final long serialVersionUID = 0L;

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public InvalidTemplateException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public InvalidTemplateException(String message, Throwable cause) {
    super(message, cause);
  }
}
