package frc.team88.swerve.configuration.exceptions;

/** Thrown when a template can not be instantiated. */
public class TemplateInstantiationException extends SwerveConfigException {

  /**
   * Constructor.
   *
   * @param message The error message.
   */
  public TemplateInstantiationException(String message) {
    super(message);
  }

  /**
   * Constructor.
   *
   * @param message The error message.
   * @param cause The error that caused this exception.
   */
  public TemplateInstantiationException(String message, Throwable cause) {
    super(message, cause);
  }
}
