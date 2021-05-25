package frc.team88.swerve.configuration.exceptions;

/**
 * Thrown when the user-specified config cannot be parsed.
 */
public class InvalidConfigFormatException extends SwerveConfigException {
    
    /**
     * Constructor.
     * 
     * @param message The error message.
     */
    public InvalidConfigFormatException(String message) {
        super(message);
    }

    /**
     * Constructor.
     * 
     * @param message The error message.
     * @param cause The error that caused this exception.
     */
    public InvalidConfigFormatException(String message, Throwable cause) {
        super(message, cause);
    }
}
