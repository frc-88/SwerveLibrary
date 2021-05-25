package frc.team88.swerve.configuration.exceptions;

/**
 * Thrown when the user supplied config cannot be used to configure the swerve drive.
 */
public class SwerveConfigException extends RuntimeException {
    
    /**
     * Constructor.
     * 
     * @param message The error message.
     */
    public SwerveConfigException(String message) {
        super(message);
    }

    /**
     * Constructor.
     * 
     * @param message The error message.
     * @param cause The error that caused this exception.
     */
    public SwerveConfigException(String message, Throwable cause) {
        super(message, cause);
    }
}
