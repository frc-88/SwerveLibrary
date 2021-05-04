package frc.team88.swerve;

import java.util.Objects;

import frc.team88.swerve.configuration.Configuration;

/**
 * The high-level API for controlling a swerve drive with this library.
 */
public class SwerveController {

    // The configuration data
    private final Configuration config;
    
    /**
     * Constructs the SwerveController using the given toml config.
     * 
     * @param configPath The file path of the toml config. It can be a
     *                   relative path inside of the deploy directory or an
     *                   absolute path.
     */
    public SwerveController(final String configPath) {
        Objects.requireNonNull(configPath);

        this.config = new Configuration(configPath);
    }
}
