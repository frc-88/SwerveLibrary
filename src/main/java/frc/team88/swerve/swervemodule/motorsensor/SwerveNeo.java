package frc.team88.swerve.swervemodule.motorsensor;

import java.util.Objects;

import com.revrobotics.CANSparkMax;

import frc.team88.swerve.configuration.NeoConfiguration;

/**
 * PIDMotor implementation for the NEO. Uses the built-in encoder and PID on the
 * Spark Max.
 */
public class SwerveNeo extends CANSparkMax implements SwerveMotor {

    // The configuration data for this motor.
    private final NeoConfiguration config;

    // The offset to add to position values, in rotations.
    private double offset = 0;

    /**
     * Constructor. Sets up the default configuration for a Spark Max.
     * 
     * @param canID
     *                         The canID for the Spark Max.
     * @param config
     *                         The config data for this motor.
     */
    public SwerveNeo(int canID, NeoConfiguration config) {
        super(canID, MotorType.kBrushless);

        this.config = Objects.requireNonNull(config);

        this.restoreFactoryDefaults();
        this.setIdleMode(IdleMode.kBrake);
    }

    /**
     * {@inheritDoc}
     * 
     * @return The motor shaft position in rotations
     */
    @Override
    public double getPosition() {
        return this.getEncoder().getPosition() + offset;
    }

    @Override
    public double getVelocity() {
        return this.getEncoder().getVelocity() / 60.;
    }

    /**
     * {@inheritDoc}
     * 
     * @param position The position to set, in rotations
     */
    @Override
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition() + this.offset;
    }

    @Override
    public void setVelocity(double velocity) {
        this.set(velocity / this.getMaxVelocity());
    }

    @Override
    public double getMaxVelocity() {
        return this.config.getMaxSpeed();
    }

}