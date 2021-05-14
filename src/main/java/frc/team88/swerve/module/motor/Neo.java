package frc.team88.swerve.module.motor;

import java.util.Objects;

import com.revrobotics.CANSparkMax;

import frc.team88.swerve.configuration.NeoConfiguration;

/**
 * PIDMotor implementation for the NEO. Uses the built-in encoder and PID on the
 * Spark Max.
 */
public class Neo extends CANSparkMax implements SwerveMotor {

    // The configuration data for this motor.
    private final NeoConfiguration config;

    // The last commanded velocity.
    private double commandVelocity = 0;

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
    public Neo(int canID, NeoConfiguration config) {
        super(canID, MotorType.kBrushless);

        this.config = Objects.requireNonNull(config);

        this.restoreFactoryDefaults();
        this.setIdleMode(IdleMode.kBrake);
        this.setInverted(config.isInverted());
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
     * Offsets the sensor position such that the current position becomes the
     * given position.
     * 
     * @param position The position to set, in rotations
     */
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition() + this.offset;
    }

    @Override
    public void setVelocity(double velocity) {
        this.set(velocity / this.getMaxVelocity());
        this.commandVelocity = velocity;
    }

    @Override
    public double getMaxVelocity() {
        return this.config.getMaxSpeed();
    }

    @Override
    public double getCurrentDraw() {
        return this.getCurrentDraw();
    }

    @Override
    public double getCommandVoltage() {
        return this.getAppliedOutput() * this.getBusVoltage();
    }

    @Override
    public double getCommandVelocity() {
        return this.commandVelocity;
    }

    @Override
    public void setCoast() {
        this.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setBrake() {
        this.setIdleMode(IdleMode.kBrake);
    }

}