package frc.team88.swerve.swervemodule.motorsensor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.team88.swerve.util.constants.PIDPreferenceConstants;

/**
 * PIDMotor implementation for the NEO. Uses the built-in encoder and PID on the
 * Spark Max.
 */
public class PIDNeo extends CANSparkMax implements PIDMotor {

    // All of the PID gains
    private PIDPreferenceConstants pidConstants;

    // The offset to add to position values, in rotations.
    private double offset = 0;

    /**
     * Constructor. Sets up the default configuration for a SparkMax.
     * 
     * @param canID        The canID for the SparkMax
     * @param pidConstants All of the PID constants for velocity control. Uses all
     *                     constants except tolerance
     */
    public PIDNeo(int canID, PIDPreferenceConstants pidConstants) {
        super(canID, MotorType.kBrushless);

        this.restoreFactoryDefaults();
        this.setIdleMode(IdleMode.kBrake);

        this.pidConstants = pidConstants;
        pidConstants.getKP().addChangeHandler(this::setKP);
        pidConstants.getKI().addChangeHandler(this::setKI);
        pidConstants.getKD().addChangeHandler(this::setKD);
        pidConstants.getKF().addChangeHandler(this::setKF);
        pidConstants.getIZone().addChangeHandler(this::setIZone);
        pidConstants.getIMax().addChangeHandler(this::setIMax);
        this.setKP(pidConstants.getKP().getValue());
        this.setKI(pidConstants.getKI().getValue());
        this.setKD(pidConstants.getKD().getValue());
        this.setKF(pidConstants.getKF().getValue());
        this.setIZone(pidConstants.getIZone().getValue());
        this.setIMax(pidConstants.getIMax().getValue());
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

    /**
     * {@inheritDoc}
     * 
     * @return The motor shaft position in rotations per second
     */
    @Override
    public double getVelocity() {
        return this.getEncoder().getVelocity() / 60.;
    }

    /**
     * {@inheritDoc}
     * 
     * @return The position to set, in rotations
     */
    @Override
    public void calibratePosition(double position) {
        this.offset = position - this.getPosition() + this.offset;
    }

    /**
     * {@inheritDoc}
     * 
     * @return The velocity to set, in rotations per second
     */
    @Override
    public void setVelocity(double velocity) {
        this.getPIDController().setReference(velocity * 60., ControlType.kVelocity, 0);
    }

    /**
     * Set the kP constant for the Spark Max velocity control.
     * 
     * @param kP The proportional gain
     */
    private void setKP(double kP) {
        this.getPIDController().setP(kP);
    }

    /**
     * Set the kI constant for the Spark Max velocity control.
     * 
     * @param kI The integral gain
     */
    private void setKI(double kI) {
        this.getPIDController().setI(kI);
    }

    /**
     * Set the kD constant for the Spark Max velocity control.
     * 
     * @param kD The differential gain
     */
    private void setKD(double kD) {
        this.getPIDController().setD(kD);
    }

    /**
     * Set the kF constant for the Spark Max velocity control.
     * 
     * @param kF The feedforward gain
     */
    private void setKF(double kF) {
        this.getPIDController().setFF(kF);
    }

    /**
     * Set the iZone constant for the Spark Max velocity control.
     * 
     * @param iZone The max error which will accumulate in the integral
     */
    private void setIZone(double iZone) {
        this.getPIDController().setIZone(iZone);
    }

    /**
     * Set the iMax constant for the Spark Max velocity control.
     * 
     * @param iMax The max accumulated error for the integral
     */
    private void setIMax(double iMax) {
        this.getPIDController().setIMaxAccum(iMax, 0);
    }

}