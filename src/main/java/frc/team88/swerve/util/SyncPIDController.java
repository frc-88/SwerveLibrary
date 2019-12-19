package frc.team88.swerve.util;

import edu.wpi.first.wpilibj.RobotController;

/**
 * Class that performs the math of a PID controller, converting a setpoint and 
 * error to an output.
 */
public class SyncPIDController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double iZone;
    private double iMax;
    private double tolerance;

    private boolean enableIZone;
    private boolean enableIMax;

    private double m_accum = 0;
    private double m_prevError = 0;
    private long m_lastLoopTime = 0; // us

    /**
     * Constructor which specifies all parameters.
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The differential gain
     * @param kF The feedforward gain
     * @param iZone The integral will only accumulate while the current error
     * is less than this value
     * @param iMax The maximum absolute value that the integral error will
     * accumulate
     */
    public SyncPIDController(double kP, double kI, double kD, double kF, 
            double iZone, double iMax) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
        this.iMax = iMax;
        this.tolerance = 0;

        this.enableIZone = true;
        this.enableIMax = true;
    }

    /**
     * Constructor with only the basic parameters.
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The differential gain
     */
    public SyncPIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);

        this.enableIZone = false;
        this.enableIMax = false;
    }
    
    /**
     * Sets the proportial constant for the PID.
     * @param kP The proportional gain
     */
    public void setKP(double kP) {
        this.kP = kP;
    }
    /**
     * Sets the integral constant for the PID.
     * @param kI The integral gain
     */
    public void setKI(double kI) {
        this.kI = kI;
    }
    /**
     * Sets the differential constant for the PID.
     * @param kD The differential gain
     */
    public void setKD(double kD) {
        this.kD = kD;
    }
    /**
     * Sets the feedforward constant for the PID.
     * @param kF The feedforward gain
     */
    public void setKF(double kF) {
        this.kF = kF;
    }
    /**
     * Sets the maximum error for the integral to accumulate, and enable the 
     * iZone functionality
     * @param iZone the maximum error for integral accumulation
     */
    public void setIZone(double iZone) {
        this.iZone = iZone;
        this.enableIZone = true;
    }
    /**
     * Disables the iZone functionality.
     */
    public void disableIZone() {
        this.enableIZone = false;
    }
    /**
     * Sets the maximum error the integral will accumulate, and enable the 
     * iMax functionality
     * @param iMax The maximum accumulated error
     */
    public void setIMax(double iMax) {
        this.iMax = iMax;
        this.enableIMax = true;
    }
    /**
     * Disables the iZone functionality.
     */
    public void disableIMax() {
        this.enableIMax = false;
    }

    /**
     * Gets the proportial constant for the PID.
     * @return The proportional gain
     */
    public double getKP() {
        return this.kP;
    }
    /**
     * Gets the integral constant for the PID.
     * @return The integral gain
     */
    public double getKI() {
        return this.kI;
    }
    /**
     * Gets the derivative constant for the PID.
     * @return The differential gain
     */
    public double getKD() {
        return this.kD;
    }
    /**
     * Gets the feedforward constant for the PID.
     * @return the feedforward gain
     */
    public double getkF() {
        return this.kF;
    }
    /**
     * Gets the iZone constant for the PID.
     * @return the maximum error for integral accumulation
     */
    public double getIZone() {
        return this.iZone;
    }
    /**
     * Gets if iZone functionality is enabled.
     * @return True if iZone functionality is enabled, false otherwise
     */
    public boolean isIZoneEnabled() {
        return this.enableIZone;
    }
    /**
     * Gets the iMax constant for the PID.
     * @return The maximum accumulated error
     */
    public double getIMax() {
        return this.iMax;
    }
    /**
     * Gets if iMax functionality is enabled.
     * @return True if iMax functionality is enabled, false otherwise
     */
    public boolean isIMaxEnabled() {
        return this.enableIMax;
    }
    /**
     * Gets the tolerance for the PID.
     * @return The max error to consider the PID on target
     */
    public double getTolerance() {
        return this.tolerance;
    }
    /**
     * Set the tolerance for the PID.
     * @param tolerance The max error to consider the PID on target
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
    

    /**
     * Resets the controller's saved info about integral and derivative.
     * Should be called right before the first use of the controller after
     * it hasn't been used for a while.
     */
    public void reset() {
        this.m_accum = 0;
        this.m_prevError = 0;
        this.m_lastLoopTime = 0;
    }

    /**
     * Gets the output calculated by this PID.
     * @param input The current input value
     * @param setpoint The target input value
     * @return The output value
     */
    public double calculateOutput(double input, double setpoint) {
        double error = setpoint - input;

        if (Math.abs(error) < tolerance) {
            return 0;
        }

        double output = 0;
        output += calculateP(error);
        output += calculateI(error);
        output += calculateD(error);
        output += calculateF(setpoint);
        
        return output;
    }

    /**
     * Calculate the proportional output
     * @param error The current error
     * @return The proportional output
     */
    protected double calculateP(double error) {
        return kP * error;
    }

    /**
     * Calculate the integral output
     * @param error The current error
     * @return The integral output
     */
    protected double calculateI(double error) {
        
        if (!enableIZone || Math.abs(error) < iZone) {
            m_accum += error;
        }

        if (enableIMax) {
            if (m_accum > 0) {
                m_accum = Math.min(m_accum, iMax/kI);
            } else {
                m_accum = Math.max(m_accum, -iMax/kI);
            }
        }

        return kI * m_accum;
    }

    /**
     * Calculate the differential output
     * @param error The current error
     * @return The differential output
     */
    protected double calculateD(double error) {

        long curTime = RobotController.getFPGATime();

        double ret = kD * (error - m_prevError) 
                / ((curTime - m_lastLoopTime) / 1e6);

        m_prevError = error;
        m_lastLoopTime = curTime;

        return ret;

    }

    /**
     * Calulcate the feedforward output
     * @param setpoint The current setpoint
     * @return The feedforward output
     */
    protected double calculateF(double setpoint) {
        return kF * setpoint;
    }
}