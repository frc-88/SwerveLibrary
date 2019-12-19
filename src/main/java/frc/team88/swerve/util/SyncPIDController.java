package frc.team88.swerve.util;

import frc.team88.swerve.util.wpilibwrappers.RobotControllerWrapper;

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

    public SyncPIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);

        this.enableIZone = false;
        this.enableIMax = false;
    }
    
    /**
     * Sets the proportial constant for the PID.
     */
    public void setKP(double kP) {
        this.kP = kP;
    }
    /**
     * Sets the integral constant for the PID.
     */
    public void setKI(double kI) {
        this.kI = kI;
    }
    /**
     * Sets the differential constant for the PID.
     */
    public void setKD(double kD) {
        this.kD = kD;
    }
    /**
     * Sets the feedforward constant for the PID.
     */
    public void setKF(double kF) {
        this.kF = kF;
    }
    /**
     * Sets the maximum error for the integral to accumulate, and enable the 
     * iZone functionality
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
     */
    public double getKP() {
        return this.kP;
    }
    /**
     * Gets the integral constant for the PID.
     */
    public double getKI() {
        return this.kI;
    }
    /**
     * Gets the derivative constant for the PID.
     */
    public double getKD() {
        return this.kD;
    }
    /**
     * Gets the feedforward constant for the PID.
     */
    public double getkF() {
        return this.kF;
    }
    /**
     * Gets the iZone constant for the PID.
     */
    public double getIZone() {
        return this.iZone;
    }
    /**
     * Gets if iZone functionality is enabled.
     */
    public boolean isIZoneEnabled() {
        return this.enableIZone;
    }
    /**
     * Gets the iMax constant for the PID.
     */
    public double getIMax() {
        return this.iMax;
    }
    /**
     * Gets if iMax functionality is enabled.
     */
    public boolean isIMaxEnabled() {
        return this.enableIMax;
    }
    /**
     * Gets the tolerance for the PID
     */
    public double getTolerance() {
        return this.tolerance;
    }
    /**
     * Set the tolerance for the PID
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }
    

    /**
     * Resets the controller's saved info ab out integral and derivative.
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

    protected double calculateP(double error) {
        return kP * error;
    }
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

        //System.out.println(kI*m_accum);

        return kI * m_accum;
    }
    protected double calculateD(double error) {

        long curTime = RobotControllerWrapper.getInstance().getFPGATime();

        double ret = kD * (error - m_prevError) 
                / ((curTime - m_lastLoopTime) / 1e6);

        m_prevError = error;
        m_lastLoopTime = curTime;

        return ret;

    }
    protected double calculateF(double setpoint) {
        return kF * setpoint;
    }
}