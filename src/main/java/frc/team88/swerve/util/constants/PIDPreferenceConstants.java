package frc.team88.swerve.util.constants;

import java.util.Objects;

/**
 * Collection of PreferenceConstants for gains and values that are typically
 * used by PID. A particular PID implementation may choose to ignore some of
 * these values.
 */
public class PIDPreferenceConstants {

    private DoublePreferenceConstant kP;
    private DoublePreferenceConstant kI;
    private DoublePreferenceConstant kD;
    private DoublePreferenceConstant kF;
    private DoublePreferenceConstant iZone;
    private DoublePreferenceConstant iMax;
    private DoublePreferenceConstant tolerance;

    /**
     * Constructor. All values given are defaults that will be overriden if a
     * preference for them already exists.
     * @param name The name to pre-pend to all preference names
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The differential gain
     * @param kF The feedforward gain
     * @param iZone The error range in which the integral accumulates
     * @param iMax The max error absolute value that the integral will 
     * acumulate
     * @param tolerance The minimim error absolute value where an output will 
     * be applied
     */
    public PIDPreferenceConstants(String name, double kP, double kI, 
            double kD, double kF, double iZone, double iMax, 
            double tolerance) {
        Objects.requireNonNull(name);
        this.kP = new DoublePreferenceConstant(name + " kP", kP);
        this.kI = new DoublePreferenceConstant(name + " kI", kI);
        this.kD = new DoublePreferenceConstant(name + " kD", kD);
        this.kF = new DoublePreferenceConstant(name + " kF", kF);
        this.iZone = new DoublePreferenceConstant(name + " iZone", iZone);
        this.iMax = new DoublePreferenceConstant(name + " iMax", iMax);
        this.tolerance = new DoublePreferenceConstant(name + " tolerance", 
                tolerance);
    }

    /**
     * Updates all of the PID's preference constants
     */
    public void updateAll() {
        this.kP.update();
        this.kI.update();
        this.kD.update();
        this.kF.update();
        this.iZone.update();
        this.iMax.update();
        this.tolerance.update();
    }

    /**
     * Get the kP preference constant.
     * @return The kP preference constant
     */
    public DoublePreferenceConstant getKP() {
        return kP;
    }

    /**
     * Get the kI preference constant.
     * @return The kI preference constant
     */
    public DoublePreferenceConstant getKI() {
        return kI;
    }

    /**
     * Get the kD preference constant.
     * @return The kD preference constant
     */
    public DoublePreferenceConstant getKD() {
        return kD;
    }

    /**
     * Get the kF preference constant.
     * @return The kF preference constant
     */
    public DoublePreferenceConstant getKF() {
        return kF;
    }

    /**
     * Get the iZone preference constant.
     * @return The iZone preference constant
     */
    public DoublePreferenceConstant getIZone() {
        return iZone;
    }


    /**
     * Get the iMax preference constant.
     * @return The iMax preference constant
     */
    public DoublePreferenceConstant getIMax() {
        return iMax;
    }

    /**
     * Get the tolerance preference constant.
     * @return The tolerance preference constant
     */
    public DoublePreferenceConstant getTolerance() {
        return tolerance;
    }

}