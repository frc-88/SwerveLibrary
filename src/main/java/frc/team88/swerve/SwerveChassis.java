package frc.team88.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import frc.team88.swerve.kinematics.InverseKinematics;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.LongPreferenceConstant;
import frc.team88.swerve.wrappers.RobotControllerWrapper;
import frc.team88.swerve.wrappers.gyro.Gyro;

/**
 * Represents a complete swerve chassis, with high level operations for
 * controlling it.
 */
public class SwerveChassis {

    // Preference constants for the translation acceleration limit, in
    // feet per second^2.
    private DoublePreferenceConstant translationAccelerationLimit;

    // Preference constant for the rotation acceleration limit, in
    // rotations per second^2.
    private DoublePreferenceConstant rotationAccelerationLimit;

    // The angle to offset in hammer mode.
    private DoublePreferenceConstant hammerModeAngle;

    // The time to spend hammering in each direction
    private LongPreferenceConstant hammerModeTime;

    // The swerve modules on this chassis.
    private List<SwerveModule> modules;

    // The gyro for this chassis.
    private Gyro gyro;

    // The inverse kinematics controller for this chassis.
    private InverseKinematics inverseKinematics;

    // The target translation velocity, as a velocity vector in feet per
    // second.
    private Vector2D translationVelocity = Vector2D.ORIGIN;

    // The target rotation velocity, in degrees per second.
    private double rotationVelocity = 0;

    // True if in field-centric mode, false if in robot-centric mode.
    private boolean inFieldCentric = true;

    // True if in hammer mode, false otherwise.
    private boolean inHammerMode = false;

    // The expected rate at which update will be called, in Hz.
    private double expectedUpdateRate = 50.;

    // The time when hammer mode direction was last changed, in microseconds.
    private long lastHammerModeChangeTime;

    // If count of how many times hammer mode has changed direction.
    private int hammerModeChangeCount;

    /**
     * Construct.
     * 
     * @param gyro
     *                    The gyro measuring this chassis's field-oriented angle.
     * @param modules
     *                    The modules on this chassis. Minimum 2.
     */
    public SwerveChassis(Gyro gyro, SwerveModule... modules) {
        this.gyro = Objects.requireNonNull(gyro);

        if (modules.length < 2) {
            throw new IllegalArgumentException("A swerve chassis must have at least 2 swerve modules");
        }
        for (SwerveModule module : modules) {
            Objects.requireNonNull(module);
        }
        this.modules = Arrays.asList(modules);

        this.inverseKinematics = new InverseKinematics(modules);

        translationAccelerationLimit = new DoublePreferenceConstant("Translation Accel Limit", 100);
        rotationAccelerationLimit = new DoublePreferenceConstant("Rotation Accel Limit", 1080);
        hammerModeAngle = new DoublePreferenceConstant("Hammer Mode Angle", 70);
        hammerModeTime = new LongPreferenceConstant("Hammer Mode Time", 1_000_000);
    }

    /**
     * Updates all periodic processes in the swerve chassis, such as setting module
     * controls.
     */
    public void update() {
        Vector2D adjustedTranslation = this.translationVelocity;

        // Apply field-centric to translation if necessary
        if (this.inFieldCentric()) {
            adjustedTranslation = adjustedTranslation.rotate(-gyro.getYaw());
        }

        // Apply hammer mode if necessary
        if (inHammerMode()) {
            // Halve the time to change if this is the first hammer
            long minTimeToChange = hammerModeTime.getValue();
            if (hammerModeChangeCount == 0) {
                minTimeToChange /= 2;
            }
            // Check if it is time to change

            if ((RobotControllerWrapper.getInstance().getFPGATime() - lastHammerModeChangeTime) > minTimeToChange) {
                hammerModeChangeCount++;
                lastHammerModeChangeTime = RobotControllerWrapper.getInstance().getFPGATime();
            }
            // Rotate the vector by the offset in the right direction
            double angleToRotateVector = hammerModeAngle.getValue();
            if (hammerModeChangeCount % 2 == 1) {
                angleToRotateVector *= -1;
            }
            adjustedTranslation = adjustedTranslation.rotate(angleToRotateVector);
        }
        
        // Calculate the change limit for translation
        double translationChangeLimit = this.translationAccelerationLimit.getValue() / this.expectedUpdateRate;
        // Calculate the acceleration limited translation
        Vector2D changeLimitedTranslation = this.inverseKinematics.getTranslationVelocity()
                .limitChange(adjustedTranslation, translationChangeLimit);
        // Apply the translation velocity
        this.inverseKinematics.setTranslationVelocity(changeLimitedTranslation);

        // Calculate the change limit for rotation
        double rotationChangeLimit = this.rotationAccelerationLimit.getValue() / this.expectedUpdateRate;
        // Calculate the acceleration limited rotation
        double changeLimitedRotation = MathUtils.limitChange(inverseKinematics.getRotationVelocity(),
                this.rotationVelocity, rotationChangeLimit);
        // Apply the rotation velocity
        this.inverseKinematics.setRotationVelocity(changeLimitedRotation);

        // Update the inverse kinematics
        this.inverseKinematics.update();
    }

    /**
     * Set the rate at which update should be called, in seconds. Used to calculate
     * the acceleration limit.
     * 
     * @param rate
     *                 The update rate, in Hz
     */
    public void setExpectedUpdateRate(double rate) {
        if (rate <= 0) {
            throw new IllegalArgumentException("Update rate must be positive");
        }
        this.expectedUpdateRate = rate;
    }

    /**
     * Sets the translation velocity to target. Whether this is robot-centric or
     * field-centric depends on which has been set.
     * 
     * @param velocity
     *                     A velocity vector for chassis translation, in feet per
     *                     second
     */
    public void setTranslationVelocity(Vector2D velocity) {
        this.translationVelocity = Objects.requireNonNull(velocity);
    }

    /**
     * Gets the targetted translation velocity. Whether this is robot-centric or
     * field-centric depends on which has been set.
     * 
     * @return A velocity vector for chassis translation, in feet per second
     */
    public Vector2D getTranslationVelocity() {
        return this.translationVelocity;
    }

    /**
     * Sets the rotation velocity to target.
     * 
     * @param velocity
     *                     The rotation velocity, in degrees per second
     */
    public void setRotationVelocity(double velocity) {
        this.rotationVelocity = velocity;
    }

    /**
     * Gets the targetted rotation velocity.
     * 
     * @return The rotation velocity, in degrees per second
     */
    public double getRotationVelocity() {
        return this.rotationVelocity;
    }

    /**
     * Sets the center of rotation. Always robot-centric.
     * 
     * @param center
     *                   A position vector describing the center of rotation
     *                   relative to the chassis's origin
     */
    public void setCenterOfRotation(Vector2D center) {
        this.inverseKinematics.setCenterOfRotation(center);
    }

    /**
     * Gets the center of rotation. Always robot-centric.
     * 
     * @return A position vector describing the center of rotation relative to the
     *         chassis's origin
     */
    public Vector2D getCenterOfRotation() {
        return this.inverseKinematics.getCenterOfRotation();
    }

    /**
     * Sets the max wheel speed.
     * 
     * @param speed
     *                  The maximum wheel speed, in feet per second
     */
    public void setMaxWheelSpeed(double speed) {
        this.inverseKinematics.setMaxSpeed(speed);
    }

    /**
     * Gets the max wheel speed.
     * 
     * @return The maximum wheel speed, in feet per second
     */
    public double getMaxWheelSpeed() {
        return this.inverseKinematics.getMaxSpeed();
    }

    /**
     * Enables field-centric mode, which means that all given translation angles are
     * relative to gyro 0.
     */
    public void setFieldCentic() {
        this.inFieldCentric = true;
    }

    /**
     * Enables robot-centric mode, which means that all given translation angles are
     * relative to the front of the robot.
     */
    public void setRobotCentic() {
        this.inFieldCentric = false;
    }

    /**
     * Is this chassis in field-centric mode?
     * 
     * @return True if in field-centric mode, false if in robot-centric mode
     */
    public boolean inFieldCentric() {
        return this.inFieldCentric;
    }

    /**
     * Enables hammer mode.
     */
    public void enableHammerMode() {
        if (!inHammerMode()) {
            this.inHammerMode = true;
            lastHammerModeChangeTime = RobotControllerWrapper.getInstance().getFPGATime();
            hammerModeChangeCount = 0;
        }
    }

    /*
     * Disables hammer mode.
     */
    public void disableHammerMode() {
        this.inHammerMode = false;
    }

    /**
     * Is this chassis in hammer mode?
     * 
     * @return True if in hammer mode, false otherwise
     */
    public boolean inHammerMode() {
        return this.inHammerMode;
    }
}
