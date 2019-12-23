package frc.team88.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import frc.team88.swerve.kinematics.InverseKinematics;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.wrappers.gyro.Gyro;

/**
 * Represents a complete swerve chassis, with high level operations for
 * controlling it.
 */
public class SwerveChassis {

    // The swerve modules on this chassis.
    private List<SwerveModule> modules;

    // The gyro for this chassis.
    private Gyro gyro;

    // The inverse kinematics controller for this chassis.
    private InverseKinematics inverseKinematics;

    // The target translation velocity, as a velocity vector in feet per
    // second. Sperate from inverseKinematics to apply field-centricity.
    private Vector2D translationVelocity = Vector2D.ORIGIN;

    // True if in field-centric mode, false if in robot-centric mode
    private boolean inFieldCentric = true;

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
    }

    /**
     * Updates all periodic processes in the swerve chassis, such as
     * setting module controls.
     */
    public void update() {
        // Apply field-centric if necessary
        if (this.inFieldCentric()) {
            this.inverseKinematics.setTranslationVelocity(this.translationVelocity.rotate(-gyro.getYaw()));
        } else {
            this.inverseKinematics.setTranslationVelocity(this.translationVelocity);
        }

        // Update the inverse kinematics
        this.inverseKinematics.update();
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
        this.inverseKinematics.setRotationVelocity(velocity);
    }

    /**
     * Gets the targetted rotation velocity.
     * 
     * @return The rotation velocity, in degrees per second
     */
    public double getRotationVelocity() {
        return this.inverseKinematics.getRotationVelocity();
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
     * @param speed The maximum wheel speed, in feet per second
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

}
