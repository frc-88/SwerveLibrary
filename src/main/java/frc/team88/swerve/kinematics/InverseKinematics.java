package frc.team88.swerve.kinematics;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Handles the mathematical processing of inverse kinematics for a swerve drive.
 * Manages the modules it is controlling. All calculations are robot-centric.
 */
public class InverseKinematics {

    // The target translation velocity, as a velocity vector in feet per
    // second.
    private Vector2D translationVelocity = Vector2D.ORIGIN;

    // The target rotation velocity, in degrees per second.
    private double rotationVelocity = 0;

    // The center of rotation, as a position vector from the robot's origin,
    // in feet.
    private Vector2D centerOfRotation = Vector2D.ORIGIN;

    // The maximum speed to set to a module.
    private double maxModuleSpeed = Double.POSITIVE_INFINITY;

    // The modules being controlled.
    private List<SwerveModule> modules;

    // The last azimuth target for each module that wasn't accompanied by 0 
    // wheel speed.
    private List<WrappedAngle> lastGoodAzimuths;

    /**
     * Constructor. Just does some small initialization work.
     */
    public InverseKinematics() {
        modules = new LinkedList<>();
        lastGoodAzimuths = new ArrayList<>();
    }

    /**
     * Registers the given swerve module to be controlled by inverse kinematics.
     */
    public void addSwerveModule(SwerveModule module) {
        this.modules.add(module);
        this.lastGoodAzimuths.add(new WrappedAngle(0));
    }

    /**
     * Sets the target translation velocity.
     * 
     * @param velocity
     *                     A velocity vector representing the translation velocity,
     *                     in feet per second
     */
    public void setTranslationVelocity(Vector2D velocity) {
        this.translationVelocity = Objects.requireNonNull(velocity);
    }

    /**
     * Gets the target translation velocity
     * 
     * @return A velocity vector representing the translation velocity, in feet per
     *         second
     */
    public Vector2D getTranslationVelocity() {
        return this.translationVelocity;
    }

    /**
     * Sets the target rotation velocity.
     * 
     * @param velocity
     *                     The rotation velocity, in feet per second
     */
    public void setRotationVelocity(double velocity) {
        this.rotationVelocity = velocity;
    }

    /**
     * Gets the target rotation velocity
     * 
     * @return The rotation velocity, in degrees per second
     */
    public double getRotationVelocity() {
        return this.rotationVelocity;
    }

    /**
     * Sets the center of rotation.
     * 
     * @param center
     *                   A position vector from the origin of the robot representing
     *                   the center of rotation, in feet
     */
    public void setCenterOfRotation(Vector2D center) {
        this.centerOfRotation = Objects.requireNonNull(center);
    }

    /**
     * Gets the center of rotation.
     * 
     * @return A position vector from the origin of the robot representing the
     *         center of rotation, in feet
     */
    public Vector2D getCenterOfRotation() {
        return this.centerOfRotation;
    }

    /**
     * Sets the maximum speed to a module wheel.
     * 
     * @param speed
     *                  The maximum speed in feet per second
     */
    public void setMaxSpeed(double speed) {
        if (speed < 0) {
            throw new IllegalArgumentException("Speed must be greater than 0");
        }
        this.maxModuleSpeed = speed;
    }

    /**
     * Sets the maximum speed to a module wheel.
     * 
     * @return The maximum speed in feet per second
     */
    public double getMaxSpeed() {
        return this.getMaxSpeed();
    }

    /**
     * Updates the modules to their correct positions. Should be called every
     * schedule tick even if nothing has changed to update control loops. Note
     * that if a module has a speed of exactly 0, the azimuth target will stay
     * at it's previous value.
     */
    public void update() {
        // Get the translation and rotation vectors
        Vector2D[] translationVectors = new Vector2D[modules.size()];
        Vector2D[] rotationVectors = new Vector2D[modules.size()];
        for (int idx = 0; idx < modules.size(); ++idx) {
            translationVectors[idx] = calculateModuleTranslationVector();
            rotationVectors[idx] = calculateModuleRotationVectors(modules.get(idx));
        }

        // Add the vectors
        Vector2D[] unscaledVectors = new Vector2D[modules.size()];
        for (int idx = 0; idx < modules.size(); ++idx) {
            unscaledVectors[idx] = translationVectors[idx].plus(rotationVectors[idx]);
        }

        // Scale down the vectors
        Vector2D[] moduleVectors = scaleIntoRange(unscaledVectors);

        // Apply to modules
        for (int idx = 0; idx < modules.size(); ++idx) {
            modules.get(idx).setWheelSpeed(moduleVectors[idx].getMagnitude());
            if(moduleVectors[idx].getMagnitude() == 0.) {
                modules.get(idx).setAzimuthPosition(lastGoodAzimuths.get(idx));
            } else {
                modules.get(idx).setAzimuthPosition(moduleVectors[idx].getAngle());
                lastGoodAzimuths.set(idx, moduleVectors[idx].getAngle());
            }
        }
    }

    /**
     * Calculates the velocity vector for any module corresponding to the
     * translation component of motion.
     * 
     * @return The velocity vector in feet per second
     */
    protected Vector2D calculateModuleTranslationVector() {
        // The translation vector for each module is the same as the overall
        // translation vector
        return this.translationVelocity;
    }

    /**
     * Calculates the velocity vector for the given module corresponding to the
     * rotation component of motion.
     * 
     * @param module
     *                   The swerve module to calculate for
     * @return The velocity vector in feet per second
     */
    protected Vector2D calculateModuleRotationVectors(SwerveModule module) {
        // Get the location of the module relative to the robot's origin
        Vector2D location = module.getLocation();
        // Calculate the location relative to the center of rotation.
        // (location - centerOfRotation)
        Vector2D positionFromRotationCenter = location.plus(this.centerOfRotation.times(-1));
        // The direction to move is perpendicular to the position vector
        Vector2D unscaled = positionFromRotationCenter.rotate(90);
        // For every full rotation of the robot, each wheel needs to drive
        // once around the circumference of the circle defined by the center
        // of rotation at it's center and the location of the module on the
        // circle. Therefore, the velocity for w rotations per second is w
        // times the circumference of the circle, and 1/360th of that for
        // w degrees per second.
        double circumference = positionFromRotationCenter.getMagnitude() * 2. * Math.PI;
        return Vector2D.createPolarCoordinates(this.rotationVelocity * circumference / 360., unscaled.getAngle());
    }

    /**
     * Scales down the velocities linearly so that none of them have a magnitude
     * greater than the max speed.
     * 
     * @param velocities
     *                       The velocities to scale, in feet
     * @return The scaled velocities
     */
    protected Vector2D[] scaleIntoRange(Vector2D[] velocities) {
        int largestVelocityIdx = 0;
        for (int idx = 1; idx < velocities.length; ++idx) {
            if (velocities[idx].getMagnitude() > velocities[largestVelocityIdx].getMagnitude()) {
                largestVelocityIdx = idx;
            }
        }

        Vector2D[] ret = new Vector2D[velocities.length];
        double reduction = velocities[largestVelocityIdx].getMagnitude() / this.maxModuleSpeed;
        if (reduction > 1) {
            for (int idx = 0; idx < ret.length; ++idx) {
                ret[idx] = Vector2D.createPolarCoordinates(velocities[idx].getMagnitude() / reduction,
                        velocities[idx].getAngle());
            }
        } else {
            for (int idx = 0; idx < ret.length; ++idx) {
                ret[idx] = velocities[idx];
            }
        }
        return ret;
    }
}