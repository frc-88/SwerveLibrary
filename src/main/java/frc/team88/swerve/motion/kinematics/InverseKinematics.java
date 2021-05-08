package frc.team88.swerve.motion.kinematics;

import java.util.Objects;

import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.modifiers.MotionModifier;
import frc.team88.swerve.motion.state.FullVelocityMotionState;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.util.Vector2D;
import frc.team88.swerve.util.WrappedAngle;
import frc.team88.swerve.util.logging.DataLogger;

/**
 * Handles the mathematical processing of inverse kinematics for a swerve drive.
 * Manages the modules it is controlling. All calculations are robot-centric.
 */
public class InverseKinematics implements MotionModifier<Void> {

    // The robot-centric target state for the chassis's motion.
    private MotionState targetState;

    // The maximum speed to set to a module.
    private double maxModuleSpeed = Double.POSITIVE_INFINITY;

    // The modules being controlled.
    private SwerveModule[] modules;

    // The last azimuth target for each module that wasn't accompanied by 0
    // wheel speed.
    private WrappedAngle[] lastGoodAzimuths;

    /**
     * Constructor.
     * 
     * @param modules
     *                    The modules to be controlled. Minimumn 2.
     */
    public InverseKinematics(SwerveModule... modules) {
        if (modules.length < 2) {
            throw new IllegalArgumentException("Cannot do inverse kinematics with less than 2 modules");
        }
        this.modules = modules;

        this.targetState = FullVelocityMotionState.createRobotCentricDefault();

        this.lastGoodAzimuths = new WrappedAngle[modules.length];
        for (int idx = 0; idx < modules.length; ++idx) {
            Objects.requireNonNull(modules[idx]);
            lastGoodAzimuths[idx] = new WrappedAngle(0);
        }
    }

    /**
     * Sets the target motion state.
     * 
     * @param target
     *                   The motion state to target. Robot-centric
     */
    public void setTargetState(MotionState target) {
        Objects.requireNonNull(target);
        if (target.isFieldCentric()) {
            throw new IllegalArgumentException("Cannot give field centric motion state to inverse kinematics");
        }
        this.targetState = target;
    }

    /**
     * Gets the target motion state.
     * 
     * @return The currently set target motion state. Robot-centric
     */
    public MotionState getTargetState() {
        return this.targetState;
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
     * schedule tick even if nothing has changed to update control loops. Note that
     * if a module has a speed of exactly 0, the azimuth target will stay at it's
     * previous value.
     */
    public void update() {
        this.apply(this.getTargetState());
    }

    @Override
    public Void apply(MotionState state) {
        return state.acceptModifier(this);
    }

    @Override
    public Void applyToFullVelocityState(FullVelocityMotionState state) {
        // Get the translation and rotation vectors
        Vector2D[] translationVectors = new Vector2D[modules.length];
        Vector2D[] rotationVectors = new Vector2D[modules.length];
        for (int idx = 0; idx < modules.length; ++idx) {
            translationVectors[idx] = calculateModuleTranslationVector(state);
            rotationVectors[idx] = calculateModuleRotationVectors(state, modules[idx]);
        }

        // Add the vectors
        Vector2D[] unscaledVectors = new Vector2D[modules.length];
        for (int idx = 0; idx < modules.length; ++idx) {
            unscaledVectors[idx] = translationVectors[idx].plus(rotationVectors[idx]);
        }

        // Scale down the vectors
        Vector2D[] moduleVectors = scaleIntoRange(unscaledVectors);

        // Apply to modules
        for (int idx = 0; idx < modules.length; ++idx) {
            // Log module data
            class ModuleData {
                double targetWheelSpeed;
                double targetAzimuthAngle;
                double actualWheelSpeed;
                double actualAzimuthAngle;
                double actualAzimuthSpeed;
                double xPos;
                double yPos;

                public ModuleData(double targetWheelSpeed, double targetAzimuthAngle, SwerveModule module) {
                    this.targetWheelSpeed = targetWheelSpeed;
                    this.targetAzimuthAngle = targetAzimuthAngle;
                    this.actualWheelSpeed = module.getWheelSpeed();
                    this.actualAzimuthAngle = module.getAzimuthPosition().asDouble();
                    this.actualAzimuthSpeed = module.getAzimuthVelocity();
                    this.xPos = module.getLocation().getX();
                    this.yPos = module.getLocation().getY();
                }
            }
            DataLogger.getInstance().addData("Module " + idx, new ModuleData(moduleVectors[idx].getMagnitude(),
                    moduleVectors[idx].getAngle().asDouble(), this.modules[idx]));

            modules[idx].setWheelSpeed(moduleVectors[idx].getMagnitude());
            if (moduleVectors[idx].getMagnitude() == 0.) {
                modules[idx].setAzimuthPosition(lastGoodAzimuths[idx]);
            } else {
                modules[idx].setAzimuthPosition(moduleVectors[idx].getAngle());
                lastGoodAzimuths[idx] = moduleVectors[idx].getAngle();
            }
        }

        // Null required because generic return type is Void, not void
        return null;
    }

    /**
     * Calculates the velocity vector for any module corresponding to the
     * translation component of motion.
     * 
     * @param state
     *                  The target motion state that specifies velocities
     * @return The velocity vector in feet per second
     */
    protected Vector2D calculateModuleTranslationVector(FullVelocityMotionState state) {
        // The translation vector for each module is the same as the overall
        // translation vector
        return state.getTranslationVelocity();
    }

    /**
     * Calculates the velocity vector for the given module corresponding to the
     * rotation component of motion.
     * 
     * @param state
     *                   The target motion state that specifies velocities
     * @param module
     *                   The swerve module to calculate for
     * @return The velocity vector in feet per second
     */
    protected Vector2D calculateModuleRotationVectors(FullVelocityMotionState state, SwerveModule module) {
        // Get the location of the module relative to the robot's origin
        Vector2D location = module.getLocation();
        // Calculate the location relative to the center of rotation.
        // (location - centerOfRotation)
        Vector2D positionFromRotationCenter = location.plus(state.getCenterOfRotation().times(-1));
        // The direction to move is perpendicular to the position vector
        Vector2D unscaled = positionFromRotationCenter.rotate(90);
        // For every full rotation of the robot, each wheel needs to drive
        // once around the circumference of the circle defined by the center
        // of rotation at it's center and the location of the module on the
        // circle. Therefore, the velocity for w rotations per second is w
        // times the circumference of the circle, and 1/360th of that for
        // w degrees per second.
        double circumference = positionFromRotationCenter.getMagnitude() * 2. * Math.PI;
        return unscaled.changeMagnitude(state.getRotationVelocity() * circumference / 360.);
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
                ret[idx] = velocities[idx].changeMagnitude(velocities[idx].getMagnitude() / reduction);
            }
        } else {
            for (int idx = 0; idx < ret.length; ++idx) {
                ret[idx] = velocities[idx];
            }
        }
        return ret;
    }
}