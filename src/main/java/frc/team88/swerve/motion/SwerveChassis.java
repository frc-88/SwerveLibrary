package frc.team88.swerve.motion;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.gyro.SwerveGyro;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.motion.kinematics.ForwardKinematics;
import frc.team88.swerve.motion.kinematics.InverseKinematics;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.logging.DataLogger;

/**
 * Represents a complete swerve chassis, with high level operations for
 * controlling it.
 */
public class SwerveChassis {

    // The config for this swerve drive.
    private Configuration config;

    // The unmodified commanded target state.
    private VelocityState targetState = new VelocityState(0, 0, 0, false);

    // The inverse kinematics controller for this chassis.
    private InverseKinematics inverseKinematics;

    // The forward kinematics controller for this chassis.
    private ForwardKinematics forwardKinematics;

    // A mode for holding wheel azimuths and setting speed to 0.
    private boolean holdMode = true;;

    /**
     * Constructs the SwerveChassis from the config.
     * 
     * @param config The config info for this swerve.
     */
    public SwerveChassis(Configuration config) {
        this.config = Objects.requireNonNull(config);

        this.inverseKinematics = new InverseKinematics(this.config.getModules());
        this.forwardKinematics = new ForwardKinematics(this.config.getModules());
    }

    
    /**
     * Sets the target velocity state.
     * 
     * @param target The velocity state to set.
     */
    public void setTargetState(VelocityState target) {
        this.holdMode = false;
        this.targetState = Objects.requireNonNull(target);
    }

    /**
     * Gets the target velocity state.
     * 
     * @return The target velocity state.
     */
    public VelocityState getTargetState() {
        return this.targetState;
    }

    public void holdAzimuths() {
        this.holdMode = true;
    }

    /**
     * Updates all periodic processes in the swerve chassis, such as setting module
     * controls.
     */
    public void update() {
        // Update the forward kinematics and compute current pose
        this.forwardKinematics.update(getDt());

        if (this.holdMode) {
            return;
        }

        VelocityState velocityState = this.getTargetState();

        velocityState = this.makeRobotCentric(velocityState);

        // Set the target state
        this.inverseKinematics.setTargetState(velocityState);

        // Update the inverse kinematics
        this.inverseKinematics.update();
    }

    /**
     * Gets the chassis odometry state.
     * 
     * @return The odometry state
     */
    public OdomState getOdomState() {
        return this.forwardKinematics.getOdom();
    }

    /**
     * Sets the chassis odometry state.
     * (for setting chassis initial conditions)
     * 
     * @param state  state to set the chassis to
     */
    public void setOdomState(OdomState state) {
        this.forwardKinematics.setOdom(state);
    }

    /**
     * Sets the chassis odometry state.
     * (for setting chassis initial conditions)
     * 
     * @param x The x position of the chassis.
     * @param y The y position of the chassis.
     */
    public void setOdomState(double x, double y) {
        OdomState state = new OdomState();
        state.x = x;
        state.y = y;
        this.setOdomState(state);
    }

    /**
     * Sets the chassis odometry state.
     * (for setting chassis initial conditions)
     * 
     * @param x The x position of the chassis.
     * @param y The y position of the chassis.
     * @param theta The heading of the chasis.
     */
    public void setOdomState(double x, double y, double theta) {
        OdomState state = new OdomState();
        state.x = x;
        state.y = y;
        state.t = theta;
        this.setOdomState(state);
    }

    /**
     * Converts a velocity state to robot-centric using the gyro if it is field
     * centric, otherwise returns it unmodified.
     * 
     * @param state The state to make robot-centric.
     * @return A robot-centric state.
     */
    private VelocityState makeRobotCentric(VelocityState state) {
        if (!state.isFieldCentric()) {
            return state;
        }
        return state.changeTranslationDirection(state.getTranslationDirection() - this.config.getGyro().getYaw()).changeIsFieldCentric(false);
    }
}
