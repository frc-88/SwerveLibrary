package frc.team88.swerve;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import edu.wpi.first.wpilibj.RobotController;
import frc.team88.swerve.kinematics.ForwardKinematics;
import frc.team88.swerve.kinematics.InverseKinematics;
import frc.team88.swerve.motion.modifiers.LimitAcceleration;
import frc.team88.swerve.motion.modifiers.ToHammerMode;
import frc.team88.swerve.motion.modifiers.ToRobotCentric;
import frc.team88.swerve.motion.state.MotionState;
import frc.team88.swerve.motion.state.OdomState;
import frc.team88.swerve.swervemodule.SwerveModule;
import frc.team88.swerve.swervemodule.SwerveModule.SwitchingMode;
import frc.team88.swerve.util.constants.DoublePreferenceConstant;
import frc.team88.swerve.util.constants.LongPreferenceConstant;
import frc.team88.swerve.util.logging.DataLogger;
import frc.team88.swerve.wrappers.gyro.Gyro;

/**
 * Represents a complete swerve chassis, with high level operations for
 * controlling it.
 */
public class SwerveChassis {

    // The gyro to use for this chassis.
    private Gyro gyro;

    // The swerve modules on this chassis.
    private List<SwerveModule> modules;

    // The unmodified commanded target state
    private MotionState targetState;

    // The inverse kinematics controller for this chassis.
    private InverseKinematics inverseKinematics;

    // The forward kinematics controller for this chassis.
    private ForwardKinematics forwardKinematics;

    // True if in field-centric mode, false if in robot-centric mode.
    private boolean inFieldCentric = true;

    // True if in hammer mode, false otherwise
    private boolean inHammerMode = false;

    // Modifier for field-centric mode
    private ToRobotCentric fieldCentricModifier;

    // Modifier for hammer mode
    private ToHammerMode hammerModeModifier;

    // Modifier for acceleration limiting
    private LimitAcceleration accelerationLimitModifier;

    // Timers for computing update rate
    private double currentTime;
    private double prevTime;

    /**
     * Construct.
     * 
     * @param gyro
     *                               The gyro measuring this chassis's
     *                               field-oriented angle.
     * @param modules
     *                               The modules on this chassis. Minimum 2.
     * @param expectedUpdateRate
     *                               The expected rate at which update will be
     *                               called, in Hz.
     */
    public SwerveChassis(Gyro gyro, double expectedUpdateRate, SwerveModule... modules) {
        Objects.requireNonNull(gyro);
        this.gyro = gyro;

        if (modules.length < 2) {
            throw new IllegalArgumentException("A swerve chassis must have at least 2 swerve modules");
        }
        for (SwerveModule module : modules) {
            Objects.requireNonNull(module);
            module.setSwitchingMode(SwitchingMode.kSmart);
        }
        this.modules = Arrays.asList(modules);

        this.inverseKinematics = new InverseKinematics(modules);
        this.forwardKinematics = new ForwardKinematics(modules);

        currentTime = 0.0;
        prevTime = 0.0;

        this.targetState = inverseKinematics.getTargetState();

        fieldCentricModifier = new ToRobotCentric(gyro);
        hammerModeModifier = new ToHammerMode(new DoublePreferenceConstant("Hammer Mode Angle", 70),
                new LongPreferenceConstant("Hammer Mode Time", 1_000_000));
        accelerationLimitModifier = new LimitAcceleration(new DoublePreferenceConstant("Translation Accel Limit", 100),
                new DoublePreferenceConstant("Rotation Accel Limit", 1080), inverseKinematics::getTargetState,
                expectedUpdateRate);
    }

    /**
     * Updates all periodic processes in the swerve chassis, such as setting module
     * controls.
     */
    public void update() {
        // Log Gyro State
        class GyroInfo {
            double yaw;
            double yawRate;
            public GyroInfo(Gyro gyro) {
                this.yaw = gyro.getYaw();
                this.yawRate = gyro.getYawRate();
            }
        }
        DataLogger.getInstance().addData("Gyro Info", new GyroInfo(this.gyro));

        MotionState modifiedState = this.targetState;
        DataLogger.getInstance().addData("Target State", modifiedState);

        // Apply field-centric to translation if necessary
        if (this.inFieldCentric()) {
            modifiedState = fieldCentricModifier.apply(modifiedState);
        }

        // Apply hammer mode if necessary
        if (inHammerMode()) {
            modifiedState = hammerModeModifier.apply(modifiedState);
        }

        // Apply acceleration limits
        modifiedState = accelerationLimitModifier.apply(modifiedState);

        DataLogger.getInstance().addData("Modified State", modifiedState);

        // Set the target state
        this.inverseKinematics.setTargetState(modifiedState);

        // Update the inverse kinematics
        this.inverseKinematics.update();

        // Update the forward kinematics and compute current pose
        this.forwardKinematics.update(getDt());
    }

    private double getDt()
    {
        currentTime = RobotController.getFPGATime() * 1E-6;
        double dt = currentTime - prevTime;
        prevTime = currentTime;
        return dt;
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
     * Gets a module object
     * 
     * @param module_num The ID number for the module
     * 
     * @return A module object
     */
    public SwerveModule getModule(int module_num) {
        if (module_num >= modules.size() || module_num < 0) {
            throw new IllegalArgumentException(module_num + " exceeds module num bounds");
        }
        return modules.get(module_num);
    }

    /**
     * Gets the number of modules
     * 
     * @return Number of modules
     */
    public int getNumModules() {
        return modules.size();
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
    public void setOdomState(double x, double y) {
        OdomState state = new OdomState();
        state.x = x;
        state.y = y;
        this.setOdomState(state);
    }
    public void setOdomState(double x, double y, double theta) {
        OdomState state = new OdomState();
        state.x = x;
        state.y = y;
        state.t = theta;
        this.setOdomState(state);
    }

    /**
     * Sets the target motion state.
     * 
     * @param target
     *                   The motion state to set
     */
    public void setTargetState(MotionState target) {
        this.targetState = Objects.requireNonNull(target);
    }

    /**
     * Gets the target motion state.
     * 
     * @return The target motion state
     */
    public MotionState getTargetState() {
        return this.targetState;
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
            hammerModeModifier.reset();
            for (SwerveModule module : modules) {
                module.setSwitchingMode(SwitchingMode.kAlwaysSwitch);
            }
        }
    }

    /*
     * Disables hammer mode.
     */
    public void disableHammerMode() {
        if (inHammerMode()) {
            this.inHammerMode = false;
            for (SwerveModule module : modules) {
                module.setSwitchingMode(SwitchingMode.kSmart);
            }
        }
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
