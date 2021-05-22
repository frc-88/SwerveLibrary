package frc.team88.swerve.tuning;

import java.util.Objects;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.module.SwerveModule;
import frc.team88.swerve.util.WrappedAngle;

/**
 * Tuning mode for setting the wheel speed and azimuth position of each module.
 * Uses the appropriate control loops from the module.
 */
public class ModuleControlMode implements TuningMode {

    private final SwerveModule[] modules;

    private final double[] azimuthPositions;
    private final double[] wheelSpeeds;

    private boolean firstRun = true;

    /**
     * Constructs a modules control mode object.
     * 
     * @param modules The modules to be controlled.
     */
    public ModuleControlMode(SwerveModule[] modules) {
        this.modules = Objects.requireNonNull(modules);

        azimuthPositions = new double[modules.length];
        wheelSpeeds = new double[modules.length];
    }

    @Override
    public void init() {
        this.firstRun = true;
    }

    @Override
    public void run() {
        for (int idx = 0; idx < this.modules.length; idx++) {
            this.modules[idx].set(wheelSpeeds[idx], new WrappedAngle(azimuthPositions[idx]));
        }
    }

    @Override
    public void populateNetworkTable(NetworkTable table) {
        if (this.firstRun) {
            for (int idx = 0; idx < this.modules.length; idx++) {
                table.getEntry("modules/" + idx + "/azimuthPosition").setDouble(0);
                table.getEntry("modules/" + idx + "/wheelSpeed").setDouble(0);
            }
            this.firstRun = false;
        }
        for (int idx = 0; idx < this.modules.length; idx++) {
            azimuthPositions[idx] = table.getEntry("modules/" + idx + "/azimuthPosition").getDouble(azimuthPositions[idx]);
            wheelSpeeds[idx] = table.getEntry("modules/" + idx + "/wheelSpeed").getDouble(wheelSpeeds[idx]);
        }
    }
}
