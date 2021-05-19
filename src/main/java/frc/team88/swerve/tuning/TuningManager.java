package frc.team88.swerve.tuning;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Map.Entry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.data.NetworkTablePopulator;

/**
 * Manages the running of the available tuning modes.
 */
public class TuningManager implements NetworkTablePopulator {

    private final Map<String, TuningMode> modes;

    private Optional<String> activeMode;
    private Optional<String> queuedActiveMode;

    private boolean firstNetworkTableCall = true;

    /**
     * Constructs a tuning manager.
     * 
     * @param config The config for this swerve drive.
     */
    public TuningManager(final Configuration config) {
        Objects.requireNonNull(config);

        modes = new HashMap<>();
        modes.put("motorControl", new MotorControlMode(config.getModules()));

        this.activeMode = Optional.empty();
        this.queuedActiveMode = Optional.empty();
    }

    /**
     * Checks for changes in the currently active tuning mode, then runs it if
     * there is one.
     */
    public void update() {
        // Disable all modes if robot is disabled.
        if (DriverStation.getInstance().isDisabled()) {
            this.activeMode = Optional.empty();
            this.queuedActiveMode = Optional.empty();
            return;
        }

        // Set the new mode if one is queued.
        if (queuedActiveMode.isPresent()) {
            this.activeMode = Optional.of(queuedActiveMode.get());
            this.queuedActiveMode = Optional.empty();

            this.modes.get(activeMode.get()).init();
        }

        if (activeMode.isPresent()) {
            this.modes.get(activeMode.get()).run();
        }
    }

    /**
     * Get if there is currently a tuning mode active.
     * 
     * @return True if a tuning mode is currently active, false otherwise.
     */
    public boolean isEnabled() {
        return this.activeMode.isPresent();
    }

    @Override
    public void populateNetworkTable(NetworkTable table) {
        if (this.firstNetworkTableCall) {
            // Populate table for first time
            for (Entry<String, TuningMode> entry : modes.entrySet()) {
                NetworkTable modeTable = table.getSubTable(entry.getKey());
                modeTable.getEntry("enable").setBoolean(false);
                entry.getValue().populateNetworkTable(modeTable);
            }
            this.firstNetworkTableCall = false;
        } else {
            for (Entry<String, TuningMode> entry : modes.entrySet()) {
                NetworkTable modeTable = table.getSubTable(entry.getKey());
                // Queue the mode if set to enabled.
                if (modeTable.getEntry("enable").getBoolean(false)) {
                    this.queuedActiveMode = Optional.of(entry.getKey());
                }
                // Only the enabled mode should show as such.
                if (!(activeMode.isPresent() && activeMode.get().equals(entry.getKey()))) {
                    modeTable.getEntry("enable").setBoolean(false);
                }
                entry.getValue().populateNetworkTable(modeTable);
            }
        }
    }

}
