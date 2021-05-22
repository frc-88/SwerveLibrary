package frc.team88.swerve.tuning;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Optional;

/** Manages the running of the available tuning modes. */
public class TuningManager implements NetworkTablePopulator {

  private final Map<String, TuningMode> modes;

  private Optional<String> activeMode;
  private boolean newActiveMode = false;

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
    modes.put("moduleControl", new ModuleControlMode(config.getModules()));

    this.activeMode = Optional.empty();
  }

  /** Checks for changes in the currently active tuning mode, then runs it if there is one. */
  public void update() {
    // Disable all modes if robot is disabled.
    if (DriverStation.getInstance().isDisabled()) {
      this.activeMode = Optional.empty();
      this.newActiveMode = false;
      return;
    }

    if (activeMode.isPresent()) {
      if (newActiveMode) {
        this.modes.get(activeMode.get()).init();
        this.newActiveMode = false;
      }
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
        NetworkTableEntry enabledEntry = table.getSubTable(entry.getKey()).getEntry("enable");
        String mode = entry.getKey();

        // Check if this is the active mode and was disabled.
        if (this.isActiveMode(mode) && !enabledEntry.getBoolean(false)) {
          this.activeMode = Optional.empty();
        }

        // Queue the mode if set to enabled.
        if (!this.isActiveMode(mode)
            && enabledEntry.getBoolean(false)
            && DriverStation.getInstance().isEnabled()) {
          if (this.activeMode.isPresent()) {
            table.getSubTable(this.activeMode.get()).getEntry("enable").setBoolean(false);
            this.activeMode = Optional.empty();
          }
          this.activeMode = Optional.of(mode);
          this.newActiveMode = true;
        }

        // Only the enabled mode should show as such.
        enabledEntry.setBoolean(this.isActiveMode(mode));

        entry.getValue().populateNetworkTable(table.getSubTable(mode));
      }
    }
  }

  /**
   * Determines if the given mode is the active mode.
   *
   * @param mode The mode to check.
   * @return True if the given mode is active, false if another mode is active or if no modes are
   *     active.
   */
  private boolean isActiveMode(String mode) {
    return this.activeMode.isPresent() && this.activeMode.get().equals(mode);
  }
}
