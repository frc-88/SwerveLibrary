package frc.team88.swerve.tuning;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.team88.swerve.module.SwerveModule;
import java.util.Objects;

/**
 * Tuning mode for setting the velocity of individual motors. This is done with open loop control.
 */
public class MotorControlMode implements TuningMode {

  private final SwerveModule[] modules;

  private double[][] velocities;

  private boolean firstRun = true;

  /**
   * Constructs a motor control mode object.
   *
   * @param modules The modules whose motors will be controlled.
   */
  public MotorControlMode(SwerveModule[] modules) {
    this.modules = Objects.requireNonNull(modules);

    velocities = new double[modules.length][2];
  }

  @Override
  public void init() {
    this.firstRun = true;
  }

  @Override
  public void run() {
    for (int i = 0; i < velocities.length; i++) {
      for (int j = 0; j < velocities[i].length; j++) {
        modules[i].getMotors()[j].setVelocity(velocities[i][j]);
      }
    }
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    if (this.firstRun) {
      for (int i = 0; i < velocities.length; i++) {
        for (int j = 0; j < velocities[i].length; j++) {
          this.getVelocityEntry(table, i, j).setDouble(0);
        }
      }
      this.firstRun = false;
    }
    for (int i = 0; i < velocities.length; i++) {
      for (int j = 0; j < velocities[i].length; j++) {
        velocities[i][j] = this.getVelocityEntry(table, i, j).getDouble(velocities[i][j]);
      }
    }
  }

  /**
   * Get the NetworkTables entry for the given module and motor ID.
   *
   * @param table The base table for this mode.
   * @param module The module ID.
   * @param motor The motor ID.
   * @return The corresponding NetworkTables entry.
   */
  private NetworkTableEntry getVelocityEntry(NetworkTable table, int module, int motor) {
    return table.getEntry("modules/" + module + "/motors/" + motor + "/velocity");
  }
}
