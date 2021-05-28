package frc.team88.swerve.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.util.RobotControllerWrapper;

/** Handles callbacks for when the command subtable is updated */
public class NetworkTableCommandListener implements TableEntryListener {
  private VelocityState currentState;
  private NetworkTable m_table;
  private long commandTime = 0;

  public NetworkTableCommandListener() {
    this.currentState = new VelocityState(0.0, 0.0, 0.0, false);
  }

  /**
   * Gets the last command sent via network tables
   *
   * @return VelocityState
   */
  public VelocityState getCommand() {
    return currentState;
  }

  /**
   * Take a NetworkTable instance from DataManager and setup keys to listen to
   *
   * @param table A table from DataManager
   */
  public void setTable(NetworkTable table) {
    m_table = table;
    m_table.getEntry("timestamp").setDouble(0.0);
    m_table.getEntry("translationDirection").setDouble(0.0);
    m_table.getEntry("translationSpeed").setDouble(0.0);
    m_table.getEntry("rotationVelocity").setDouble(0.0);
    m_table.getEntry("isFieldCentric").setBoolean(false);
  }

  /**
   * Gets the time at which the last command was sent via network tables
   *
   * @return FPGA time in microseconds
   */
  public long getCommandTime() {
    return commandTime;
  }

  /** Callback for when data is pushed to the designated subtable */
  @Override
  public void valueChanged(
      NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
    commandTime = RobotControllerWrapper.getInstance().getFPGATime();
    double translationDirection = table.getEntry("translationDirection").getDouble(0.0);
    double translationSpeed = table.getEntry("translationSpeed").getDouble(0.0);
    double rotationVelocity = table.getEntry("rotationVelocity").getDouble(0.0);
    boolean isFieldCentric = table.getEntry("isFieldCentric").getBoolean(false);

    currentState =
        currentState
            .changeTranslationDirection(translationDirection)
            .changeTranslationSpeed(translationSpeed)
            .changeRotationVelocity(rotationVelocity)
            .changeIsFieldCentric(isFieldCentric);
  }
}
