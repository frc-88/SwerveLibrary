package frc.team88.swerve.data;

import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;

import frc.team88.swerve.configuration.Configuration;
import frc.team88.swerve.motion.state.VelocityState;
import frc.team88.swerve.commandmux.CommandMux;
import frc.team88.swerve.commandmux.CommandMuxEntry;

import java.util.Objects;

public class NetworkTableCommandListener implements TableEntryListener {
    // The config for this swerve drive.
    private Configuration config;
    private int muxId = -1;
    private VelocityState currentState;
    private NetworkTable m_table;
    private CommandMux commandMux;

    public NetworkTableCommandListener(final Configuration config) {
        this.config = config;
        this.currentState = new VelocityState(0.0, 0.0, 0.0, false);
        this.commandMux = this.config.getCommandMux();
        findNtMux();
    }

    public int getMuxId() {
        return this.muxId;
    }

    public VelocityState getCommand() {
        return currentState;
    }

    public void setTable(NetworkTable table) {
        m_table = table;
        m_table.getEntry("timestamp").setDouble(0.0);
        m_table.getEntry("translationDirection").setDouble(0.0);
        m_table.getEntry("translationSpeed").setDouble(0.0);
        m_table.getEntry("rotationVelocity").setDouble(0.0);
        m_table.getEntry("isFieldCentric").setBoolean(false);
    }

    private void findNtMux()
    {
        for (int index = 0; index < this.commandMux.getNumEntries(); index++) {
            CommandMuxEntry entry = this.commandMux.getEntry(index);
            if (entry.getTemplateName().equals("networktables")) {
                muxId = entry.getID();
                return;
            }
        }
    }

    @Override
    public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags)
    {
        if (muxId != -1) {
            this.config.getCommandMux().activate(muxId);
        }

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
