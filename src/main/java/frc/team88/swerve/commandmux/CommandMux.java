package frc.team88.swerve.commandmux;


import com.electronwill.nightconfig.core.Config;
import java.util.List;

public class CommandMux {
    private CommandMuxEntry[] mux;

    public final int kInactive = -1;

    private int m_activeMux = kInactive;
    private int m_lastActiveMux = kInactive;

    public CommandMux(List<Config> configs) {
        mux = new CommandMuxEntry[configs.size()];
    }

    public CommandMux() {
        mux = new CommandMuxEntry[0];
    }

    public int getNumEntries() {
        return mux.length;
    }

    public CommandMuxEntry getEntry(int index) {
        return mux[index];
    }

    public void instantiateMux(int index, CommandMuxEntry entry) {
        mux[index] = entry;
    }

    public int getActive()
    {
        if (mux.length == 0) {
            return kInactive;
        }
        int priority = Integer.MAX_VALUE;
        m_activeMux = kInactive;
        for (int index = 0; index < mux.length; index++) {
            if (mux[index].isActive() && mux[index].getPriority() < priority) {
                priority = mux[index].getPriority();
                m_activeMux = mux[index].getID();
            }
        }
        return m_activeMux;
    }

    public void activate(int muxId) {
        if (muxId == kInactive) {
            // disable the mux and respond to any command that comes in
            m_activeMux = muxId;
            m_lastActiveMux = muxId;
            return;
        }
        boolean activated = false;
        for (int index = 0; index < mux.length; index++) {
            if (mux[index].getID() == muxId) {
                activated = true;
                mux[index].activate();
                m_lastActiveMux = muxId;
            }
        }
        if (!activated) {
            System.out.println(String.format("Warning: %d was not activated", muxId));
        }
    }

    public int getLastActivated() {
        return m_lastActiveMux;
    }
}
