package frc.team88.swerve.data;

import edu.wpi.first.networktables.NetworkTable;

/**
 * Represents a data class that can populate a network table with values.
 */
public interface NetworkTablePopulator {
    

    /**
     * Populates the given network table with data from this class.
     * 
     * @param table The table to populate.
     */
    public void populateNetworkTable(NetworkTable table);

}
