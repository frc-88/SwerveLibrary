package frc.team88.swerve.configuration;

import com.electronwill.nightconfig.core.Config;
import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;
import java.util.Objects;

/** Captures all of the configuration information about a command mux. */
public class CommandMuxConfiguration implements NetworkTablePopulator {

  // Configuration values. See getters for documentation.
  private int priority;

  // Identifing number of the command mux
  private int muxId;

  // Number of seconds before command is relinquished to the next mux
  private double timeout;

  // Name of template this command mux is based off of
  private String templateName;
  
  /**
   * Constructs this configuration from a command mux config.
   *
   * @param config The instantiated command mux template.
   */
  public CommandMuxConfiguration(Config config, String templateName) {
    Objects.requireNonNull(config);
    this.priority = config.get("priority");
    this.muxId = config.get("id");
    this.timeout = config.get("timeout");
    this.templateName = templateName;
    if (this.muxId < 0) {
      throw new IllegalArgumentException(
            String.format("Can't have a command mux with a negative ID: %d", this.muxId));
    }
  }

  /**
   * Gets the command mux priority. Lower numbers are higher priority
   *
   * @return priority of the command mux
   */
  public int getPriority() {
    return this.priority;
  }

  /**
   * Gets the command mux ID
   *
   * @return command mux ID
   */
  public int getID() {
    return this.muxId;
  }

  /**
   * Gets the command mux timeout
   *
   * @return command mux timeout
   */
  public double getTimeout() {
    return this.timeout;
  }

  /**
   * Gets the command mux template name
   *
   * @return command mux template name
   */
  public String getTemplateName() {
    return this.templateName;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    table.getEntry("id").setNumber(this.muxId);
    table.getEntry("priority").setNumber(this.priority);
    table.getEntry("timeout").setNumber(this.timeout);
    table.getEntry("name").setString(this.templateName);
  }
}
