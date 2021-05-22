package frc.team88.swerve.motion.state;

import edu.wpi.first.networktables.NetworkTable;
import frc.team88.swerve.data.NetworkTablePopulator;

/** Represents the state of the robot as determined by the odometry, both position and velocity. */
public class OdomState implements NetworkTablePopulator {
  private double xPosition = 0.0;
  private double yPosition = 0.0;
  private double theta = 0.0;

  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double thetaVelocity = 0.0;

  // Default constructor.

  /**
   * Gets the x position.
   *
   * @return The x position, in feet.
   */
  public double getXPosition() {
    return this.xPosition;
  }

  /**
   * Gets the y position.
   *
   * @return The y position, in feet.
   */
  public double getYPosition() {
    return this.yPosition;
  }

  /**
   * Sets the position.
   *
   * @param x The x positon, in feet.
   * @param y The y position, in feet.
   */
  public void setPosition(double x, double y) {
    this.xPosition = x;
    this.yPosition = y;
  }

  /**
   * Adds the given values to the position.
   *
   * @param x The x positon to add, in feet.
   * @param y The y position to add, in feet.
   */
  public void addToPosition(double x, double y) {
    this.xPosition += x;
    this.yPosition += y;
  }

  /**
   * Gets the heading.
   *
   * @return The heading, in degrees.
   */
  public double getTheta() {
    return this.theta;
  }

  /**
   * Sets the heading.
   *
   * @param theta The heading, in degrees.
   */
  public void setTheta(double theta) {
    this.theta = theta;
  }

  /**
   * Adds the given value the heading.
   *
   * @param theta The heading to add, in degrees.
   */
  public void addToTheta(double theta) {
    this.theta += theta;
  }

  /**
   * Gets the x velocity.
   *
   * @return The x velocity, in feet per second.
   */
  public double getXVelocity() {
    return this.xVelocity;
  }

  /**
   * Gets the y velocity.
   *
   * @return The y velocity, in feet per second.
   */
  public double getYVelocity() {
    return this.yVelocity;
  }

  /**
   * Sets the velocity.
   *
   * @param vx The x velocity, in feet per second.
   * @param vy The y velocity, in feet per second.
   */
  public void setVelocity(double vx, double vy) {
    this.xVelocity = vx;
    this.yVelocity = vy;
  }

  /**
   * Gets the heading angular velocity.
   *
   * @return The heading angular velocity, in degrees per second.
   */
  public double getThetaVelocity() {
    return this.thetaVelocity;
  }

  /**
   * Sets the heading angular velocity.
   *
   * @param vtheta The heading angular velocity, in degrees per second.
   */
  public void setThetaVelocity(double vtheta) {
    this.thetaVelocity = vtheta;
  }

  @Override
  public void populateNetworkTable(NetworkTable table) {
    table.getEntry("xPosition").setDouble(this.xPosition);
    table.getEntry("yPosition").setDouble(this.yPosition);
    table.getEntry("theta").setDouble(this.theta);
    table.getEntry("xVelocity").setDouble(this.xVelocity);
    table.getEntry("yVelocity").setDouble(this.yVelocity);
    table.getEntry("thetaVelocity").setDouble(this.thetaVelocity);
  }
}
