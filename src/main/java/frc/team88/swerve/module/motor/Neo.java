package frc.team88.swerve.module.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.team88.swerve.configuration.subconfig.NeoConfiguration;
import java.util.Objects;

/** SwerveMotor implementation for the NEO. */
public class Neo implements SwerveMotor {

  // The Spark Max that is being wrapped.
  private final CANSparkMax spark;

  // The configuration data for this motor.
  private final NeoConfiguration config;

  // The last commanded velocity.
  private double commandVelocity = 0;

  // The offset to add to position values, in rotations.
  private double offset = 0;

  /**
   * Constructor. Sets up the default configuration for a Spark Max.
   *
   * @param canID The canID for the Spark Max.
   * @param config The config data for this motor.
   */
  public Neo(int canID, NeoConfiguration config) {
    spark = new CANSparkMax(canID, MotorType.kBrushless);

    this.config = Objects.requireNonNull(config);

    this.spark.restoreFactoryDefaults();
    this.spark.setIdleMode(IdleMode.kBrake);
    this.spark.setInverted(config.isInverted());
  }

  /**
   * {@inheritDoc}
   *
   * @return The motor shaft position in rotations
   */
  @Override
  public double getPosition() {
    return this.spark.getEncoder().getPosition() + offset;
  }

  @Override
  public double getVelocity() {
    return this.spark.getEncoder().getVelocity() / 60.;
  }

  /**
   * Offsets the sensor position such that the current position becomes the given position.
   *
   * @param position The position to set, in rotations
   */
  public void calibratePosition(double position) {
    this.offset = position - this.getPosition() + this.offset;
  }

  @Override
  public void setVelocity(double velocity) {
    this.spark.set(velocity / this.getMaxVelocity());
    this.commandVelocity = velocity;
  }

  @Override
  public double getMaxVelocity() {
    return this.config.getMaxSpeed();
  }

  @Override
  public double getCurrentDraw() {
    return this.getCurrentDraw();
  }

  @Override
  public double getCommandVoltage() {
    return this.spark.getAppliedOutput() * this.spark.getBusVoltage();
  }

  @Override
  public double getCommandVelocity() {
    return this.commandVelocity;
  }

  @Override
  public void setCoast() {
    this.spark.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void setBrake() {
    this.spark.setIdleMode(IdleMode.kBrake);
  }
}
