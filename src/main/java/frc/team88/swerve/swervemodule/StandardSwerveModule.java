package frc.team88.swerve.swervemodule;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.team88.swerve.Constants;
import frc.team88.swerve.util.MathUtils;
import frc.team88.swerve.util.SyncPIDController;
import frc.team88.swerve.wrappers.SwerveAzimuthSensor;
import frc.team88.swerve.wrappers.SwerveMotor;

/**
 * Encapsulates the hardware interface for a standard Swerve Module.
 */
public class StandardSwerveModule implements SwerveModule {

    // Hardware device objects
    private SwerveMotor wheelMotor;
    private SwerveMotor azimuthMotor;
    private SwerveAzimuthSensor azimuthSensor;

    // Ratios
    private double wheelRatio;
    private double azimuthRatio;


    // PID
    private SyncPIDController wheelSpeedPID;
    private SyncPIDController azimuthPID;

    private double azimuthOffset = 0;

    public StandardSwerveModule(SwerveMotor wheelMotor, double wheelRatio, 
            SwerveMotor azimuthMotor, double azimuthRatio, SwerveAzimuthSensor azimuthSensor) {

        // Collect parameters
        this.wheelMotor = wheelMotor;
        this.wheelRatio = wheelRatio;
        this.azimuthMotor = azimuthMotor;
        this.azimuthRatio = azimuthRatio;
        this.azimuthSensor = azimuthSensor;

        // Set up PIDs
        wheelSpeedPID = new SyncPIDController(
            Constants.mk2WheelKP,
            Constants.mk2WheelKI,
            Constants.mk2WheelKD,
            Constants.mk2WheelKF,
            Constants.mk2WheelIZone,
            Constants.mk2WheelIMax);

        azimuthPID = new SyncPIDController(
            Constants.mk2AzimuthKP,
            Constants.mk2AzimuthKI,
            Constants.mk2AzimuthKD);
    }

    /**
     * Sets the offset of the azbsolutw azimuth sensor.
     * 
     * @param offset the ammount of degrees to add to the azimuth absolute sensor reading.
     */
    public void setAzimuthOffset(double offset) {
        this.azimuthOffset = offset;
    }

    @Override
    public void set(double wheelSpeed, double azimuth) {

        // Set the wheel angle to the closest multiple of 180

        // Set the wheel speed
        wheelMotor.setPercentVoltage(wheelSpeedPID.calculateOutput(
            wheelMotor.getSpeed(), wheelSpeed * wheelRatio));

        // Set the azimuth
        azimuthMotor.setPercentVoltage(azimuthPID.calculateOutput(
            azimuthSensor.getPosition(), azimuth * azimuthRatio));
                
    }

    // double currentAzimuth = azimuthEncoder.getPosition();
    //     double targetAzimuth = azimuth % 180;
    //     while (Math.abs(targetAzimuth - currentAzimuth) > 90) {
    //         if (targetAzimuth < currentAzimuth) {
    //             targetAzimuth += 180;
    //         } else {
    //             targetAzimuth -= 180;
    //         }
    //     }
    //     if (Math.abs(MathUtils.getReferenceAngle(azimuth) - MathUtils.getReferenceAngle(targetAzimuth)) > 90) {
    //         wheelSpeed = -wheelSpeed;
    //     }

    @Override
    public double getWheelSpeed() {
        return wheelMotor.getSpeed() / wheelRatio;
    }

    @Override
    public double getAzimuth() {
        return azimuthSensor.getPosition();
    }

    @Override
    public double getAzimuthVelocity() {
        return azimuthSensor.getSpeed();    
    }
}