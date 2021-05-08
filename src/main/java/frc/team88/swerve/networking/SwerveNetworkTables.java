package frc.team88.swerve.networking;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Timer;
import frc.team88.swerve.motion.SwerveChassis;
import frc.team88.swerve.motion.state.OdomState;


public class SwerveNetworkTables {
    SwerveChassis chassis;
    
    NetworkTable odom_table;
    NetworkTableEntry odom_time;
    NetworkTableEntry odom_x;
    NetworkTableEntry odom_y;
    NetworkTableEntry odom_t;
    NetworkTableEntry odom_vx;
    NetworkTableEntry odom_vy;
    NetworkTableEntry odom_vt;

    NetworkTable module_table;

    NetworkTable command_table;
    NetworkTableEntry command_time;
    NetworkTableEntry command_linear_x;
    NetworkTableEntry command_linear_y;
    NetworkTableEntry command_angular_z;

    public SwerveNetworkTables(SwerveChassis chassis)
    {
        this.chassis = chassis;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.setUpdateRate(0.01);
        NetworkTable table = inst.getTable("Swerve");

        odom_table = table.getSubTable("odom");
        odom_time = odom_table.getEntry("time");
        odom_x = odom_table.getEntry("x");
        odom_y = odom_table.getEntry("y");
        odom_t = odom_table.getEntry("t");
        odom_vx = odom_table.getEntry("vx");
        odom_vy = odom_table.getEntry("vy");
        odom_vt = odom_table.getEntry("vt");

        command_table = table.getSubTable("command");
        command_time = command_table.getEntry("time");
        command_linear_x = command_table.getEntry("linear_x");
        command_linear_y = command_table.getEntry("linear_y");
        command_angular_z = command_table.getEntry("angular_z");

        module_table = table.getSubTable("modules");
    }

    public void publishSwerve()
    {
        long timestamp = RobotController.getFPGATime();
        OdomState state = chassis.getOdomState();
        setOdom(timestamp, state);

        for (int index = 0; index < chassis.getNumModules(); index++) {
            NetworkTable module_subtable = module_table.getSubTable(Integer.toString(index + 1));
            NetworkTableEntry module_wheel = module_subtable.getEntry("wheel");
            NetworkTableEntry module_azimuth = module_subtable.getEntry("azimuth");

            module_wheel.setDouble(chassis.getModule(index).getWheelSpeed());
            module_azimuth.setDouble(chassis.getModule(index).getAzimuthPosition().asDouble());
        }
    }

    public void setOdom(double timestamp, OdomState state)
    {
        odom_time.setDouble(timestamp);
        odom_x.setDouble(state.x);
        odom_y.setDouble(state.y);
        odom_t.setDouble(state.t);
        odom_vx.setDouble(state.vx);
        odom_vy.setDouble(state.vy);
        odom_vt.setDouble(state.vt);
    }

    public long getCmdTime()
    {
        return command_time.getNumber(0).longValue();
    }

    public double getLinearXCmd()
    {
        return command_linear_x.getDouble(0.0);
    }

    public double getLinearYCmd()
    {
        return command_linear_y.getDouble(0.0);
    }

    public double getAngularZCmd()
    {
        return command_angular_z.getDouble(0.0);
    }

    public boolean didCmdUpdate()
    {
        double dt = Timer.getFPGATimestamp() - command_time.getDouble(0.0);
        return dt < 0.5;
    }
}
